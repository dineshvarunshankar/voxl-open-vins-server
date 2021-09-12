/*******************************************************************************
 * Copyright 2021 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

// Project Includes
#include "config_parser.h"

// Includes from open_vins
#include <core/VioManagerOptions.h>
#include <core/VioManager.h>
#include <utils/quat_ops.h>
#include <utils/sensor_data.h>
#include <state/State.h>
#include <utils/print.h>

// ModalAI Includes
#include <modal_pipe_server.h>
#include <modal_pipe_client.h>
#include <modal_start_stop.h>
#include <modal_pipe_interfaces.h>

// Other Libs 
#include <Eigen/Eigen>

// C/C++ Includes
#include <iostream>
#include <thread>
#include <deque>
#include <getopt.h>

/** The name of this app.  This is the binary name
 */
#define PROCESS_NAME "voxl-open-vins-server"

/** The non-configurable (aka hard-coded) pipe information for the client pipes (aka the pipes this 
 *  application gets data from).
 */
#define IMU_CH                 (0)
#define IMU_BUFFER_LEN         (40 * 10)
#define CAMERA_CH_START_OFFSET (1)

/** The non-configurable (aka hard-coded) pipe information for the server pipes (aka the pipes this 
 *  application publishes data to).
 */
#define SIMPLE_OUTPUT_CH (1)
#define VIO_SIMPLE_NAME        "open-vins"
#define VIO_SIMPLE_LOCATION    MODAL_PIPE_DEFAULT_BASE_DIR VIO_SIMPLE_NAME "/"

/** The configs of the system that are loaded from the config file
 */
ConfigParser::VIOConfigs configs;

/** The VIO Manager that we will be using for the VIO estimation
 */
std::unique_ptr<ov_msckf::VioManager> vio_manager;
std::mutex vio_manager_mutex;

/** The thread for the publishing thread.
 */
std::thread publish_vio_data_thread;

/** If we got IMU data. We shouldnt ingest camera data until we 
 */
std::atomic<bool> got_imu_data{false};

/** Atomic bool used to signal that threads should be shutdown
 *  This is because the "main_running" variable is just an int and is only implicitly atomic
 */
std::atomic<bool> shutdown_threads{false};

/** A buffer of IMU data so that we can find the IMU packet that is closest to the
 * current VIO data frame so we can fill in the angular velocities
 */
std::deque<imu_data_t> imu_data;
std::mutex imu_data_mutex;

/** Last angular velocity we sent.
 */
float last_angular_velocity_data[3];

/** The variables we need for computing the initial gyro bias
 */
#define GYRO_BIAS_COUNTER_THRESHOLD (100)
int initial_gyro_bias_counter{0};
double initial_gyro_bias[3];
std::atomic<bool> gyro_calibrated{false};

/** Command Line options
 */
bool config_only{false};
bool enable_debug_mode{false};
int8_t verbosity_level{static_cast<uint8_t>(ov_core::Printer::PrintLevel::SILENT)};
std::string config_file_to_load{"/home/root/voxl_open_vins/config.json"};

/** Handle new camera data
 *
 * @param ch The channel the data was from
 * @param data The imu data
 * @param bytes The number of bytes in the data array
 * @param context The context for the camera data
 */
static void _new_imu_data_handler(int ch, char* data, int bytes, void* context)
{
    // Unused parameters
    (void)ch;
    (void)context;

    // If the vio manager is not yet created then we dont need to ingest any data
    if(!vio_manager)
    {
        return;
    }

    // Unpack the IMu data into the imu struct
    int n_packets;
    imu_data_t* unpacked_imu_data = pipe_validate_imu_data_t(data, bytes, &n_packets);

    // If there is no data to unpack
    if(unpacked_imu_data == nullptr)
    {
        return;
    }

    // If the gyros are not calibrated for initial bias then do that first before ingesting data
    if(!gyro_calibrated)
    {
        for(int i = 0; i < n_packets;i++)
        {
            if(initial_gyro_bias_counter == 0)
            {
                initial_gyro_bias[0] = static_cast<double>(unpacked_imu_data[i].gyro_rad[0]);
                initial_gyro_bias[1] = static_cast<double>(unpacked_imu_data[i].gyro_rad[1]);
                initial_gyro_bias[2] = static_cast<double>(unpacked_imu_data[i].gyro_rad[2]); 
                initial_gyro_bias_counter++;    
                continue;
            }

            initial_gyro_bias[0] += static_cast<double>(unpacked_imu_data[i].gyro_rad[0]);
            initial_gyro_bias[1] += static_cast<double>(unpacked_imu_data[i].gyro_rad[1]);
            initial_gyro_bias[2] += static_cast<double>(unpacked_imu_data[i].gyro_rad[2]);
            initial_gyro_bias_counter++;
        }

        // IF we have enough data then compute the bias and say that we are calibrated
        if(initial_gyro_bias_counter > GYRO_BIAS_COUNTER_THRESHOLD)
        {
            initial_gyro_bias[0] /= static_cast<double>(initial_gyro_bias_counter);
            initial_gyro_bias[1] /= static_cast<double>(initial_gyro_bias_counter);
            initial_gyro_bias[2] /= static_cast<double>(initial_gyro_bias_counter);

            // Mark 
            gyro_calibrated = true;
        }

        return;
    }


    std::lock_guard<std::mutex> lg(vio_manager_mutex);
    std::lock_guard<std::mutex> lg2(imu_data_mutex);

    for(int i = 0; i < n_packets;i++)
    {
        // Create the data struct that we will use for ingesting data into the vio manager
        ov_core::ImuData vio_manager_data;
        vio_manager_data.timestamp = static_cast<double>(unpacked_imu_data[i].timestamp_ns) / 1000000000.0;

        vio_manager_data.wm(0,0) = static_cast<double>(unpacked_imu_data[i].gyro_rad[0]) - initial_gyro_bias[0];
        vio_manager_data.wm(1,0) = static_cast<double>(unpacked_imu_data[i].gyro_rad[1]) - initial_gyro_bias[1];
        vio_manager_data.wm(2,0) = static_cast<double>(unpacked_imu_data[i].gyro_rad[2]) - initial_gyro_bias[2];

        vio_manager_data.am(0,0) = unpacked_imu_data[i].accl_ms2[0];
        vio_manager_data.am(1,0) = unpacked_imu_data[i].accl_ms2[1];
        vio_manager_data.am(2,0) = unpacked_imu_data[i].accl_ms2[2];
        vio_manager->feed_measurement_imu(vio_manager_data);

        // Push the IMU packet
        imu_data.push_back(unpacked_imu_data[i]);

        // Signal that we got IMU data
        got_imu_data = true;
    }
    return;
}

/** Handle new camera data
 *
 * @param ch The channel the data was from
 * @param meta The metadata for the camera data
 * @param frame The camera frame to process
 * @param context The context for the camera data
 */
static void _new_camera_data_handler(int ch, camera_image_metadata_t meta, char* frame, void* context)
{    
    // Unused parameters
    (void)context;

    // Get the index of the camera in the configs so we can pass in the camera correctly into the system
    int camera_index = ch - CAMERA_CH_START_OFFSET;

    if(!got_imu_data)
    {
        return;
    }

    // If the vio manager is not yet created then we dont need to ingest any datal
    if(!vio_manager)
    {
        return;
    }

    // Do nothing until we have calibrated the gyro
    if(!gyro_calibrated)
    {
        return;
    }

    // Extract the camera configs
    ConfigParser::CameraConfigs camera_configs = configs.camera_configs[camera_index];

    // Unpack the data into an opencv image Mat
    cv::Mat img(meta.height, meta.width, CV_8UC1);
    std::memcpy(img.data, frame, img.cols*img.rows);

    // Create a mask for the ingestion.  We want the full image to be ingested
    cv::Mat mask(meta.height, meta.width, CV_8UC1, cv::Scalar(1));

    /* Create the data struct that we will use for ingesting data into the vio manager
     * Note: If multiple images are added in the same struct, they are treated as pairwise stereo images
     * and so will need hardware image capture synchronization 
     */
    ov_core::CameraData vio_manager_data;
    vio_manager_data.timestamp = static_cast<double>(meta.timestamp_ns) / 1000000000.0;
    vio_manager_data.sensor_ids.push_back(camera_configs.camera_id);
    vio_manager_data.images.push_back(img);
    vio_manager_data.masks.push_back(mask);

    // Ingest the data
    std::lock_guard<std::mutex> lg(vio_manager_mutex);
    vio_manager->feed_measurement_camera(vio_manager_data);

    return;
}

/** Publish the VIO data to the pipes
 * 
 *  @param millisecond_wait_time The amount of time to sleep on each loop execution
 */
static void _publish_vio_data(uint32_t millisecond_wait_time)
{
    // Loop until shutdown signaled
    while (!shutdown_threads)
    {
        // Sleep so we dont loop this too fast 
        std::this_thread::sleep_for(std::chrono::milliseconds(millisecond_wait_time));

        // If there is no vio manager then there is nothing to publish..
        if(!vio_manager)
        {
            continue;
        }

        //  We want to fill in the data
        vio_data_t vio_data;
        vio_data.magic_number = VIO_MAGIC_NUMBER;

        // The gravity vector is fixed since the vio frame is already gravity aligned
        vio_data.gravity_vector[0] = 0;
        vio_data.gravity_vector[1] = 0;
        vio_data.gravity_vector[2] = -1;

        // Set the quality to be fixed to a positive number
        vio_data.quality = 1;

        // Load the data into the struct that we will be publishing
        {
            std::lock_guard<std::mutex> lg(vio_manager_mutex);
            std::lock_guard<std::mutex> lg2(imu_data_mutex);

            // Grab the current state
            std::shared_ptr<ov_msckf::State> current_state = vio_manager->get_state();

            vio_data.timestamp_ns = static_cast<int64_t>(current_state->_timestamp * 1e9);
            Eigen::MatrixXf::Map(vio_data.T_imu_wrt_vio, 3, 1) = current_state->_imu->pos().cast<float>();
            Eigen::MatrixXf::Map(reinterpret_cast<float *>(vio_data.R_imu_to_vio), 3, 3) = current_state->_imu->Rot().cast<float>();
            Eigen::MatrixXf::Map(vio_data.vel_imu_wrt_vio, 3, 1) = current_state->_imu->vel().cast<float>();
            Eigen::MatrixXf::Map(vio_data.T_cam_wrt_imu, 3, 1) = current_state->_calib_IMUtoCAM[0]->pos().cast<float>();
            Eigen::MatrixXf::Map(reinterpret_cast<float *>(vio_data.R_cam_to_imu), 3, 3) = ov_core::quat_2_Rot(current_state->_calib_IMUtoCAM[0]->quat()).cast<float>();
            vio_data.n_feature_points = current_state->_features_SLAM.size();

            // Set the flag.  Note we cant say when we have a bad VIO state
            // @todo Figure out a way to set the bad VIO state
            if(vio_manager->initialized())
            {
                vio_data.state = VIO_STATE_OK;
            }
            else
            {
                vio_data.state = VIO_STATE_INITIALIZING;
            }

            // find the closest IMU packet and use that
            if(imu_data.empty())
            {
                // Oh No we have a problem, use the last angular velocity given
                vio_data.imu_angular_vel[0] = last_angular_velocity_data[0];
                vio_data.imu_angular_vel[1] = last_angular_velocity_data[1];
                vio_data.imu_angular_vel[2] = last_angular_velocity_data[2];
            }
            else
            {
                // Find the closest IMU element
                imu_data_t closest_imu_data;
                while(!imu_data.empty())
                {
                    // Get the head of the queue
                    imu_data_t first_imu_data = imu_data.front();
                    imu_data.pop_front();

                    // If there is no imu data left in the queue then we have the closet IMU data
                    if(imu_data.empty())
                    {
                        std::memcpy(&closest_imu_data, &first_imu_data, sizeof(imu_data_t));
                        break;
                    }

                    // If there is data in the queue after popping the head of the queue then we need to check the head of the next element

                    // Compute the diffs between timestamps
                    int64_t imu_data_time_diff1 = std::abs(static_cast<int64_t>(first_imu_data.timestamp_ns - vio_data.timestamp_ns));
                    int64_t imu_data_time_diff2 = std::abs(static_cast<int64_t>(imu_data.front().timestamp_ns - vio_data.timestamp_ns));

                    /* If the first element has a smaller diff than the next element then we found the closet element
                     * otherwise we need to pop the element off the queue and check the next 2 elements for the closest one
                     */
                    if(imu_data_time_diff1 < imu_data_time_diff2)
                    {
                        std::memcpy(&closest_imu_data, &first_imu_data, sizeof(imu_data_t));
                        break;
                    }
                }
                
                // Add the angular velocities as the IMU data with estimated biases subtracted
                vio_data.imu_angular_vel[0] = closest_imu_data.gyro_rad[0] - static_cast<float>(current_state->_imu->bias_g()(0,0));
                vio_data.imu_angular_vel[1] = closest_imu_data.gyro_rad[1] - static_cast<float>(current_state->_imu->bias_g()(0,1));
                vio_data.imu_angular_vel[2] = closest_imu_data.gyro_rad[2] - static_cast<float>(current_state->_imu->bias_g()(0,2));

                // Update the last angular velocity we sent in case we dont get IMU data (but we should always get IMU data)
                last_angular_velocity_data[0] = vio_data.imu_angular_vel[0];
                last_angular_velocity_data[1] = vio_data.imu_angular_vel[1];
                last_angular_velocity_data[2] = vio_data.imu_angular_vel[2];
            }
        }

        // Send the data our the pipe
        pipe_server_write(SIMPLE_OUTPUT_CH,   (char*)&vio_data, sizeof(vio_data_t));
    }
}

/** Print the usage information for the application
 */
static void _print_usage(void)
{

    std::cout << "voxl-open-vins-server usually runs as a systemd background service.  This application" << std::endl;
    std::cout << "is a VIO implementation using the open_vins project. For debug purposes" << std::endl;
    std::cout << "purposes it can be started from the command line manually with any of the following" << std::endl;
    std::cout << "debug options. When started from the command line, voxl-open-vins-server will automatically" << std::endl;
    std::cout << "stop the background service so you don't have to stop it manually" << std::endl;
    std::cout << std::endl;
    std::cout << "-c, --config                     load the config file only, this will terminate the application" << std::endl;
    std::cout << "                                    after the config file is loaded" << std::endl;
    std::cout << "-d, --debug                      run in debug mode which computes everything even when there" << std::endl;
    std::cout << "                                    are no clients to receive the data" << std::endl;
    std::cout << "-l, --config_file_to_load        A config file to load if a custom config file should be used" << std::endl;
    std::cout << "                                    default: \"/home/root/voxl_open_vins/config.json\"" << std::endl;
    std::cout << "-v <number>, --verbose <number>  sets the verbosity level for debug information" << std::endl;
    std::cout << "                                    0 - ALL" << std::endl;
    std::cout << "                                    1 - DEBUG" << std::endl;
    std::cout << "                                    2 - INFO" << std::endl;
    std::cout << "                                    3 - WARNING" << std::endl;
    std::cout << "                                    4 - ERROR" << std::endl;
    std::cout << "                                    5 - SILENT" << std::endl;
    std::cout << "-h, --help               print this help message" << std::endl;
    std::cout << std::endl;
    return;
}

/** Parses out the
 *
 *  @param argc The number of arguments that exist
 *  @param argv The argument pointer
 *  @return true if the program should terminate
 */
static bool _parse_opts(int argc, char* argv[])
{
    static struct option long_options[] =
    {
        {"config",                no_argument,          0, 'c'},
        {"debug",                 no_argument,          0, 'd'},
        {"help",                  no_argument,          0, 'h'},
        {"verbose",               required_argument,    0, 'v'},
        {"config_file_to_load",   required_argument,    0, 'l'},
        {0, 0, 0, 0}
    };

    while (1)
    {
        int option_index = 0;
        int c = getopt_long(argc, argv, "cdhsv:", long_options, &option_index);

        // Detect the end of the options.
        if (c == -1)
        {
            break;
        }

        switch (c)
        {
        case 0:
            // for long args without short equivalent that just set a flag nothing left to do so just break.
            if (long_options[option_index].flag != 0) break;
            break;

        case 'c':
            config_only = true;
            break;

        case 'd':
            std::cout << "Enabling debug mode" << std::endl;
            enable_debug_mode = true;
            break;

        case 'h':
            _print_usage();
            return true;

        case 'v':
            verbosity_level = static_cast<uint8_t>(std::atoi(optarg));
            break;

        case 'l':
            config_file_to_load = std::string(optarg);
            break;

        default:
            // Print the usage if there is an incorrect command line option
            _print_usage();
            return true;
        }
    }

    return false;
}

/** Terminates the app cleanly.
 *  Call this instead of return when it's time to exit to cleans up everything
 *
 *  @param ret The return code to use when exiting
 */
static void _quit(int ret)
{
    // Signal for threads to shudown
    shutdown_threads = true;

    // Close all the open pipe connections
    pipe_server_close_all();
    pipe_client_close_all();

    // Make sure all the threads shutdown before moving on
    if(publish_vio_data_thread.joinable())
    {
        publish_vio_data_thread.join();
    }

    // Delete the vio manager so we can close cleanly and quickly
    {
        std::lock_guard<std::mutex> lg(vio_manager_mutex);
        vio_manager.reset(nullptr);
    }

    // Remove this process ID file from the filesystem so this app can run again latter
    remove_pid_file(PROCESS_NAME);

    // If we are exiting cleanly then say so
    if (ret == 0)
    {
        std::cout << "Exiting Cleanly" << std::endl;
    }

    // Exit with the return code
    exit(ret);
    return;
}

/** Create the server pipes to get data from
 */
static void create_server_pipes(void)
{
    // We enable the control flag for future use.
    // int flags = SERVER_FLAG_EN_CONTROL_PIPE;
    int flags = 0;

    // init simple pipe
    pipe_info_t info = 
    { 
        VIO_SIMPLE_NAME,            // name
        VIO_SIMPLE_LOCATION,        // location
        "vio_data_t",               // type
        PROCESS_NAME,               // server_name
        VIO_RECOMMENDED_PIPE_SIZE,  // size_bytes
        0                           // server_pid
    };

    if(pipe_server_create(SIMPLE_OUTPUT_CH, info, flags))
    {
        _quit(-1);
    }

    // add in optional fields to the info JSON file
    cJSON* json = pipe_server_get_info_json_ptr(SIMPLE_OUTPUT_CH);
    cJSON_AddStringToObject(json, "imu", configs.imu_name.c_str());
    for(size_t i = 0; i < configs.camera_configs.size();i++)
    {
        std::string key = "cam" + std::to_string(i);
        cJSON_AddStringToObject(json, key.c_str(), configs.camera_configs[i].camera_name.c_str());
    }
    pipe_server_update_info(SIMPLE_OUTPUT_CH);
    // pipe_server_set_available_control_commands(SIMPLE_OUTPUT_CH, CONTROL_COMMANDS);
}

/** Create the client pipes to get data from
 */
static void connect_client_pipes(void)
{
    // Connect to the IMU pipe
    {
        pipe_client_set_simple_helper_cb(IMU_CH, _new_imu_data_handler, NULL);
        int flags = CLIENT_FLAG_EN_SIMPLE_HELPER;
        if (pipe_client_open(IMU_CH, configs.imu_name.c_str(), PROCESS_NAME, flags, IMU_BUFFER_LEN) != 0)
        {
            exit(0);
        }
    }

    // Connect to all the camera pipes
    for(size_t i = 0; i < configs.camera_configs.size();i++)
    {
        int channel_number = CAMERA_CH_START_OFFSET + i;

        pipe_client_set_camera_helper_cb(channel_number, _new_camera_data_handler, NULL);
        int flags = CLIENT_FLAG_EN_CAMERA_HELPER;
        if (pipe_client_open(channel_number, configs.camera_configs[i].camera_name.c_str(), PROCESS_NAME, flags, 0) != 0)
        {
            exit(0);
        }
    }
}

/** The main function
 * 
 * @param argc Argument Count
 * @param argv Argument array
 */
int main(int argc, char *argv[])
{
    // Parse the command line options and terminate if the parser says we should terminate
    if (_parse_opts(argc, argv))
    {
        return -1;
    }

    // Set the debugging verbosity level of the open_vins app
    switch(verbosity_level)
    {
        case static_cast<uint8_t>(ov_core::Printer::PrintLevel::ALL):
            ov_core::Printer::setPrintLevel(ov_core::Printer::PrintLevel::ALL);
            break;
        case static_cast<uint8_t>(ov_core::Printer::PrintLevel::DEBUG):
            ov_core::Printer::setPrintLevel(ov_core::Printer::PrintLevel::DEBUG);
            break;
        case static_cast<uint8_t>(ov_core::Printer::PrintLevel::INFO):
            ov_core::Printer::setPrintLevel(ov_core::Printer::PrintLevel::INFO);
            break;
        case static_cast<uint8_t>(ov_core::Printer::PrintLevel::WARNING):
            ov_core::Printer::setPrintLevel(ov_core::Printer::PrintLevel::WARNING);
            break;
        case static_cast<uint8_t>(ov_core::Printer::PrintLevel::ERROR):
            ov_core::Printer::setPrintLevel(ov_core::Printer::PrintLevel::ERROR);
            break;
        case static_cast<uint8_t>(ov_core::Printer::PrintLevel::SILENT):
            ov_core::Printer::setPrintLevel(ov_core::Printer::PrintLevel::SILENT);
            break;
        default:
            std::cerr << "Unknown Print Level." << std::endl;
            _print_usage();
            _quit(-1);
    }

    // Init whatever variables need to be initialized
    last_angular_velocity_data[0] = 0;
    last_angular_velocity_data[1] = 0;
    last_angular_velocity_data[2] = 0;

    /* make sure another instance isn't running
     * if return value is -3 then a background process is running with
     * higher privaledges and we couldn't kill it, in which case we should
     * not continue or there may be hardware conflicts. If it returned -4
     * then there was an invalid argument that needs to be fixed.
     */
    if(kill_existing_process(PROCESS_NAME, 2.0) < -2)
    {
        std::cerr << "ERROR: could not kill existing process" << std::endl;
        _quit(-1);
    } 

    // start signal handler so we can exit cleanly
    if(enable_signal_handler() == -1)
    {
        std::cerr << "ERROR: failed to start signal handler" << std::endl;
        _quit(-1);
    }

    // we are starting so signal for the threads to keep running
    shutdown_threads = false;

    // Set the main running flag to 1 to indicate that we are running
    main_running = 1;

    /* make PID file to indicate your project is running
     * due to the check made on the call to rc_kill_existing_process() above
     * we can be fairly confident there is no PID file already and we can
     * make our own safely.
     */
    make_pid_file(PROCESS_NAME);

    // Load the config file
    ConfigParser config_parser(config_file_to_load);
    if(!config_parser.parse_file())
    {
        std::cerr << "ERROR: Could not load config file." << std::endl;
        _quit(-1);
    }

    configs = config_parser.get_configs();

    // Create the VIO Manager
    ov_msckf::VioManagerOptions vio_manager_options = config_parser.generate_open_vins_manager_options();
    vio_manager = std::unique_ptr<ov_msckf::VioManager>(new ov_msckf::VioManager(vio_manager_options));

    // If we are in config only mode then we are done here (after the files have been loaded)
    if(config_only)
    {
        // Print the configs only
        config_parser.print_configs();
        _quit(0);
    }

    // Create the server pipes
    create_server_pipes();

    // Connect to the client pipes and start getting data
    connect_client_pipes();

    // Start the read and publish thread
    publish_vio_data_thread = std::thread(_publish_vio_data, 30);

    // Run forever
    while (main_running == 1)
    {
        std::this_thread::sleep_for(std::chrono::nanoseconds(250));
    }

    // Shutdown Nicely
    _quit(0);

    return 0;
}
