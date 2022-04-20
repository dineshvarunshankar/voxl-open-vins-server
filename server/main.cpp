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

#include "config_file.h"

#include <core/VioManager.h>
#include <core/VioManagerOptions.h>
#include <state/State.h>
#include <utils/print.h>
#include <utils/quat_ops.h>
#include <utils/sensor_data.h>

#include <modal_pipe.h>

#include <Eigen/Eigen>

#include <getopt.h>

#include <deque> // this should be replaced with a ringbuffer
#include <iostream>
#include <thread>


#define PROCESS_NAME "voxl-open-vins-server"
#define IMU_CH (0)
#define CAMERA_CH_START_OFFSET (1)

#define SIMPLE_OUTPUT_CH (1)
#define VIO_SIMPLE_NAME "open-vins"
#define VIO_SIMPLE_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR VIO_SIMPLE_NAME "/"

#define CAM_READ_BUF_SIZE (1024 * 1024 * 32)



std::unique_ptr<ov_msckf::VioManager> vio_manager;
std::mutex vio_manager_mutex;

std::thread publish_vio_data_thread;
std::atomic<bool> got_imu_data{false};
std::atomic<bool> shutdown_threads{false};
// std::deque<imu_data_t> imu_data;
std::mutex imu_data_mutex;
float last_angular_velocity_data[3];

bool config_only{false};
bool enable_debug_mode{false};
int8_t verbosity_level{static_cast<uint8_t>(ov_core::Printer::PrintLevel::SILENT)};

// these are the last timestamps that have completely passed into
static volatile int64_t last_imu_timestamp_ns = 0;
static volatile int64_t last_cam_timestamp_ns = 0;

static int64_t _apps_time_monotonic_ns() {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts)) {
        fprintf(stderr, "ERROR calling clock_gettime\n");
        return -1;
    }
    return (int64_t)ts.tv_sec * 1000000000 + (int64_t)ts.tv_nsec;
}

static void _new_imu_data_handler(__attribute__((unused)) int ch, char* data, int bytes, __attribute__((unused)) void* context) {
    if (!vio_manager) return;

    int n_packets;
    imu_data_t* data_array = pipe_validate_imu_data_t(data, bytes, &n_packets);

    if (data_array == NULL) return;
    if (n_packets <= 0) return;

    std::lock_guard<std::mutex> lg(vio_manager_mutex);
    std::lock_guard<std::mutex> lg2(imu_data_mutex);

    for (int i = 0; i < n_packets; i++) {
        // Create the data struct that we will use for ingesting data into the vio manager
        ov_core::ImuData vio_manager_data;
        vio_manager_data.timestamp = data_array[i].timestamp_ns / 1000000000.0; // (seconds)

        vio_manager_data.wm(0, 0) = data_array[i].gyro_rad[0];
        vio_manager_data.wm(1, 0) = data_array[i].gyro_rad[1];
        vio_manager_data.wm(2, 0) = data_array[i].gyro_rad[2];

        vio_manager_data.am(0, 0) = data_array[i].accl_ms2[0];
        vio_manager_data.am(1, 0) = data_array[i].accl_ms2[1];
        vio_manager_data.am(2, 0) = data_array[i].accl_ms2[2];

        vio_manager->feed_measurement_imu(vio_manager_data);

        if (!got_imu_data) got_imu_data = true;
        last_imu_timestamp_ns = data_array[i].timestamp_ns;
    }
    return;
}

static void _new_camera_data_handler(int ch, camera_image_metadata_t meta, char* frame, __attribute__((unused)) void* context) {
    int camera_index = ch - CAMERA_CH_START_OFFSET;

    fprintf(stderr, "camera index: %d\n", camera_index);

    if (!got_imu_data) return;

    if (!vio_manager) return;

    int64_t cam_timestamp_ns = meta.timestamp_ns;
    cam_timestamp_ns += meta.exposure_ns / 2;

    if (cam_timestamp_ns < (_apps_time_monotonic_ns() - 3000000000)) {
        fprintf(stderr, "dropping old frame older than 3 seconds\n");
        return;
    }

    // don't let image go in until IMU has caught up
    // if cam-imu alignment is POSITIVE that means the camera timestamp is early
    // and the image was actually taken after the reported timestamp
    // need a way to get the cam imu dif
    while (last_imu_timestamp_ns < cam_timestamp_ns) {
        // don't get stuck here forever
        if (shutdown_threads) return;
        if (cam_timestamp_ns < (_apps_time_monotonic_ns() - 300000000)) {
            fprintf(stderr, "ERROR waited more than 0.3 seconds for imu to catch up, dropping frame\n");
            return;
        }
        usleep(5000);
    }

    // Unpack the data into an opencv image Mat
    cv::Mat img(meta.height, meta.width, CV_8UC1, frame);
    // Create a mask for the ingestion.  We want the full image to be ingested
    cv::Mat mask(meta.height, meta.width, CV_8UC1, cv::Scalar(0));

    /* Create the data struct that we will use for ingesting data into the vio manager
     * Note: If multiple images are added in the same struct, they are treated as pairwise stereo images
     * and so will need hardware image capture synchronization
     */
    ov_core::CameraData vio_manager_data;
    vio_manager_data.timestamp = cam_timestamp_ns / 1000000000.0;
    vio_manager_data.sensor_ids.push_back(camera_index);
    vio_manager_data.images.push_back(img);
    vio_manager_data.masks.push_back(mask);

    // Ingest the data
    std::lock_guard<std::mutex> lg(vio_manager_mutex);
    vio_manager->feed_measurement_camera(vio_manager_data);
    last_cam_timestamp_ns = cam_timestamp_ns;

    return;
}

static void _publish_vio_data(uint32_t millisecond_wait_time) {
    while (!shutdown_threads) {
        std::this_thread::sleep_for(std::chrono::milliseconds(millisecond_wait_time));

        // If there is no vio manager then there is nothing to publish..
        if (!vio_manager) continue;

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

            // check the latest image that its using
            cv::Mat tester_im = vio_manager->get_historical_viz_image();

            camera_image_metadata_t meta_;
            meta_.timestamp_ns = _apps_time_monotonic_ns();
            meta_.width = 640;
            meta_.height = 480;
            meta_.size_bytes = meta_.width * meta_.height * 3;
            meta_.stride = meta_.width * 3;
            meta_.format = IMAGE_FORMAT_RGB;

            pipe_server_write_camera_frame(SIMPLE_OUTPUT_CH + 1, meta_, (char*)tester_im.data);

            vio_data.timestamp_ns = static_cast<int64_t>(current_state->_timestamp * 1e9);
            vio_data.T_imu_wrt_vio[0] = current_state->_imu->pos()(0);
            vio_data.T_imu_wrt_vio[1] = current_state->_imu->pos()(1);
            vio_data.T_imu_wrt_vio[2] = current_state->_imu->pos()(2);

            Eigen::MatrixXf::Map(reinterpret_cast<float*>(vio_data.R_imu_to_vio), 3, 3) = current_state->_imu->Rot_fej().cast<float>();

            // vio_data.vel_imu_wrt_vio[0] = state_plus(7); // we do not estimate this...
            // vio_data.vel_imu_wrt_vio[1] = state_plus(8); // we do not estimate this...
            // vio_data.vel_imu_wrt_vio[2] = state_plus(9); // we do not estimate this...

            Eigen::MatrixXf::Map(vio_data.vel_imu_wrt_vio, 3, 1) = current_state->_imu->vel().cast<float>();
            Eigen::MatrixXf::Map(vio_data.T_cam_wrt_imu, 3, 1) = current_state->_calib_IMUtoCAM[0]->pos().cast<float>();
            Eigen::MatrixXf::Map(reinterpret_cast<float*>(vio_data.R_cam_to_imu), 3, 3) = ov_core::quat_2_Rot(current_state->_calib_IMUtoCAM[0]->quat()).cast<float>();

            vio_data.n_feature_points = current_state->_features_SLAM.size();

            // Set the flag.  Note we cant say when we have a bad VIO state
            // @todo Figure out a way to set the bad VIO state
            if (vio_manager->initialized()) {
                vio_data.state = VIO_STATE_OK;
            } else {
                vio_data.state = VIO_STATE_INITIALIZING;
            }
        }
        // Send the data our the pipe
        pipe_server_write(SIMPLE_OUTPUT_CH, (char*)&vio_data, sizeof(vio_data_t));
    }
}

/** Print the usage information for the application
 */
static void _print_usage(void) {
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

static bool _parse_opts(int argc, char* argv[]) {
    static struct option long_options[] =
        {
            {"config", no_argument, 0, 'c'},
            {"debug", no_argument, 0, 'd'},
            {"help", no_argument, 0, 'h'},
            {"verbose", required_argument, 0, 'v'},
            {"config_file_to_load", required_argument, 0, 'l'},
            {0, 0, 0, 0}};

    while (1) {
        int option_index = 0;
        int c = getopt_long(argc, argv, "cdhsv:", long_options, &option_index);

        // Detect the end of the options.
        if (c == -1) {
            break;
        }

        switch (c) {
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

static void _quit(int ret) {
    // Signal for threads to shudown
    shutdown_threads = true;

    // Close all the open pipe connections
    pipe_server_close_all();
    pipe_client_close_all();

    // Make sure all the threads shutdown before moving on
    if (publish_vio_data_thread.joinable()) {
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
    if (ret == 0) {
        std::cout << "Exiting Cleanly" << std::endl;
    }

    // Exit with the return code
    exit(ret);
    return;
}


static int create_server_pipes(void) {
    int flags = SERVER_FLAG_EN_CONTROL_PIPE;

    pipe_info_t info =
    {
        VIO_SIMPLE_NAME,            // name
        VIO_SIMPLE_LOCATION,        // location
        "vio_data_t",               // type
        PROCESS_NAME,               // server_name
        VIO_RECOMMENDED_PIPE_SIZE,  // size_bytes
        0                           // server_pid
    };

    if (pipe_server_create(SIMPLE_OUTPUT_CH, info, flags)) {
        return -1;
    }

    pipe_info_t info_ =
    {
        "open-vins-cam",                   // name
        "/run/mpa/open-vins-cam",          // location
        "camera_image_metadata_t",         // type
        PROCESS_NAME,                      // server_name
        VIO_RECOMMENDED_PIPE_SIZE * 1024,  // size_bytes
        0                                  // server_pid
    };

    if (pipe_server_create(SIMPLE_OUTPUT_CH + 1, info_, flags)) {
        return -1;
    }

    // // add in optional fields to the info JSON file
    // cJSON* json = pipe_server_get_info_json_ptr(SIMPLE_OUTPUT_CH);
    // cJSON_AddStringToObject(json, "imu", configs.imu_name.c_str());
    // for(size_t i = 0; i < configs.camera_configs.size();i++)
    // {
    //     std::string key = "cam" + std::to_string(i);
    //     cJSON_AddStringToObject(json, key.c_str(), configs.camera_configs[i].camera_name.c_str());
    // }
    // pipe_server_update_info(SIMPLE_OUTPUT_CH);
    // pipe_server_set_available_control_commands(SIMPLE_OUTPUT_CH, CONTROL_COMMANDS);

    return 0;
}

static int connect_client_pipes(void) {
    // connect to imu
    char full_pipe[CHAR_BUF_SIZE];
    if (pipe_expand_location_string(imu_name, full_pipe) < 0) {
        fprintf(stderr, "ERROR: unable to expand location string with imu %s\n", imu_name);
        return -1;
    }

    pipe_client_set_simple_helper_cb(IMU_CH, _new_imu_data_handler, NULL);
    int flags = CLIENT_FLAG_EN_SIMPLE_HELPER;
    if (pipe_client_open(IMU_CH, full_pipe, PROCESS_NAME, flags, IMU_RECOMMENDED_READ_BUF_SIZE) != 0) {
        return -1;
    }

    // Connect to all the camera pipes
    for (size_t i = 0; i < MAX_CAMERAS; i++) {
        if (cam_info_vec[i].enable){
            memset(full_pipe, '\0', CHAR_BUF_SIZE);
            int channel_number = CAMERA_CH_START_OFFSET + i;
            if (pipe_expand_location_string(cam_info_vec[i].name, full_pipe) < 0) {
                fprintf(stderr, "ERROR: unable to expand location string with camera %s\n", cam_info_vec[i].name);
                return -1;
            }

            pipe_client_set_camera_helper_cb(channel_number, _new_camera_data_handler, NULL);
            int flags = CLIENT_FLAG_EN_CAMERA_HELPER;
            if (pipe_client_open(channel_number, full_pipe, PROCESS_NAME, flags, 1024 * 1024 * 32) != 0) {
                fprintf(stderr, "ERROR: FAILED TO OPEN %s\n", full_pipe);
                return -1;
            }
        }
    }
    return 0;
}

static ov_msckf::VioManagerOptions generate_open_vins_manager_options() {
    // Create the VIO Manager Options (aka the settings for the manager)
    ov_msckf::VioManagerOptions vio_manager_options;

    /// STATE OPTIONS ///
    vio_manager_options.state_options.do_fej = true;
    vio_manager_options.state_options.imu_avg = true;
    vio_manager_options.state_options.use_rk4_integration = true;
    vio_manager_options.state_options.num_cameras = 1;
    vio_manager_options.state_options.do_calib_camera_pose = camera_to_imu_pose_calibration;
    vio_manager_options.state_options.do_calib_camera_intrinsics = camera_intrinsics_calibration;
    vio_manager_options.state_options.do_calib_camera_timeoffset = camera_imu_timestamp_calibration;
    vio_manager_options.state_options.max_clone_size = max_clone_size;
    vio_manager_options.state_options.max_slam_features = 50;
    vio_manager_options.state_options.max_slam_in_update = 25;
    vio_manager_options.state_options.max_msckf_in_update = 40;
    vio_manager_options.state_options.feat_rep_msckf = ov_type::LandmarkRepresentation::GLOBAL_3D;
    vio_manager_options.state_options.feat_rep_slam = ov_type::LandmarkRepresentation::ANCHORED_MSCKF_INVERSE_DEPTH;
    vio_manager_options.calib_camimu_dt = -0.002;
    vio_manager_options.dt_slam_delay = 1.0;

    /// INERTIAL INITIALIZER OPTIONS ///
    // only use: (params.gravity_mag, params.init_window_time, params.init_imu_thresh)
    vio_manager_options.gravity_mag = 9.81;
    vio_manager_options.init_window_time = 2.0;
    vio_manager_options.init_imu_thresh = 1.5;

    /// IMU NOISE OPTIONS ///
    // playing with these
    // NOTE - JAMES ONLY BUMP COVARIANCE (i.e _2 stats) NOT HZ
    vio_manager_options.imu_noises.sigma_w = 1.6968e-02 * 20;
    vio_manager_options.imu_noises.sigma_wb = 1.9393e-03 * 20;
    vio_manager_options.imu_noises.sigma_a = 2.0000e-1 * 20;
    vio_manager_options.imu_noises.sigma_ab = 3.0000e-01 * 20;

    vio_manager_options.imu_noises.sigma_w_2 = pow(1.6968e-02, 2);
    vio_manager_options.imu_noises.sigma_wb_2 = pow(1.9393e-03, 2);
    vio_manager_options.imu_noises.sigma_a_2 = pow(2.0000e-1, 2);
    vio_manager_options.imu_noises.sigma_ab_2 = pow(3.0000e-01, 2);

    // leaving default for now, values listed below
    // vio_manager_options.imu_noises.sigma_w = 1.6968e-04;
    // vio_manager_options.imu_noises.sigma_w_2 = pow(1.6968e-04, 2);
    // vio_manager_options.imu_noises.sigma_wb = 1.9393e-05;
    // vio_manager_options.imu_noises.sigma_wb_2 = pow(1.9393e-05, 2);
    // vio_manager_options.imu_noises.sigma_a = 2.0000e-3;
    // vio_manager_options.imu_noises.sigma_a_2 = pow(2.0000e-3, 2);
    // vio_manager_options.imu_noises.sigma_ab = 3.0000e-03;

    /// FEATURE OPTIONS - all use the same struct, can be dif per feature set ///
    // msckf
    vio_manager_options.msckf_options.chi2_multipler = 0.10;
    vio_manager_options.msckf_options.sigma_pix = 5;
    vio_manager_options.msckf_options.sigma_pix_sq = 25;
    // slam
    vio_manager_options.slam_options.chi2_multipler = 0.10;
    vio_manager_options.slam_options.sigma_pix = 5;
    vio_manager_options.slam_options.sigma_pix_sq = 25;

    /// ZUPT OPTIONS ///
    vio_manager_options.try_zupt = use_zupt;
    if (use_zupt) {
        vio_manager_options.zupt_max_velocity = zupt_max_velocity;
        vio_manager_options.zupt_only_at_beginning = zupt_only_at_beginning;
        vio_manager_options.zupt_noise_multiplier = zupt_noise_multiplier;
        vio_manager_options.zupt_max_disparity = zupt_max_disparity;
        vio_manager_options.zupt_options.chi2_multipler = 0;
    } else
        vio_manager_options.init_imu_thresh = init_imu_thresh;

    /// GENERAL OPTIONS ///
    vio_manager_options.use_stereo = false;
    vio_manager_options.use_mask = false;
    vio_manager_options.use_aruco = false;

    /// TRACKER + EXTRACTOR OPTIONS ///
    vio_manager_options.use_klt = true;
    vio_manager_options.num_pts = 200;
    vio_manager_options.fast_threshold = 20;
    vio_manager_options.grid_x = 20;
    vio_manager_options.grid_y = 20;
    vio_manager_options.min_px_dist = 10;
    vio_manager_options.knn_ratio = 0.70;
    vio_manager_options.downsample_cameras = downsample_cams;
    vio_manager_options.use_multi_threading = false;

    /// CAMERA INTRINSICS + EXTRINSICS ///
    for (size_t i = 0; i < 1; i++) {
        // Set the camera type
        // if (camera_config.camera_type == CameraType::TRACKING)
        // {
        vio_manager_options.camera_fisheye[0] = true;
        // }
        // else
        // {
        // std::cerr << "Camera type not supported in \"generate_open_vins_manager_options(...)\"" << std::endl;
        // exit(0);
        // }

        // Set the dims, if stereo then this would be the size of 1 image not both
        vio_manager_options.camera_wh[0] = std::make_pair(cam_info_vec[i].cam_calib_intrinsic(8, 0), cam_info_vec[i].cam_calib_intrinsic(9, 0));

        // The camera intrinsics
        Eigen::Matrix<double, 10, 1> cam_calib_intrinsic = cam_info_vec[0].cam_calib_intrinsic;
        vio_manager_options.camera_intrinsics[0] = cam_calib_intrinsic;
        vio_manager_options.camera_extrinsics[0] = cam_info_vec[0].cam_wrt_imu;
    }

    return vio_manager_options;
}


int main(int argc, char* argv[]) {
    // Parse the command line options and terminate if the parser says we should terminate
    if (_parse_opts(argc, argv)) {
        return -1;
    }

    // Set the debugging verbosity level of the open_vins app
    switch (verbosity_level) {
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
    if (kill_existing_process(PROCESS_NAME, 2.0) < -2) {
        std::cerr << "ERROR: could not kill existing process" << std::endl;
        _quit(-1);
    }

    // start signal handler so we can exit cleanly
    if (enable_signal_handler() == -1) {
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

    // Load the config files
    printf("Loading our own config file\n");
    if (config_file_read() < 0) _quit(-1);

    printf("Loading extrinsics config file\n");
    if (load_extrinsics_file() < 0) _quit(-1);

    printf("Loading intrinsics config file\n");
    if (load_intrinsics_file() < 0) _quit(-1);

    // Create the VIO Manager
    ov_msckf::VioManagerOptions vio_manager_options = generate_open_vins_manager_options();
    vio_manager = std::unique_ptr<ov_msckf::VioManager>(new ov_msckf::VioManager(vio_manager_options));

    // If we are in config only mode then we are done here (after the files have been loaded)
    if (config_only) {
        // Print the configs only
        config_file_print();
        _quit(0);
    }

    // Create the server pipes
    if (create_server_pipes() < 0) _quit(0);

    // Connect to the client pipes and start getting data
    if (connect_client_pipes() < 0) _quit(0);

    // Start the read and publish thread
    publish_vio_data_thread = std::thread(_publish_vio_data, 30);

    // Run forever
    while (main_running == 1) {
        std::this_thread::sleep_for(std::chrono::nanoseconds(250));
    }

    // Shutdown Nicely
    _quit(0);

    return 0;
}
