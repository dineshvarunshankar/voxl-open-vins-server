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

#include <core/VioManager.h>
#include <getopt.h>
#include <modal_pipe.h>
#include <utils/quat_ops.h>

#include <iostream>
#include <thread>

#include "config_file.h"
#include "rc_ov_ringbuf.h"

#define PROCESS_NAME "voxl-open-vins-server"
#define IMU_CH (0)
#define CAMERA_CH_START_OFFSET (1)

#define SIMPLE_OUTPUT_CH (1)
#define VIO_SIMPLE_NAME "open-vins"
#define VIO_SIMPLE_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR VIO_SIMPLE_NAME "/"

#define OVERLAY_OUTPUT_CH (2)
#define CAM_READ_BUF_SIZE (1024 * 1024 * 64)

std::unique_ptr<ov_msckf::VioManager> vio_manager;
std::mutex vio_manager_mutex;

std::thread publish_vio_data_thread;
rc_ov_ringbuf_t imu_buf = RC_OV_RINGBUF_INITIALIZER;
std::atomic<bool> is_imu_connected{false};
std::atomic<bool> is_cam_connected{false};

bool en_debug = false;
int8_t verbosity_level{static_cast<uint8_t>(ov_core::Printer::PrintLevel::SILENT)};

// these are the last timestamps that have completely passed into
static volatile int64_t last_imu_timestamp_ns = 0;
static volatile int64_t last_cam_timestamp_ns = 0;
static int64_t last_time_alignment_ns = 0;

static uint32_t global_error_codes = 0;
std::mutex cam_mutex;

static int64_t _apps_time_monotonic_ns() {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts)) {
        fprintf(stderr, "ERROR calling clock_gettime\n");
        return -1;
    }
    return (int64_t)ts.tv_sec * 1000000000 + (int64_t)ts.tv_nsec;
}

static void _nanosleep(uint64_t ns) {
    struct timespec req, rem;
    req.tv_sec = ns / 1000000000;
    req.tv_nsec = ns % 1000000000;
    // loop untill nanosleep sets an error or finishes successfully
    errno = 0;  // reset errno to avoid false detection
    while (nanosleep(&req, &rem) && errno == EINTR) {
        req.tv_sec = rem.tv_sec;
        req.tv_nsec = rem.tv_nsec;
    }
    return;
}

static void _new_imu_data_handler(__attribute__((unused)) int ch, char* data, int bytes, __attribute__((unused)) void* context) {
    int n_packets;
    imu_data_t* data_array = pipe_validate_imu_data_t(data, bytes, &n_packets);

    if (data_array == NULL) return;
    if (n_packets <= 0) return;

    is_imu_connected = 1;

    global_error_codes &= ~ERROR_CODE_IMU_MISSING;
    if (!is_cam_connected) return;
    if (!vio_manager) return;
    if (!main_running) return;

    std::lock_guard<std::mutex> lg(vio_manager_mutex);

    for (int i = 0; i < n_packets; i++) {
        // Create the data struct that we will use for ingesting data into the vio manager
        ov_core::ImuData vio_manager_data;
        vio_manager_data.timestamp = data_array[i].timestamp_ns / 1000000000.0;  // (seconds)

        vio_manager_data.wm(0, 0) = data_array[i].gyro_rad[0];
        vio_manager_data.wm(1, 0) = data_array[i].gyro_rad[1];
        vio_manager_data.wm(2, 0) = data_array[i].gyro_rad[2];

        vio_manager_data.am(0, 0) = data_array[i].accl_ms2[0];
        vio_manager_data.am(1, 0) = data_array[i].accl_ms2[1];
        vio_manager_data.am(2, 0) = data_array[i].accl_ms2[2];

        rc_ov_t imu_buf_packet = {data_array[i].gyro_rad[0], data_array[i].gyro_rad[1], data_array[i].gyro_rad[2], (int64_t)data_array[i].timestamp_ns};
        rc_ov_ringbuf_insert(&imu_buf, &imu_buf_packet);

        vio_manager->feed_measurement_imu(vio_manager_data);

        last_imu_timestamp_ns = data_array[i].timestamp_ns;
    }
    return;
}

static void _new_camera_data_handler(int ch, camera_image_metadata_t meta, char* frame, __attribute__((unused)) void* context) {
    int camera_index = ch - CAMERA_CH_START_OFFSET;

    // fprintf(stderr, "cam index: %d\n", camera_index);
    if (!vio_manager) return;
    if (!main_running) return;

    static int64_t last_cam_ts = meta.timestamp_ns;

    int64_t cam_timestamp_ns = meta.timestamp_ns;
    cam_timestamp_ns += meta.exposure_ns / 2;

    if (cam_timestamp_ns < (_apps_time_monotonic_ns() - 3000000000)) {
        fprintf(stderr, "dropping old frame older than 3 seconds\n");
        return;
    }
    if (cam_timestamp_ns < last_cam_ts) return;

    // flag that camera data is active, skip frame is imu is disconnected
    is_cam_connected = 1;
    global_error_codes &= ~ERROR_CODE_CAM_MISSING;
    if (!is_imu_connected) return;

    // don't let image go in until IMU has caught up
    // if cam-imu alignment is POSITIVE that means the camera timestamp is early
    // and the image was actually taken after the reported timestamp
    // need a way to get the cam imu dif
    while (last_imu_timestamp_ns < (cam_timestamp_ns)){ // + last_time_alignment_ns)) {
        // don't get stuck here forever
        if (!main_running) return;
        if (!is_imu_connected) return;
        if (cam_timestamp_ns < (_apps_time_monotonic_ns() - 300000000)) {
            fprintf(stderr, "ERROR waited more than 0.3 seconds for imu to catch up, dropping frame\n");
            return;
        }
        usleep(5000);
    }

    ov_core::CameraData vio_manager_data;

    vio_manager_data.timestamp = cam_timestamp_ns / 1000000000.0;
    vio_manager_data.sensor_ids.push_back(camera_index);

    if (meta.format == IMAGE_FORMAT_RAW8){
        // fprintf(stderr, "last tracking timestamp: %ld\n", meta.timestamp_ns);
        // Unpack the data into an opencv image Mat
        cv::Mat img(meta.height, meta.width, CV_8UC1, frame);
        // Create a mask for the ingestion.  We want the full image to be ingested
        cv::Mat mask(meta.height, meta.width, CV_8UC1, cv::Scalar(0));
        vio_manager_data.images.push_back(img);
        vio_manager_data.masks.push_back(mask);
    }
    else if (meta.format == IMAGE_FORMAT_STEREO_RAW8){
        // fprintf(stderr, "last stereo timestamp: %ld\n", meta.timestamp_ns);
        // stereo pairs take camera_index as l_cam index, r_cam index is actual_index+1
        vio_manager_data.sensor_ids.push_back(camera_index+1);

        // Unpack the data into opencv image Mats
        cv::Mat img(meta.height, meta.width, CV_8UC1, frame);
        cv::Mat img2(meta.height, meta.width, CV_8UC1, frame + (meta.width * meta.height));

        // Create masks for the ingestion. We want both full images to be ingested
        cv::Mat mask(meta.height, meta.width, CV_8UC1, cv::Scalar(0));
        cv::Mat mask2(meta.height, meta.width, CV_8UC1, cv::Scalar(0));

        vio_manager_data.images.push_back(img);
        vio_manager_data.masks.push_back(mask);
        vio_manager_data.images.push_back(img2);
        vio_manager_data.masks.push_back(mask2);
    }

    // Ingest the data
    std::lock_guard<std::mutex> lg(cam_mutex);
    std::lock_guard<std::mutex> lg2(vio_manager_mutex);
    vio_manager->feed_measurement_camera(vio_manager_data);
    last_cam_timestamp_ns = cam_timestamp_ns;
    last_cam_ts = cam_timestamp_ns;

    return;
}

static void _publish_vio_data() {
    int64_t next_time = _apps_time_monotonic_ns() + (int64_t)(1000000000 / odr_hz);

    while (main_running) {
        // try to maintain output data rate (odr)
        int64_t current_time = _apps_time_monotonic_ns();
        next_time += (int64_t)(1000000000.0f / odr_hz);
        // uh oh, we fell behind, warn and get back on track
        if (next_time < current_time) {
            // fprintf(stderr, "WARNING: output data thread fell behind\n");
            next_time = current_time + (int64_t)(1000000000.0f / odr_hz);
        }
        _nanosleep(next_time - current_time);
        // If there is no vio manager then there is nothing to publish..
        if (!vio_manager) continue;

        //  We want to fill in the data
        vio_data_t vio_data;
        vio_data.magic_number = VIO_MAGIC_NUMBER;
        vio_data.error_code = global_error_codes;

        // The gravity vector is fixed since the vio frame is already gravity aligned
        vio_data.gravity_vector[0] = 0;
        vio_data.gravity_vector[1] = 0;
        vio_data.gravity_vector[2] = -1;

        // Set the quality to be fixed to a positive number
        vio_data.quality = 1;

        // @todo Figure out a way to set the bad VIO state
        if (vio_manager->initialized()) {
            vio_data.state = VIO_STATE_OK;
        } else {
            vio_data.state = VIO_STATE_INITIALIZING;
            pipe_server_write(SIMPLE_OUTPUT_CH, (char*)&vio_data, sizeof(vio_data_t));
            continue;
        }

        // Load the data into the struct that we will be publishing
        {
            std::lock_guard<std::mutex> lg(vio_manager_mutex);

            // check the latest image that its using
            cv::Mat tester_im = vio_manager->get_historical_viz_image();

            if (tester_im.cols > 1280){
                cv::resize(tester_im, tester_im, cv::Size(), 0.5, 0.5);
            }

            camera_image_metadata_t meta_;
            meta_.timestamp_ns = _apps_time_monotonic_ns();
            meta_.width = tester_im.cols;
            meta_.height = tester_im.rows;
            // known to be rgb image regardless of input
            meta_.size_bytes = meta_.width * meta_.height * 3;
            meta_.stride = meta_.width * 3;
            meta_.format = IMAGE_FORMAT_RGB;

            pipe_server_write_camera_frame(SIMPLE_OUTPUT_CH + 1, meta_, (char*)tester_im.data);

            // Grab the current state
            std::shared_ptr<ov_msckf::State> current_state = vio_manager->get_state();

            vio_data.timestamp_ns = static_cast<int64_t>(current_state->_timestamp * 1e9);
            Eigen::MatrixXf::Map(vio_data.T_imu_wrt_vio, 3, 1) = current_state->_imu->pos().cast<float>();
            Eigen::MatrixXf::Map(reinterpret_cast<float*>(vio_data.R_imu_to_vio), 3, 3) = current_state->_imu->Rot().cast<float>();
            Eigen::MatrixXf::Map(vio_data.vel_imu_wrt_vio, 3, 1) = current_state->_imu->vel().cast<float>();
            Eigen::MatrixXf::Map(vio_data.T_cam_wrt_imu, 3, 1) = current_state->_calib_IMUtoCAM[0]->pos().cast<float>();
            Eigen::MatrixXf::Map(reinterpret_cast<float*>(vio_data.R_cam_to_imu), 3, 3) = ov_core::quat_2_Rot(current_state->_calib_IMUtoCAM[0]->quat()).cast<float>();
            vio_data.n_feature_points = current_state->_features_SLAM.size();

            Eigen::MatrixXf::Map(reinterpret_cast<float*>(vio_data.R_imu_to_vio), 3, 3) = current_state->_imu->Rot_fej().cast<float>();

            rc_ov_t closest_imu_packet;
            int ret = rc_ov_ringbuf_get_ov_at_time(&imu_buf, meta_.timestamp_ns, &closest_imu_packet);
            if (ret < 0) {
                fprintf(stderr, "ERROR fetching from ringbuffer\n");
                if (ret == -2) {
                    printf("there wasn't sufficient data in the buffer\n");
                }
                if (ret == -3) {
                    printf("the requested timestamp was too new\n");
                }
                if (ret == -4) {
                    printf("the requested timestamp was too old\n");
                }
                continue;
            }

            if (vio_data.state == VIO_STATE_OK){
                // update our last time_alignments, need to convert back down to nanoseconds
                last_time_alignment_ns = current_state->_calib_dt_CAMtoIMU->value()(0) * 1e9;
                // fprintf(stderr, "time align update: %ld\n", last_time_alignment_ns);
            }

            // Add the angular velocities as the IMU data with estimated biases subtracted
            vio_data.imu_angular_vel[0] = closest_imu_packet.last_angular_velocity_data[0] - static_cast<float>(current_state->_imu->bias_g()(0, 0));
            vio_data.imu_angular_vel[1] = closest_imu_packet.last_angular_velocity_data[1] - static_cast<float>(current_state->_imu->bias_g()(0, 1));
            vio_data.imu_angular_vel[2] = closest_imu_packet.last_angular_velocity_data[2] - static_cast<float>(current_state->_imu->bias_g()(0, 2));
        }
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

static void _quit(int ret) {
    // Signal for threads to shudown
    main_running = false;

    // Close all the open pipe connections
    pipe_server_close_all();
    pipe_client_close_all();

    // Make sure all the threads shutdown before moving on
    if (publish_vio_data_thread.joinable()) {
        publish_vio_data_thread.join();
    }

    // Delete the vio manager so we can close cleanly and quickly
    // braces here to set scope, lock guard returned after scope eds
    {
        std::lock_guard<std::mutex> lg(cam_mutex);
        std::lock_guard<std::mutex> lg2(vio_manager_mutex);
        vio_manager.reset(nullptr);
    }

    // Remove this process ID file
    remove_pid_file(PROCESS_NAME);

    // If we are exiting cleanly then say so
    if (ret == 0) std::cout << "Exiting Cleanly" << std::endl;
    exit(ret);
    return;
}

static bool _parse_opts(int argc, char* argv[]) {
    static struct option long_options[] =
        {
            {"config", no_argument, 0, 'c'},
            {"debug", no_argument, 0, 'd'},
            {"help", no_argument, 0, 'h'},
            {"verbose", required_argument, 0, 'v'},
            {0, 0, 0, 0}};
    ov_core::Printer::setPrintLevel(ov_core::Printer::PrintLevel::SILENT);
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
                config_file_read();
                config_file_print();
                _quit(0);
                break;

            case 'd':
                std::cout << "Enabling debug mode" << std::endl;
                en_debug = true;
                break;

            case 'h':
                _print_usage();
                return true;

            case 'v':
                verbosity_level = static_cast<uint8_t>(std::atoi(optarg));
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
                break;

            default:
                _print_usage();
                return true;
        }
    }
    return false;
}

static ov_msckf::VioManagerOptions generate_open_vins_manager_options() {
    ov_msckf::VioManagerOptions vio_manager_options;

    /// STATE OPTIONS ///
    vio_manager_options.state_options.do_fej = do_fej;
    vio_manager_options.state_options.imu_avg = imu_avg;
    vio_manager_options.state_options.use_rk4_integration = use_rk4_integration;
    vio_manager_options.state_options.do_calib_camera_pose = cam_to_imu_refinement;
    vio_manager_options.state_options.do_calib_camera_intrinsics = cam_intrins_refinement;
    vio_manager_options.state_options.do_calib_camera_timeoffset = cam_imu_ts_refinement;
    vio_manager_options.state_options.max_clone_size = max_clone_size;
    vio_manager_options.state_options.max_slam_features = max_slam_features;
    vio_manager_options.state_options.max_slam_in_update = max_slam_in_update;
    vio_manager_options.state_options.max_msckf_in_update = max_msckf_in_update;
    vio_manager_options.state_options.feat_rep_msckf = feat_rep_msckf;
    vio_manager_options.state_options.feat_rep_slam = feat_rep_slam;
    vio_manager_options.calib_camimu_dt = cam_imu_time_offset;
    vio_manager_options.dt_slam_delay = slam_delay;

    /// INERTIAL INITIALIZER OPTIONS ///
    vio_manager_options.gravity_mag = gravity_mag;
    vio_manager_options.init_window_time = init_window_time;
    vio_manager_options.init_imu_thresh = init_imu_thresh;

    /// IMU NOISE OPTIONS ///
    // vio_manager_options.imu_noises.sigma_w = imu_sigma_w;
    // vio_manager_options.imu_noises.sigma_wb = imu_sigma_wb;
    // vio_manager_options.imu_noises.sigma_a = imu_sigma_a;
    // vio_manager_options.imu_noises.sigma_ab = imu_sigma_ab;
    // vio_manager_options.imu_noises.sigma_w_2 = imu_sigma_w_2;
    // vio_manager_options.imu_noises.sigma_wb_2 = imu_sigma_wb_2;
    // vio_manager_options.imu_noises.sigma_a_2 = imu_sigma_a_2;
    // vio_manager_options.imu_noises.sigma_ab_2 = imu_sigma_ab_2;
    // NOTE - JAMES ONLY BUMP COVARIANCE (i.e _2 stats) NOT HZ? -> didn't help much in initial tests. need both dialed wayyyy up

    // TOYED WITH, EXTRA NOISY BUT WORKS BETTER
    // vio_manager_options.imu_noises.sigma_w = 1.6968e-02;
    // vio_manager_options.imu_noises.sigma_wb = 1.9393e-03;
    // vio_manager_options.imu_noises.sigma_a = 2.0000e-1;
    // vio_manager_options.imu_noises.sigma_ab = 3.0000e-01;
    // vio_manager_options.imu_noises.sigma_w_2 = pow(1.6968e-02, 2);
    // vio_manager_options.imu_noises.sigma_wb_2 = pow(1.9393e-03, 2);
    // vio_manager_options.imu_noises.sigma_a_2 = pow(2.0000e-1, 2);
    // vio_manager_options.imu_noises.sigma_ab_2 = pow(3.0000e-01, 2);

    // DEFAULTS
    vio_manager_options.imu_noises.sigma_w = 1.6968e-04 * 20;
    // vio_manager_options.imu_noises.sigma_w_2 = pow(1.6968e-04, 2);
    vio_manager_options.imu_noises.sigma_wb = 1.9393e-05 * 20;
    // vio_manager_options.imu_noises.sigma_wb_2 = pow(1.9393e-05, 2);
    vio_manager_options.imu_noises.sigma_a = 2.0000e-3 * 20;
    // vio_manager_options.imu_noises.sigma_a_2 = pow(2.0000e-3, 2);
    vio_manager_options.imu_noises.sigma_ab = 3.0000e-03 * 20;

    /// FEATURE OPTIONS - all use the same struct, can be dif per feature set ///
    // msckf
    vio_manager_options.msckf_options.chi2_multipler = msckf_chi2_multiplier;
    vio_manager_options.msckf_options.sigma_pix = msckf_sigma_px;
    vio_manager_options.msckf_options.sigma_pix_sq = msckf_sigma_px_sq;
    // slam
    vio_manager_options.slam_options.chi2_multipler = slam_chi2_multiplier;
    vio_manager_options.slam_options.sigma_pix = slam_sigma_px;
    vio_manager_options.slam_options.sigma_pix_sq = slam_sigma_px_sq;
    // zupt
    vio_manager_options.zupt_options.chi2_multipler = zupt_chi2_multiplier;  // set to 0 for only display based zupt
    vio_manager_options.zupt_options.sigma_pix = zupt_sigma_px;
    vio_manager_options.zupt_options.sigma_pix_sq = zupt_sigma_px_sq;

    // TOYED WITH -> cranked way up for everything, lets way more features "through"
    // // msckf
    // vio_manager_options.msckf_options.chi2_multipler = 0.10;
    // vio_manager_options.msckf_options.sigma_pix = 5;
    // vio_manager_options.msckf_options.sigma_pix_sq = 25;
    // // slam
    // vio_manager_options.slam_options.chi2_multipler = 0.10;
    // vio_manager_options.slam_options.sigma_pix = 5;
    // vio_manager_options.slam_options.sigma_pix_sq = 25;

    /// ZUPT OPTIONS ///
    vio_manager_options.try_zupt = try_zupt;
    vio_manager_options.zupt_max_velocity = zupt_max_velocity;
    vio_manager_options.zupt_only_at_beginning = zupt_only_at_beginning;
    vio_manager_options.zupt_noise_multiplier = zupt_noise_multiplier;
    vio_manager_options.zupt_max_disparity = zupt_max_disparity;  // set to 0 for only imu based zupt

    /// GENERAL OPTIONS ///
    vio_manager_options.use_stereo = use_stereo;
    vio_manager_options.use_mask = use_mask;
    vio_manager_options.use_aruco = false;

    /// TRACKER + EXTRACTOR OPTIONS ///
    vio_manager_options.use_klt = use_klt;
    vio_manager_options.num_pts = num_pts;
    vio_manager_options.fast_threshold = fast_threshold;
    vio_manager_options.grid_x = grid_x;
    vio_manager_options.grid_y = grid_y;
    vio_manager_options.min_px_dist = min_px_dist;
    vio_manager_options.knn_ratio = knn_ratio;
    vio_manager_options.downsample_cameras = downsample_cams;
    vio_manager_options.use_multi_threading = use_multithreading;

    /// CAMERA INTRINSICS + EXTRINSICS ///
    int actual_index = 0;
    for (size_t i = 0; i < MAX_CAMERAS; i++) {
        // Set the camera type
        if (cam_info_vec[i].enable) {
            if (cam_info_vec[i].is_fisheye)
                vio_manager_options.camera_fisheye[actual_index] = true;
            else
                vio_manager_options.camera_fisheye[actual_index] = false;
            // Set the dims, if stereo then this would be the size of 1 image not both
            vio_manager_options.camera_wh[actual_index] = std::make_pair(cam_info_vec[i].cam_calib_intrinsic(8, 0), cam_info_vec[i].cam_calib_intrinsic(9, 0));
            // The camera intrinsics
            Eigen::Matrix<double, 10, 1> cam_calib_intrinsic = cam_info_vec[i].cam_calib_intrinsic;
            vio_manager_options.camera_intrinsics[actual_index] = cam_calib_intrinsic;
            vio_manager_options.camera_extrinsics[actual_index] = cam_info_vec[i].cam_wrt_imu;
            actual_index++;
        }
    }
    vio_manager_options.state_options.num_cameras = actual_index;
    return vio_manager_options;
}

#define CONTROL_COMMANDS (RESET_VIO_HARD)
// control listens for reset commands
static void _control_pipe_cb(__attribute__((unused)) int ch, char* string, int bytes, __attribute__((unused)) void* context) {
    // remove the trailing newline from echo
    if (bytes > 1 && string[bytes - 1] == '\n') {
        string[bytes - 1] = 0;
    }

    if (strncmp(string, RESET_VIO_HARD, strlen(RESET_VIO_HARD)) == 0) {
        printf("Client requested hard reset\n");
        std::lock_guard<std::mutex> lg(cam_mutex);
        std::lock_guard<std::mutex> lg2(vio_manager_mutex);
        ov_msckf::VioManagerOptions vio_manager_options = generate_open_vins_manager_options();
        // HARD RESET
        vio_manager.reset(new ov_msckf::VioManager(vio_manager_options));
        return;
    }

    printf("WARNING: Server received unknown command through the control pipe!\n");
    printf("got %d bytes. Command is: %s\n", bytes, string);
    return;
}

static void _cam_disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context) {
    fprintf(stderr, "WARNING: disconnected from camera server, resetting VIO\n");
    global_error_codes |= ERROR_CODE_CAM_MISSING;
    last_cam_timestamp_ns = 0;
    is_cam_connected = 0;
    std::lock_guard<std::mutex> lg(cam_mutex);
    std::lock_guard<std::mutex> lg2(vio_manager_mutex);
    ov_msckf::VioManagerOptions vio_manager_options = generate_open_vins_manager_options();
    // HARD RESET
    vio_manager.reset(new ov_msckf::VioManager(vio_manager_options));
    return;
}

static void _imu_disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context) {
    fprintf(stderr, "WARNING: disconnected from imu server, resetting VIO\n");
    global_error_codes |= ERROR_CODE_IMU_MISSING;
    last_imu_timestamp_ns = 0;
    is_imu_connected = 0;
    std::lock_guard<std::mutex> lg(cam_mutex);
    std::lock_guard<std::mutex> lg2(vio_manager_mutex);
    ov_msckf::VioManagerOptions vio_manager_options = generate_open_vins_manager_options();
    // HARD RESET
    vio_manager.reset(new ov_msckf::VioManager(vio_manager_options));
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

    // add in optional fields to the info JSON file
    // cJSON* json = pipe_server_get_info_json_ptr(SIMPLE_OUTPUT_CH);
    // cJSON_AddStringToObject(json, "imu", imu_name);
    // cJSON_AddStringToObject(json, "cam", cam_pipe_location);
    pipe_server_update_info(SIMPLE_OUTPUT_CH);
    pipe_server_set_control_cb(SIMPLE_OUTPUT_CH, _control_pipe_cb, NULL);
    pipe_server_set_available_control_commands(SIMPLE_OUTPUT_CH, CONTROL_COMMANDS);

    pipe_info_t info_ =
        {
            "open-vins-cam",                   // name
            "/run/mpa/open-vins-cam",          // location
            "camera_image_metadata_t",         // type
            PROCESS_NAME,                      // server_name
            1024*1024*64,  // size_bytes
            0                                  // server_pid
        };

    if (pipe_server_create(SIMPLE_OUTPUT_CH + 1, info_, flags)) {
        return -1;
    }

    return 0;
}

static int connect_client_pipes(void) {
    // connect to imu
    char full_pipe[CHAR_BUF_SIZE];
    if (pipe_expand_location_string(imu_name, full_pipe) < 0) {
        fprintf(stderr, "ERROR: unable to expand location string with imu %s\n", imu_name);
        return -1;
    }

    pipe_client_set_disconnect_cb(IMU_CH, _imu_disconnect_cb, NULL);
    pipe_client_set_simple_helper_cb(IMU_CH, _new_imu_data_handler, NULL);
    int flags = CLIENT_FLAG_EN_SIMPLE_HELPER;
    if (pipe_client_open(IMU_CH, full_pipe, PROCESS_NAME, flags, IMU_RECOMMENDED_READ_BUF_SIZE) != 0) {
        return -1;
    }

    int actual_index = 0;

    // Connect to all the camera pipes
    for (size_t i = 0; i < MAX_CAMERAS; i++) {
        if (cam_info_vec[i].enable && cam_info_vec[i].name[0] != '\0') {
            memset(full_pipe, '\0', CHAR_BUF_SIZE);
            int channel_number = CAMERA_CH_START_OFFSET + actual_index;
            // fprintf(stderr, "channel num: %d\n", channel_number);
            if (pipe_expand_location_string(cam_info_vec[i].name, full_pipe) < 0) {
                fprintf(stderr, "ERROR: unable to expand location string with camera %s\n", cam_info_vec[i].name);
                return -1;
            }

            pipe_client_set_disconnect_cb(channel_number, _cam_disconnect_cb, NULL);
            pipe_client_set_camera_helper_cb(channel_number, _new_camera_data_handler, NULL);
            int flags = CLIENT_FLAG_EN_CAMERA_HELPER;
            if (pipe_client_open(channel_number, full_pipe, PROCESS_NAME, flags, CAM_READ_BUF_SIZE) != 0) {
                fprintf(stderr, "ERROR: FAILED TO OPEN %s\n", full_pipe);
                return -1;
            }

            // if stereo, the right camera is going to use id+1 for its images, so we need to make space for that
            if (cam_info_vec[i].is_stereo){
                actual_index += 1;
                // also need to skip the NEXT camera in the vector, since it should be the same topic, just a pair
                i+=1;
            }
            actual_index+=1; // regular bump
        }
    }
    return 0;
}

int main(int argc, char* argv[]) {
    // Parse the command line options and terminate if the parser says we should terminate
    if (_parse_opts(argc, argv)) {
        return -1;
    }

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

    rc_ov_ringbuf_alloc(&imu_buf, 1000);

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

    // Create the server pipes
    if (create_server_pipes() < 0) _quit(0);

    // Connect to the client pipes and start getting data
    if (connect_client_pipes() < 0) _quit(0);

    // Start the read and publish thread
    publish_vio_data_thread = std::thread(_publish_vio_data);

    // fprintf(stderr, "started vio thread\n");

    // run until start/stop module catches a signal and changes main_running to 0
    while (main_running) usleep(5000000);

    // Shutdown Nicely
    _quit(0);

    return 0;
}
