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
#include <state/Propagator.h>
#include <state/State.h>
#include <utils/quat_ops.h>

#include <iostream>
#include <thread>

#include "cCharacter.h"

#include "config_file.h"
#include "quality.h"

#define OV_VIO_CONTROL_COMMANDS (RESET_VIO_SOFT "," RESET_VIO_HARD)

// These are the paths of the named pipe interfaces
// MODAL_PIPE_DEFAULT_BASE_DIR is defined in modal_pipe_common.h
// the 'complete' interface sends data as a qvio_data_t struct
// the 'simple' interface uses the simple vio_data_t struct from modal_vio_server_interface.h

#define OV_VIO_EXTENDED_NAME "ov_extended"
#define OV_VIO_EXTENDED_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR OV_VIO_EXTENDED_NAME "/"
#define OV_VIO_SIMPLE_NAME "ov"
#define OV_VIO_SIMPLE_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR OV_VIO_SIMPLE_NAME "/"

#define OV_VIO_OVERLAY_NAME "ov_overlay"
#define OV_VIO_OVERLAY_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR OV_VIO_OVERLAY_NAME "/"

// server channels
#define EXTENDED_CH 0
#define SIMPLE_CH 1
#define OVERLAY_CH 2

// client channels and config
#define IMU_CH 0
#define FEATURE_CH 1
#define CAMERA_CH_START_OFFSET 1
#define IMU_PIPE_MIN_PIPE_SIZE (1 * 1024 * 1024)  // give ourselves huge buffers
#define CAM_PIPE_SIZE (256 * 1024 * 1024)         // give ourselves huge buffers
#define PROCESS_NAME "open-vins-server"
#define FEATURE_NAME "tracked_feats"
#define FEATURE_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR FEATURE_NAME "/"

// after 300ms with no response, the health monitor thread assumes mvVISLAM
// has locked up while processing a frame and starts sending messages indicating
// a stall has occured with a failed state
#define STALL_TIMEOUT_NS 300000000

// auto restart if the system fails to init after 10 seconds
#define INIT_FAILURE_TIMEOUT_NS 10000000000

// do not check for blowups until 1 second after VIO claims to have initialized
#define BLOWUP_DETECT_TIMEOUT_NS 1000000000

// not really sure if this will be needed
#define SILENT_STD(x)                             \
    {                                             \
        FILE* silentfd = fopen("/dev/null", "w"); \
        int savedstdoutfd = dup(STDOUT_FILENO);   \
        fflush(stdout);                           \
        dup2(fileno(silentfd), STDOUT_FILENO);    \
        x                                         \
            fflush(stdout);                       \
        fclose(silentfd);                         \
        dup2(savedstdoutfd, STDOUT_FILENO);       \
        close(savedstdoutfd);                     \
    }

static int en_config_only = 0;
static int en_debug = 0;
static int en_debug_pos = 0;
static int en_debug_timing_cam = 0;
static int en_debug_timing_imu = 0;
static pthread_t health_thread;

// these are the last timestamps that have completely passed into mvvislam
// cam time is middle of frame. Also last pose to have been received from mvvislam
static volatile int64_t last_imu_timestamp_ns = 0;
// this will need to be populated per camera before we start referencing it
static volatile int64_t last_real_pose_timestamp_ns = 0;
static volatile int64_t last_sent_timestamp_ns = 0;

// state of imu and camera connections
static volatile int is_imu_connected = 0;
static volatile int is_cam_connected = 0;
static volatile int has_received_all_calib = 0;
static volatile int has_received_all_params = 0;

// flag set to 1 on reset to indicate to the blowup detector not to check
// for blowups until after VIO actually initializes
static volatile int hard_reset_blowup_flag = 1;

// flag and time when a reset is requested to indicate to the init failure
// detection how long VIO has been trying to init
static volatile int init_failure_detector_reset_flag = 0;
static volatile int64_t time_of_last_reset = 0;
static volatile int last_state = VIO_STATE_FAILED;
static volatile int blowup_detector_flag = 0;
static volatile int64_t time_of_first_okay = 0;

// openvins functions do not seem to be thread safe, protect all calls to the VioManager
std::unique_ptr<ov_msckf::VioManager> vio_manager;
static ov_msckf::VioManagerOptions vio_manager_options;
std::mutex vio_manager_mutex;
std::mutex imu_lock_mutex;
static int is_initialized = 0;
static int blank_counter = 0;
static int fade_counter = 0;
static int64_t last_time_alignment_ns = 0;
static int32_t last_frame_frame_id = 0;
static int64_t last_frame_timestamp_ns = 0;

// overlay image stream stuff
#define DRAW_BONUS_ROWS_TOP 32
#define DRAW_BONUS_ROWS_BOT 32
static uint8_t* draw_frame = NULL;
static camera_image_metadata_t draw_meta;

// set any error codes here for publishing in the data structure
static uint32_t global_error_codes = 0;

typedef struct camera_info {
    Eigen::Matrix<double, 7, 1> cam_wrt_imu;
    Eigen::Matrix<double, 10, 1> cam_calib_intrinsic;
    bool is_fisheye;
    size_t cam_id;
} camera_info;

static std::vector<camera_info> cam_info_vec;
static char imu_name[CHAR_BUF_SIZE] = "imu"; // todo read in with calib data

static size_t num_cams = 0;


#define VFT_CMD_START   "start"
#define VFT_CMD_RESTART "restart"
#define VFT_CMD_PAUSE   "pause"

// function prototypes
static void _publish(camera_image_metadata_t meta, std::vector<cv::Mat> images);
// static void _publish();
static bool show_extra_points_on_overlay = true;

static int _hard_reset(bool is_locked);

static int8_t verbosity_level{static_cast<uint8_t>(ov_core::Printer::PrintLevel::SILENT)};
std::string log_path = "";

// printed if some invalid argument was given
static void _print_usage(void) {
    printf(
        "\n\
This is meant to run in the background as a systemd service, but can be\n\
run manually with the following debug options\n\
\n\
-c, --config                only parse the config file and exit, don't run\n\
-d, --debug                 enable debug prints\n\
-h, --help                  print this help message\n\
-i, --timing-imu            show timing prints for imu processing\n\
-l, --log_path              enable using calibration files from a log instead of defaults\n\
                              this is only necessary if cal files have changed or if the\n\
                              log is from a different setup.\n\
                              log path shoulf be absolute to the start of the dir\n\
                              i.e. /data/voxl-logger/log0001 (with or w/out last /)\n\
-p, --position              print position and rotation\n\
-s, --debug-crash           print lots of numbers to track down location of crashes\n\
-t, --timing-cam            enable timing prints for camera processing\n\
-v, --verbosity             sets the verbosity level for OV lib prints, will default to silent\n\
                              0 - ALL\n\
                              1 - DEBUG\n\
                              2 - INFO\n\
                              3 - WARNING\n\
                              4 - ERROR\n\
                              5 - SILENT\n\
\n");
    return;
}

static bool _parse_opts(int argc, char* argv[]) {
    static struct option long_options[] =
        {
            {"config", no_argument, 0, 'c'},
            {"debug", no_argument, 0, 'd'},
            {"help", no_argument, 0, 'h'},
            {"timing-imu", no_argument, 0, 'i'},
            {"log_path", required_argument, 0, 'l'},
            {"position", no_argument, 0, 'p'},
            {"timing-cam", no_argument, 0, 't'},
            {"verbosity", required_argument, 0, 'v'},
            {0, 0, 0, 0}};

    // set default before we do anything else
    ov_core::Printer::setPrintLevel(ov_core::Printer::PrintLevel::SILENT);

    while (1) {
        int option_index = 0;
        int c = getopt_long(argc, argv, "cdhil:ptv:", long_options, &option_index);

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
                en_config_only = 1;
                break;

            case 'd':
                printf("Enabling debug mode\n");
                en_debug = true;
                break;

            case 'h':
                _print_usage();
                return true;

            case 'l':
                log_path.assign(optarg);
                if (log_path.back() == '/') log_path.pop_back();
                printf("Using log path of: %s\n", log_path.data());
                break;

            case 'i':
                printf("Enabling debug imu timing mode\n");
                en_debug_timing_imu = 1;
                break;

            case 'p':
                en_debug_pos = 1;
                break;

            case 't':
                en_debug_timing_cam = 1;
                break;

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
                        fprintf(stderr, "Unknown debug level\n");
                        _print_usage();
                        return false;
                }
                break;

            default:
                _print_usage();
                return true;
        }
    }
    return false;
}

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

// call this instead of return when it's time to exit to cleans up everything
static void _quit(int ret) {
    // Close all the open pipe connections
    pipe_server_close_all();
    pipe_client_close_all();

    // Remove this process ID file
    remove_pid_file(PROCESS_NAME);

    if (ret == 0)
        printf("Exiting Cleanly\n");
    else
        printf("error code\n");
    exit(ret);
    return;
}

// pose data is published from the same thread that does the camera processing
// and pose estimation. That freezes, sometimes for over a second, during blowups
// so this thread exists to keep data coming out during that situation, warning
// consumers that there is an issue.
// this does NOT monitor for blowup criteria, that's done in the camera thread
// as soon as a new pose is calculated. This thread is to warn when that
// camera thread freezes.
static void* _health_thread_func(__attribute__((unused)) void* ctx) {
    while (main_running) {
        usleep(30000);  // run about the same speed as the camera

        int64_t current_time = _apps_time_monotonic_ns();
        int64_t delay_ns = current_time - last_real_pose_timestamp_ns;

        if (init_failure_detector_reset_flag) {
            if (time_of_last_reset != 0) {
                uint64_t time_since_reset = current_time - time_of_last_reset;
                if (time_since_reset > INIT_FAILURE_TIMEOUT_NS) {
                    fprintf(stderr, "WARNING failed to init in time, trying again\n");
                    _hard_reset(false);
                    continue;
                }
            } else
                continue;
        }

        // If last packet is recent enough, nothing to worry about.
        // Otherwise, send out failure packets, this inlcudes global error codes
        // indicating if we are waiting for cam or IMU data
        if (delay_ns < STALL_TIMEOUT_NS && last_real_pose_timestamp_ns != 0) continue;

        // flag that we've sent a packet with the current timestamp
        last_sent_timestamp_ns = current_time;

        ext_vio_data_t d;  // complete "extended" vio MPA packet
        vio_data_t s;      // simplified vio packet

        // make sure we start with clean data structs and apply any global error codes
        // full extended qvio packet
        memset(&d, 0, sizeof(d));
        d.v.magic_number = VIO_MAGIC_NUMBER;
        d.v.timestamp_ns = current_time;
        d.v.error_code = global_error_codes | ERROR_CODE_STALLED;
        d.v.state = VIO_STATE_FAILED;
        d.v.quality = -1.0f;
        // simple lib modal pipe standard vio packet
        memset(&s, 0, sizeof(s));
        s.magic_number = VIO_MAGIC_NUMBER;
        s.timestamp_ns = current_time;
        s.error_code = global_error_codes | ERROR_CODE_STALLED;
        s.state = VIO_STATE_FAILED;
        s.quality = -1.0f;

        // send to both pipes
        pipe_server_write(EXTENDED_CH, (char*)&d, sizeof(ext_vio_data_t));
        pipe_server_write(SIMPLE_CH, (char*)&s, sizeof(vio_data_t));

        // turn off dropped cam frame code now we have informed everyone.
        global_error_codes &= ~ERROR_CODE_DROPPED_CAM;
    }

    return NULL;
}

static int _hard_reset(bool is_locked) {
    // lock the mutex before calling any ov api calls
    if (!is_locked) vio_manager_mutex.lock();
    imu_lock_mutex.lock();

    // stop it if it's running
    if (is_initialized) {
        is_initialized = 0;
        vio_manager.reset();
    }

    // let the blowup detection know we just had a reset
    hard_reset_blowup_flag = 1;

    // let the init failure detector know we just had a reset
    init_failure_detector_reset_flag = 1;
    blowup_detector_flag = 0;
    last_state = VIO_STATE_FAILED;
    time_of_last_reset = _apps_time_monotonic_ns();

    // now start again
    vio_manager.reset(new ov_msckf::VioManager(vio_manager_options));

    if (vio_manager == NULL) {
        fprintf(stderr, "Error creating vio_manager object\n");
        _quit(-1);
    }

    if (!is_locked) vio_manager_mutex.unlock();
    imu_lock_mutex.unlock();

    return 0;
}

// control listens for reset commands
static void _control_pipe_cb(__attribute__((unused)) int ch, char* string,
                             int bytes, __attribute__((unused)) void* context) {
    // remove the trailing newline from echo
    if (bytes > 1 && string[bytes - 1] == '\n') {
        string[bytes - 1] = 0;
    }

    // soft reset is not currently allowed
    // if(strncmp(string, RESET_VIO_SOFT, strlen(RESET_VIO_SOFT))==0){
    // 	printf("Client requested soft reset\n");
    // 	pthread_mutex_lock(&mv_mtx);
    // 	mvVISLAM_Reset(mv_vislam_ptr, 0);
    // 	pthread_mutex_unlock(&mv_mtx);
    // 	return;
    // }

    if (strncmp(string, RESET_VIO_HARD, strlen(RESET_VIO_HARD)) == 0) {
        printf("Client requested hard reset\n");
        _hard_reset(false);  // close and restart the object
        return;
    }

    printf("WARNING: Server received unknown command through the control pipe!\n");
    printf("got %d bytes. Command is: %s\n", bytes, string);
    return;
}

// print when a new client connects to us
static void _overlay_connect_cb(__attribute__((unused)) int ch,
                                __attribute__((unused)) int client_id,
                                char* client_name,
                                __attribute__((unused)) void* context) {
    printf("client \"%s\" connected to overlay\n", client_name);
    return;
}

// print when a client disconnects from us
static void _overlay_disconnect_cb(__attribute__((unused)) int ch,
                                   __attribute__((unused)) int client_id,
                                   char* client_name,
                                   __attribute__((unused)) void* context) {
    printf("client \"%s\" disconnected from overlay\n", client_name);
    return;
}


#define VOXL_FT_MAGIC_NUMBER (0x54555249)
#define VOXL_CALIB_MAGIC_NUMBER (0x43414C49)
#define VOXL_PARAM_MAGIC_NUMBER (0x50415241)


typedef struct Feature {
    size_t id;
    size_t cam_id;
    float x;
    float y;
} __attribute((packed))__Feature;

// this is our meta
// follow it with a vector of feats
typedef struct FeatureOutput{
    uint32_t magic_number;
    int64_t timestamp_ns; // will match image timestamp
    uint32_t num_feats;
} __attribute((packed))__FeatureOutput;

typedef struct CalibrationPacket {
    uint32_t magic_number;
    int64_t timestamp_ns; // just in case
    size_t cam_id;
    size_t num_cams;
    Eigen::Matrix<double, 7, 1> cam_wrt_imu;
    Eigen::Matrix<double, 10, 1> cam_calib_intrinsic;
    bool is_fisheye;
} CalibrationPacket;

typedef struct ParamPacket {
    uint32_t magic_number;
    char imu_name[CHAR_BUF_SIZE];
    int num_features_to_track;
} ParamPacket;

static void _new_feat_data_handler(__attribute__((unused)) int ch, char* data, int bytes, __attribute__((unused)) void* context) {
    static std::vector<cv::Mat> cached_images;
    static camera_image_metadata_t cached_meta;
    // first check is for the magic number of our packet
    uint32_t magic_num = *(uint32_t*)data;
    
    switch (magic_num)
    {
    case VOXL_FT_MAGIC_NUMBER:{
        // now cast it to the proper type
        FeatureOutput* feat_out = (FeatureOutput*)data;
        ov_core::ProcessedCameraData vio_manager_data;

        vio_manager_data.timestamp = feat_out->timestamp_ns / 1000000000.0;

        std::vector<ov_core::MaiFeature> feat_vec(feat_out->num_feats);
        void* d_start = data + sizeof(FeatureOutput);
        feat_vec.assign((ov_core::MaiFeature*)d_start, (ov_core::MaiFeature*)d_start+feat_out->num_feats);
        vio_manager_data.feats = feat_vec;

        for (size_t i = 0; i < feat_out->num_feats; i++){
            if(std::find(vio_manager_data.sensor_ids.begin(), vio_manager_data.sensor_ids.end(), feat_vec[i].cam_id) != vio_manager_data.sensor_ids.end()) 
                continue;
            else 
                vio_manager_data.sensor_ids.push_back(feat_vec[i].cam_id);
        }

        std::lock_guard<std::mutex> lg(vio_manager_mutex);
        if (vio_manager == nullptr) return;

        int64_t time_before = _apps_time_monotonic_ns();
        vio_manager->feed_measurement_processed_camera(vio_manager_data);    
        int64_t process_time = _apps_time_monotonic_ns() - time_before;

        if (process_time > 500000000) {
            // if image processing took more than half a second, something went wrong
            // and we should drop frames
            global_error_codes |= ERROR_CODE_DROPPED_CAM;
            pipe_client_flush(ch);
            fprintf(stderr, "ERROR: slow image proc time: %6.2fms\n", ((double)process_time) / 1000000.0);
            fprintf(stderr, "Flushing camera frames\n");
        } else if (process_time > 50000000) {
            // warn if image processing was a little slow, this happens from time
            // to time and isn't a big deal
            fprintf(stderr, "WARNING: slow image proc time: %6.2fms\n", ((double)process_time) / 1000000.0);
        }

        // after the first feed i think, ov needs other data first
        is_cam_connected = true;

        _publish(cached_meta, cached_images);

        break;
    }
    case VOXL_CALIB_MAGIC_NUMBER:{
        printf("Received Calibration Packet\n");
        // now we need to fill our cam_info_vec with this info
        CalibrationPacket* cp = (CalibrationPacket*)data;
        
        size_t cam_index = 0;
        size_t cam_id = cp->cam_id;

        num_cams = cp->num_cams;

        camera_info curr_info;
        curr_info.is_fisheye = cp->is_fisheye;
        curr_info.cam_wrt_imu = cp->cam_wrt_imu;
        curr_info.cam_calib_intrinsic = cp->cam_calib_intrinsic;
        curr_info.cam_id = cp->cam_id;

        cam_info_vec.push_back(curr_info);

        if (cam_info_vec.size() == num_cams){
            has_received_all_calib = 1;
        }
    }
    case CAMERA_MAGIC_NUMBER: {
        std::lock_guard<std::mutex> lg(vio_manager_mutex);

        cached_images.clear();
        camera_image_metadata_t* meta = (camera_image_metadata_t*)data;
        cached_meta = *meta;

        void* d_start = data + sizeof(camera_image_metadata_t);

        // majority of the time we should receive this type
        if (meta->format == IMAGE_FORMAT_RAW8) {
            cv::Mat img(meta->height, meta->width, CV_8UC1, d_start);
            cached_images.push_back(img);
        }
        // this is weird and should not happen often, as we are tracking only in the grey plane
        else if (meta->format == IMAGE_FORMAT_STEREO_NV12 || meta->format == IMAGE_FORMAT_STEREO_NV21) {
            cv::Mat img(meta->height, meta->width, CV_8UC1, d_start);
            cv::Mat img2(meta->height, meta->width, CV_8UC1, d_start + (meta->height * meta->width)*(3/2)); // yuv (3/2)
            cached_images.push_back(img);
            cached_images.push_back(img2);
        }
        else if (meta->format == IMAGE_FORMAT_STEREO_RAW8){
            cv::Mat img(meta->height, meta->width, CV_8UC1, d_start);
            cv::Mat img2(meta->height, meta->width, CV_8UC1, d_start + (meta->height * meta->width));
            cached_images.push_back(img);
            cached_images.push_back(img2);
        }
    }
        break;

    case VOXL_PARAM_MAGIC_NUMBER: {
        printf("Received Param Packet\n");

        ParamPacket* pp = (ParamPacket*)data;
        strncpy(imu_name, pp->imu_name, CHAR_BUF_SIZE);
        num_pts = pp->num_features_to_track;
        
        // stop blocking
        if (has_received_all_calib) has_received_all_params = 1;
    }
        break;

    default:
        break;
    }
}


// imu callback registered to the imu server
static void _new_imu_data_handler(__attribute__((unused)) int ch, char* data, int bytes, __attribute__((unused)) void* context) {
    int n_packets;
    imu_data_t* data_array = pipe_validate_imu_data_t(data, bytes, &n_packets);

    if (data_array == NULL) return;
    if (n_packets <= 0) return;

    // flag that imu data is active, skip data if camera is disconnected
    is_imu_connected = 1;
    global_error_codes &= ~ERROR_CODE_IMU_MISSING;
    if (!is_cam_connected) return;

    std::lock_guard<std::mutex> lg(imu_lock_mutex);

    // time this in debug mode
    int64_t time_before, process_time;
    if (en_debug_timing_imu) time_before = _apps_time_monotonic_ns();

    for (int i = 0; i < n_packets; i++) {
        // check if we somehow got an out-of-order imu sample and reject it
        if ((int64_t)data_array[i].timestamp_ns <= last_imu_timestamp_ns) {
            double dt = (last_imu_timestamp_ns - data_array[i].timestamp_ns) / 1000000.0;
            fprintf(stderr, "WARNING out-of-order imu %fms before previous\n", dt);
            continue;
        }
        // Create the data struct that we will use for ingesting data into the vio manager
        ov_core::ImuData vio_manager_data;
        vio_manager_data.timestamp = data_array[i].timestamp_ns / 1000000000.0;  // (seconds)

        vio_manager_data.wm(0, 0) = data_array[i].gyro_rad[0];
        vio_manager_data.wm(1, 0) = -data_array[i].gyro_rad[1];
        vio_manager_data.wm(2, 0) = -data_array[i].gyro_rad[2];

        vio_manager_data.am(0, 0) = data_array[i].accl_ms2[0];
        vio_manager_data.am(1, 0) = -data_array[i].accl_ms2[1];
        vio_manager_data.am(2, 0) = -data_array[i].accl_ms2[2];


        vio_manager->feed_measurement_imu(vio_manager_data);

        last_imu_timestamp_ns = data_array[i].timestamp_ns;
    }

    if (en_debug_timing_imu) {
        process_time = _apps_time_monotonic_ns() - time_before;
        printf("IMU proc time %6.2fms for %d samples\n", ((double)process_time) / 1000000.0, n_packets);
    }
    return;
}


#ifdef BUILD_QRB5165

// for qrb5165 only (right now) set the camera processing thread to run on
// CPU 7, big boy
static void _check_and_set_affinity(void) {
    // only do this once
    static int has_set = 0;
    if (has_set) return;

    cpu_set_t cpuset;
    pthread_t thread;
    thread = pthread_self();

    /* Set affinity mask to include CPUs 7 only */
    CPU_ZERO(&cpuset);
    CPU_SET(7, &cpuset);

    if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset)) {
        perror("pthread_setaffinity_np");
    }

    /* Check the actual affinity mask assigned to the thread */
    if (pthread_getaffinity_np(thread, sizeof(cpu_set_t), &cpuset)) {
        perror("pthread_getaffinity_np");
    }
    printf("Camera processing thread is now locked to the following cores:");
    for (int j = 0; j < CPU_SETSIZE; j++) {
        if (CPU_ISSET(j, &cpuset)) printf(" %d", j);
    }
    printf("\n");

    // only do this once on start
    has_set = 1;

    return;
}
#endif

// return 0 if all is well, otherwise return the reason for blowup
static int _check_for_blowup(std::shared_ptr<ov_msckf::State> current_state, Eigen::Matrix<double, 12, 12> cov_plus, int good_features) {
    int64_t current_ts = current_state->_timestamp * 1e9;
    static int64_t last_time_with_good_cov = 0;
    static int64_t last_time_with_enough_features = 0;

    // reset timers to current time after reset so we don't trip this during init
    if (hard_reset_blowup_flag) {
        last_time_with_enough_features = current_ts;
        last_time_with_good_cov = current_ts;
        hard_reset_blowup_flag = 0;
    }

    // Now go through our 4 blowup criteria
    // max velocity check  current_state->_imu->vel().cast<float>()
    float vel = sqrtf((current_state->_imu->vel_fej()(0) * current_state->_imu->vel_fej()(0)) +
                      (current_state->_imu->vel_fej()(1) * current_state->_imu->vel_fej()(1)) +
                      (current_state->_imu->vel_fej()(2) * current_state->_imu->vel_fej()(2)));
    if (vel > auto_reset_max_velocity) {
        fprintf(stderr, "WARNING auto-resetting due to exceeding max velocity of %4.1fm/s\n", (double)auto_reset_max_velocity);
        return ERROR_CODE_VEL_INST_CERT;
    }

    // get max velocity covariance
    double cov = cov_plus(6, 6);
    if (cov_plus(7, 7) > cov) cov = cov_plus(7, 7);
    if (cov_plus(8, 8) > cov) cov = cov_plus(8, 8);

    // min feature timeout check
    if (good_features > auto_reset_min_features) {
        last_time_with_enough_features = current_ts;
    } else {
        float tmp = (float)(current_ts - last_time_with_enough_features) / 1000000000.0f;
        if (tmp > auto_reset_min_feature_timeout_s) {
            fprintf(stderr, "WARNING auto-resetting due to low feature count\n");
            fprintf(stderr, "feats: %d, limit: %d\n", good_features, auto_reset_min_features);
            return ERROR_CODE_LOW_FEATURES;
        }
    }

    // max v cov timeout check
    if (cov < auto_reset_max_v_cov) {
        last_time_with_good_cov = current_ts;
    } else {
        float tmp = (current_ts - last_time_with_good_cov) / 1000000000.0f;
        if (tmp > auto_reset_max_v_cov_timeout_s) {
            fprintf(stderr, "WARNING auto-resetting due to high vel covariance\n");
            fprintf(stderr, "vel cov: %6.5f, limit: %6.5f\n", cov, auto_reset_max_v_cov);
            return ERROR_CODE_VEL_WINDOW_CERT;
        }
    }

    // check for instant vel covariance limit
    if (cov > auto_reset_max_v_cov_instant) {
        fprintf(stderr, "WARNING auto-resetting due to vel covariance instant limit\n");
        fprintf(stderr, "vel cov: %6.5f, limit: %6.5f\n", cov, auto_reset_max_v_cov_instant);
        return ERROR_CODE_VEL_INST_CERT;
    }

    // all is good (for now)
    return 0;
}

static void _publish(camera_image_metadata_t meta, std::vector<cv::Mat> images) {
    std::shared_ptr<ov_msckf::State> current_state = {nullptr};                      // contains a few extra pieces we need
    Eigen::Matrix<double, 13, 1> state_plus = Eigen::Matrix<double, 13, 1>::Zero();  // not necessary
    Eigen::Matrix<double, 12, 12> cov_plus = Eigen::Matrix<double, 12, 12>::Zero();  // covariance!!!!

    ext_vio_data_t d;  // complete "extended" vio MPA packet
    vio_data_t s;      // simplified vio packet
    int nPoints;
    int n_good_points = 0;
    int n_oos_points = 0;
    int i, j;

    // make sure we start with clean data structs and apply any global error codes
    // full extended vio packet
    memset(&d, 0, sizeof(d));
    d.v.magic_number = VIO_MAGIC_NUMBER;
    d.v.error_code = global_error_codes;

    // simple lib modal pipe standard vio packet
    memset(&s, 0, sizeof(s));
    s.magic_number = VIO_MAGIC_NUMBER;
    s.error_code = global_error_codes;

    // Grab the state, then fill our state_plus and covariance
    current_state = vio_manager->get_state();
    if (!vio_manager->get_propagator()->fast_state_propagate(current_state, (double)(last_imu_timestamp_ns) / 1e9, state_plus, cov_plus)) {
        return;
    }

    // get features
    // this function will give us back as much info as available for features in various stages of the overall state
    std::vector<output_feature> curr_pixel_locs = vio_manager->get_pixel_loc_features();

    for(size_t d=0;d<curr_pixel_locs.size();d++){
        
		if(curr_pixel_locs[d].point_quality == 2 && curr_pixel_locs[d].pix_loc[0]>0.0f && curr_pixel_locs[d].pix_loc[1]>0.0f){
			n_good_points++;
			//printf("%2d %7.1f %7.1f %7.1f %7.1f\n", n_good_points, (double)pPoints[i].depth, (double)pPoints[i].depthErrorStdDev, pPoints[i].pixLoc[0], pPoints[i].pixLoc[1]);
		}
		if(curr_pixel_locs[d].point_quality == 1){
			n_oos_points++;
		}
	}

    // record that we just got a successful pose and point cloud
    last_real_pose_timestamp_ns = static_cast<int64_t>(current_state->_timestamp * 1e9);

    // check if its initialized or not
    if (!vio_manager->initialized()) {
        s.state = VIO_STATE_INITIALIZING;
        is_initialized = false;
        // send to both pipes
        pipe_server_write(EXTENDED_CH, (char*)&d, sizeof(ext_vio_data_t));
        pipe_server_write(SIMPLE_CH, (char*)&s, sizeof(vio_data_t));
        return;
    } else {
        s.state = VIO_STATE_OK;
        is_initialized = true;
    }

    // sometimes qvio will report covariance as invalid but state is still OKAY
    // this is NOT alright, in this case manually set the state to failed.
    if (cov_plus(3, 3) <= 0.0f || cov_plus(4, 4) <= 0.0f || cov_plus(5, 5) <= 0.0f) {
        fprintf(stderr, "ERROR: got negative covariance\n");
        s.state = VIO_STATE_FAILED;
    }

    if (current_state->error_flag == VIO_STATE_FAILED){
        fprintf(stderr, "WARNING auto-resetting, EKF starved of good features for too long\n");
        _hard_reset(true);
        s.state = VIO_STATE_FAILED;
    }

    // we finished initializing, no longer check for init timeout
    if (s.state == VIO_STATE_OK) {
        init_failure_detector_reset_flag = 0;
    }

    // if we just went from good to failed, treat this like a reset for the
    // init failure detector so it can timeout the same as VIO tries to re-init
    // itself after it's own internal reset
    if (last_state != VIO_STATE_FAILED && s.state == VIO_STATE_FAILED) {
        init_failure_detector_reset_flag = 1;
        time_of_last_reset = static_cast<int64_t>(current_state->_timestamp * 1e9);
    }

    // record time when vio claimed to have initialized and only check for
    // for blowups some time after this
    if (last_state != VIO_STATE_OK && s.state == VIO_STATE_OK) {
        blowup_detector_flag = 1;
        time_of_first_okay = static_cast<int64_t>(current_state->_timestamp * 1e9);
    }
    last_state = s.state;

    // while VIO state is OK, do our own additional blowup checks if enough
    // time has passed since the init
    int64_t time_since_first_okay = _apps_time_monotonic_ns() - time_of_first_okay;
    if (blowup_detector_flag && time_since_first_okay > BLOWUP_DETECT_TIMEOUT_NS) {
        int code = _check_for_blowup(current_state, cov_plus, n_good_points);
        if (code) {
            _hard_reset(true);
            s.state = VIO_STATE_FAILED;
            s.error_code |= code;
        }
    }

    // don't send packets from the past, this can happen when qvio stalls
    // during a reset
    if (static_cast<int64_t>(current_state->_timestamp * 1e9) < last_sent_timestamp_ns) {
        // fprintf(stderr, "WARNING skipping pose data from the past\n");
        return;
    }

    // All checks passed, after this point this function should not return
    // until the end
    last_sent_timestamp_ns = static_cast<int64_t>(current_state->_timestamp * 1e9);

    // populate some other data
    s.timestamp_ns = static_cast<int64_t>(current_state->_timestamp * 1e9);
    d.imu_cam_time_shift_s = current_state->_calib_dt_CAMtoIMU->value()(0);
    last_time_alignment_ns = current_state->_calib_dt_CAMtoIMU->value()(0) * 1e9;
    s.n_feature_points = n_good_points;
    d.last_cam_frame_id = last_frame_frame_id;
    d.last_cam_timestamp_ns = last_frame_timestamp_ns;

    // Eigen::MatrixXf::Map(reinterpret_cast<float*>(s.R_imu_to_vio), 3, 3) = current_state->_imu->Rot_fej().cast<float>();
    Eigen::MatrixXf::Map(s.T_imu_wrt_vio, 3, 1) = current_state->_imu->pos().cast<float>();
    Eigen::MatrixXf::Map(s.vel_imu_wrt_vio, 3, 1) = current_state->_imu->vel().cast<float>();

    Eigen::Matrix3d correction_mat;
    correction_mat << 1,0,0,0,-1,0,0,0,-1;
    Eigen::Matrix3d final_out;
        
    final_out = current_state->_imu->Rot_fej() * correction_mat;
    final_out =  correction_mat * final_out;
    Eigen::MatrixXf::Map(reinterpret_cast<float*>(s.R_imu_to_vio), 3, 3) = final_out.cast<float>();
    

    // camera position here is a bit funky, since open vins outputs imu to cam and we want cam to imu
    Eigen::MatrixXf::Map(reinterpret_cast<float*>(s.R_cam_to_imu), 3, 3) = ov_core::quat_2_Rot(current_state->_calib_IMUtoCAM[0]->quat()).transpose().cast<float>();
    // Eigen::MatrixXf::Map(s.T_cam_wrt_imu, 3, 1) = ((ov_core::quat_2_Rot(current_state->_calib_IMUtoCAM[1]->quat().transpose()) * current_state->_calib_IMUtoCAM[1]->pos()) * -1).cast<float>();
    Eigen::MatrixXf::Map(s.T_cam_wrt_imu, 3, 1) = ((ov_core::quat_2_Rot(current_state->_calib_IMUtoCAM[0]->quat().transpose()) * current_state->_calib_IMUtoCAM[0]->pos()) * -1).cast<float>();

    Eigen::MatrixXf::Map(reinterpret_cast<float*>(d.gyro_bias), 3, 3) = current_state->_imu->bias_g_fej().cast<float>();
    Eigen::MatrixXf::Map(reinterpret_cast<float*>(d.accl_bias), 3, 3) = current_state->_imu->bias_a_fej().cast<float>();

    /////////////////////////////////////////////////////////////////////////////////////////////
    // GRAVITY ALIGNMENT WITH QVIO
    // Z AND Y AXES MUST BE FLIPPED FOR EVERYTHING
    /////////////////////////////////////////////////////////////////////////////////////////////

    s.T_imu_wrt_vio[2] = -s.T_imu_wrt_vio[2];
    s.T_imu_wrt_vio[1] = -s.T_imu_wrt_vio[1];
    s.vel_imu_wrt_vio[2] = -s.vel_imu_wrt_vio[2];
    s.vel_imu_wrt_vio[1] = -s.vel_imu_wrt_vio[1];

    // pose covariance diagonals, 6 entries
    s.pose_covariance[0] = (float)cov_plus(0, 0);
    s.pose_covariance[6] = (float)cov_plus(1, 1);
    s.pose_covariance[11] = (float)cov_plus(2, 2);
    s.pose_covariance[15] = (float)cov_plus(3, 3);
    s.pose_covariance[18] = (float)cov_plus(4, 4);
    s.pose_covariance[20] = (float)cov_plus(5, 5);

    // velocity covariance diagonals, 3 entries
    s.velocity_covariance[0] = cov_plus(6, 6);
    s.velocity_covariance[6] = cov_plus(7, 7);
    s.velocity_covariance[11] = cov_plus(8, 8);

    // open vins does not estimate this, but reports it
    // INVERTING Y AND Z AS ABOVE FOR NOW, SHOULD BE IN KALMAN FILTER
    s.imu_angular_vel[0] = state_plus(10);
    s.imu_angular_vel[1] = -state_plus(11);
    s.imu_angular_vel[2] = -state_plus(12);

    // since open vins does the gravity alignment internally, gravity vec is always 0,0,1 and cov is 0'd out
    static float grav_vec[3] = {0, 0, 1};
    memcpy(s.gravity_vector, grav_vec, sizeof(float) * 3);

    // limit the number of features to what fits in our pipe packet
    // TODO figure out why these viz weirdly
    d.n_total_features = (int)curr_pixel_locs.size();
    if (d.n_total_features > VIO_MAX_REPORTED_FEATURES) {
        d.n_total_features = VIO_MAX_REPORTED_FEATURES;
    }

    // NOTE: TODO FIX THIS
    // since we flip the entire world, these reps need the same transformation
    memcpy(d.features, curr_pixel_locs.data(), d.n_total_features*sizeof(vio_feature_t));

    // todo add support for multicam here
	s.quality = calc_quality(s.state, s.velocity_covariance, images[0].rows,\
					images[0].cols, d.n_total_features, d.features);

    // fill in simplified struct inside the extended packet
    memcpy(&d.v, &s, sizeof(vio_data_t));

    // send to both pipes
    pipe_server_write(EXTENDED_CH, (char*)&d, sizeof(ext_vio_data_t));
    pipe_server_write(SIMPLE_CH, (char*)&s, sizeof(vio_data_t));

    // for debug only
    if (en_debug) {
        printf("state: ");
        pipe_print_vio_state(s.state);
        printf(" err: ");
        pipe_print_vio_error(s.error_code);
        printf("\n");
    }
    if (en_debug_pos) {
        printf("%6.3f %6.3f %6.3f ", (double)s.T_imu_wrt_vio[0], (double)s.T_imu_wrt_vio[1], (double)s.T_imu_wrt_vio[2]);
        printf("\n");
    }

    // turn off dropped frame code now we have informed everyone.
    global_error_codes &= ~ERROR_CODE_DROPPED_CAM;

    // if someone has subscribed to the overlay, draw it
    if (pipe_server_get_num_clients(OVERLAY_CH) > 0) {
        // grid can be 1x1, 2x2, or 3x3
        static int overlay_grid_side = 0;
        static bool remove_last_row = false;

        // this only happens on first cycle
        if (!overlay_grid_side) {
            if (images.size() == 1) {  // 1x1
                overlay_grid_side = 1;
            } else if (images.size() <= 4) {  // 2x2
                if (images.size() == 2) {
                    // can chop off the entire bottom row here
                    remove_last_row = true;
                }
                overlay_grid_side = 2;
            } else if (images.size() <= 9) {  // 3x3
                if (images.size() == 6) {
                    // can chop off the entire bottom row here
                    remove_last_row = true;
                }
                overlay_grid_side = 3;
            }
        }

        // we are going to send out one singular image, with resolution = the max res of all input sensors
        static int image_width = 0;
        static int image_height = 0;
        static int chopped_image_height = 0;

        // this only happens on first cycle
        if (!image_height && !image_width) {
            for (unsigned int i = 0; i < images.size(); i++) {
                if (images[i].rows > image_height && images[i].cols > image_width) {
                    image_height = images[i].rows;
                    image_width = images[i].cols;
                }
            }
        }

        if (remove_last_row && !chopped_image_height) {
            chopped_image_height = (image_height / overlay_grid_side) * (overlay_grid_side - 1);
        } else if (!chopped_image_height) {
            chopped_image_height = image_height;
        }
        int orig_bytes = image_width * chopped_image_height;
        draw_meta = meta;
        draw_meta.format = IMAGE_FORMAT_RAW8;
        draw_meta.height = chopped_image_height + DRAW_BONUS_ROWS_TOP + DRAW_BONUS_ROWS_BOT;
        draw_meta.width = image_width;
        draw_meta.size_bytes = draw_meta.width * draw_meta.height;

        // allocate memory for the overlay if this is the first time through
        if (draw_frame == NULL) {
            draw_frame = (uint8_t*)malloc(draw_meta.size_bytes);
        }

        // blank out the top rows
        memset(draw_frame, 0, draw_meta.width * DRAW_BONUS_ROWS_TOP);

        // todo replace with cv::put text
        // steal from tflite example

        // write strings for top bar
        char output_string[128];
        sprintf(output_string, "Q: %d%% XYZ: %6.2lf %6.2lf %6.2lf #Pts: %3d",
                s.quality, (double)s.T_imu_wrt_vio[0], (double)s.T_imu_wrt_vio[1], (double)s.T_imu_wrt_vio[2], n_good_points);
        // cCharacter_dwrite_white(draw_frame, draw_meta.width, draw_meta.height, output_string, 5 + draw_meta.width / 3, 9);
        cCharacter_dwrite_white(draw_frame, draw_meta.width, draw_meta.height, output_string, 5, 9); //apq tracking cam only

        // draw in-state and out-of-state points
        for (unsigned int i = 0; i < curr_pixel_locs.size(); i++) {
            if (curr_pixel_locs[i].point_quality == 2) // 2 is instate
                cv::drawMarker(images[0], cv::Point(curr_pixel_locs[i].pix_loc[0], curr_pixel_locs[i].pix_loc[1]), cv::Scalar(255, 255, 255), cv::MARKER_SQUARE, 8, 2);
            else if (show_extra_points_on_overlay) {
                cv::drawMarker(images[0], cv::Point(curr_pixel_locs[i].pix_loc[0], curr_pixel_locs[i].pix_loc[1]), cv::Scalar(127, 127, 127), cv::MARKER_DIAMOND, 4, 2);
                n_oos_points++;
            }
        }

        cv::Mat combo_image(image_height, image_width, CV_8UC1, cv::Scalar(0));
        cv::Mat dest_roi;

        int row_ind = 0;
        int col_ind = 0;

        // now that features are drawn, add these images into our grid
        for (unsigned int i = 0; i < images.size(); i++) {
            cv::Mat downsampled_image;

            cv::resize(images[i], downsampled_image, cv::Size(image_width / overlay_grid_side, image_height / overlay_grid_side));
            dest_roi = combo_image(cv::Rect(row_ind, col_ind, downsampled_image.cols, downsampled_image.rows));
            downsampled_image.copyTo(dest_roi);

            if (i && i % overlay_grid_side == 0) {
                row_ind = 0;
                col_ind += (image_height) / overlay_grid_side;
            } else {
                row_ind += image_width / overlay_grid_side;
            }
        }

        uint8_t* start_of_img = draw_frame + (draw_meta.width * DRAW_BONUS_ROWS_TOP);
        memcpy(start_of_img, combo_image.data, orig_bytes);

        ///////////////////////////////////////////////////////////////////////////
        // TODO FIGURE OUT WHY THE TOP OF MY IMAGE IS JACKED UP
        ///////////////////////////////////////////////////////////////////////////
        memset(start_of_img, 0, draw_meta.width * 5);
        // this line should not be here

        // blank out bottom rows
        uint8_t* start_of_bottom = draw_frame + (draw_meta.width * DRAW_BONUS_ROWS_TOP) + orig_bytes;
        memset(start_of_bottom, 0, draw_meta.width * DRAW_BONUS_ROWS_BOT);

        // write string for bottom bar
        char oos_pts_string[32];
        if (show_extra_points_on_overlay) {
            sprintf(oos_pts_string, "#Out of State Pts: %3d", n_oos_points);
        } else
            oos_pts_string[0] = 0;

        sprintf(output_string, "ex(ms): %6.1f Gain: %5d %s",
                draw_meta.exposure_ns / 1000000.0, draw_meta.gain, oos_pts_string);
        // cCharacter_dwrite_white(draw_frame, draw_meta.width, draw_meta.height, output_string, 5 + draw_meta.width / 3, (draw_meta.height - DRAW_BONUS_ROWS_BOT) + 9);
        cCharacter_dwrite_white(draw_frame, draw_meta.width, draw_meta.height, output_string, 5, (draw_meta.height - DRAW_BONUS_ROWS_BOT) + 9); //apq tracking only

        // draw out to pipe
        pipe_server_write_camera_frame(OVERLAY_CH, draw_meta, (char*)draw_frame);
    }

    return;
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
    vio_manager_options.init_options.gravity_mag = gravity_mag;
    vio_manager_options.init_options.init_window_time = init_window_time;
    vio_manager_options.init_options.init_imu_thresh = init_imu_thresh;
    vio_manager_options.init_options.init_dyn_num_pose = max_clone_size;
    // TODO expose
    vio_manager_options.init_options.init_max_disparity = 10000;
    vio_manager_options.init_options.init_dyn_use = false;

    /// IMU NOISE OPTIONS ///
    vio_manager_options.imu_noises.sigma_w = imu_sigma_w;
    vio_manager_options.imu_noises.sigma_wb = imu_sigma_wb;
    vio_manager_options.imu_noises.sigma_a = imu_sigma_a;
    vio_manager_options.imu_noises.sigma_ab = imu_sigma_ab;

    vio_manager_options.imu_noises.sigma_w_2 = std::pow(vio_manager_options.imu_noises.sigma_w, 2);
    vio_manager_options.imu_noises.sigma_wb_2 = std::pow(vio_manager_options.imu_noises.sigma_wb, 2);
    vio_manager_options.imu_noises.sigma_a_2 = std::pow(vio_manager_options.imu_noises.sigma_a, 2);
    vio_manager_options.imu_noises.sigma_ab_2 = std::pow(vio_manager_options.imu_noises.sigma_ab, 2);

    /// FEATURE OPTIONS - all use the same struct, can be dif per feature set ///
    // msckf
    vio_manager_options.msckf_options.chi2_multipler = msckf_chi2_multiplier;
    vio_manager_options.msckf_options.sigma_pix = msckf_sigma_px;
    vio_manager_options.msckf_options.sigma_pix_sq = std::pow(vio_manager_options.msckf_options.sigma_pix, 2);

    // slam
    vio_manager_options.slam_options.chi2_multipler = slam_chi2_multiplier;
    vio_manager_options.slam_options.sigma_pix = slam_sigma_px;
    vio_manager_options.slam_options.sigma_pix_sq = std::pow(vio_manager_options.slam_options.sigma_pix, 2);
    // zupt
    vio_manager_options.zupt_options.chi2_multipler = zupt_chi2_multiplier;  // set to 0 for only display based zupt
    vio_manager_options.zupt_options.sigma_pix = zupt_sigma_px;
    vio_manager_options.zupt_options.sigma_pix_sq = std::pow(vio_manager_options.zupt_options.sigma_pix, 2);

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
    vio_manager_options.fast_threshold = fast_threshold;
    vio_manager_options.grid_x = grid_x;
    vio_manager_options.grid_y = grid_y;
    vio_manager_options.min_px_dist = min_px_dist;
    vio_manager_options.knn_ratio = knn_ratio;
    vio_manager_options.downsample_cameras = downsample_cams;
    // vio_manager_options.use_multi_threading_subs = use_multithreading;

    /// FEATURE INITIALIZER OPTIONS ///
    vio_manager_options.featinit_options.triangulate_1d = triangulate_1d;
    vio_manager_options.featinit_options.refine_features = refine_features;
    vio_manager_options.featinit_options.max_runs = max_runs;
    vio_manager_options.featinit_options.init_lamda = init_lamda;
    vio_manager_options.featinit_options.max_lamda = max_lamda;
    vio_manager_options.featinit_options.min_dx = min_dx;
    vio_manager_options.featinit_options.min_dcost = min_dcost;
    vio_manager_options.featinit_options.lam_mult = lam_mult;
    vio_manager_options.featinit_options.min_dist = min_dist;
    vio_manager_options.featinit_options.max_dist = max_dist;
    vio_manager_options.featinit_options.max_baseline = max_baseline;
    vio_manager_options.featinit_options.max_cond_number = max_cond_number;

    /// CAMERA INTRINSICS + EXTRINSICS ///
    // these need to come from voxl-feature-tracker now
    while (main_running){
        // fprintf(stderr, "WAITING TO FILL UP THE CONFIG\n");
        if (has_received_all_params == 1) break;
        usleep(50000); 
    }    

    std::lock_guard<std::mutex> lg(vio_manager_mutex);
    vio_manager_options.num_pts = num_pts;

    std::shared_ptr<ov_core::CamBase> cam_calib_intrinsic;
    for (size_t i = 0; i < cam_info_vec.size(); i++) {
        // ov uses camequi model to represent fisheye cameras, and the radtan model for standard lenses
        if (cam_info_vec[i].is_fisheye) {
            cam_calib_intrinsic = std::make_shared<ov_core::CamEqui>(cam_info_vec[i].cam_calib_intrinsic(8, 0), cam_info_vec[i].cam_calib_intrinsic(9, 0));
        } else {
            cam_calib_intrinsic = std::make_shared<ov_core::CamRadtan>(cam_info_vec[i].cam_calib_intrinsic(8, 0), cam_info_vec[i].cam_calib_intrinsic(9, 0));
        }
        // The camera intrinsics
        cam_calib_intrinsic->set_value(cam_info_vec[i].cam_calib_intrinsic);
        vio_manager_options.camera_intrinsics[i] = cam_calib_intrinsic;
        vio_manager_options.camera_extrinsics[i] = cam_info_vec[i].cam_wrt_imu;
    }

    vio_manager_options.init_options.camera_intrinsics = vio_manager_options.camera_intrinsics;
    vio_manager_options.init_options.camera_extrinsics = vio_manager_options.camera_extrinsics;
    // this needs to be set in both locations, for some reason the updaters address both in different stages
    vio_manager_options.init_options.num_cameras = cam_info_vec.size();
    vio_manager_options.state_options.num_cameras = cam_info_vec.size();

    return vio_manager_options;
}

static void _feat_disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context) {
    fprintf(stderr, "WARNING: disconnected from feature server, resetting VIO\n");

    std::lock_guard<std::mutex> lg(vio_manager_mutex);
    global_error_codes |= ERROR_CODE_CAM_MISSING;
    is_cam_connected = 0;
    ov_msckf::VioManagerOptions vio_manager_options = generate_open_vins_manager_options();
    // HARD RESET
    _hard_reset(false);
    return;
}

static void _imu_disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context) {
    fprintf(stderr, "WARNING: disconnected from imu server, resetting VIO\n");

    global_error_codes |= ERROR_CODE_IMU_MISSING;
    last_imu_timestamp_ns = 0;
    is_imu_connected = 0;
    std::lock_guard<std::mutex> lg(vio_manager_mutex);
    ov_msckf::VioManagerOptions vio_manager_options = generate_open_vins_manager_options();
    // HARD RESET
    _hard_reset(false);
    return;
}

static int create_server_pipes(void) {
    int flags = SERVER_FLAG_EN_CONTROL_PIPE;

    // init extended pipe
    pipe_info_t info1 = {
        OV_VIO_EXTENDED_NAME,       // name
        OV_VIO_EXTENDED_LOCATION,   // location
        "ext_vio_data_t",           // type
        PROCESS_NAME,               // server_name
        VIO_RECOMMENDED_PIPE_SIZE,  // size_bytes
        0                           // server_pid
    };

    if (pipe_server_create(EXTENDED_CH, info1, flags)) {
        _quit(-1);
    }

    // add in optional fields to the info JSON file
    cJSON* json = pipe_server_get_info_json_ptr(EXTENDED_CH);
    cJSON_AddStringToObject(json, "imu", imu_name);
    pipe_server_update_info(EXTENDED_CH);
    pipe_server_set_control_cb(EXTENDED_CH, _control_pipe_cb, NULL);
    pipe_server_set_available_control_commands(EXTENDED_CH, OV_VIO_CONTROL_COMMANDS);

    // init simple pipe
    pipe_info_t info2 = {
        OV_VIO_SIMPLE_NAME,         // name
        OV_VIO_SIMPLE_LOCATION,     // location
        "vio_data_t",               // type
        PROCESS_NAME,               // server_name
        VIO_RECOMMENDED_PIPE_SIZE,  // size_bytes
        0                           // server_pid
    };

    if (pipe_server_create(SIMPLE_CH, info2, flags)) {
        _quit(-1);
    }

    // add in optional fields to the info JSON file
    json = pipe_server_get_info_json_ptr(SIMPLE_CH);
    cJSON_AddStringToObject(json, "imu", imu_name);
    pipe_server_update_info(SIMPLE_CH);
    pipe_server_set_control_cb(SIMPLE_CH, _control_pipe_cb, NULL);
    pipe_server_set_available_control_commands(SIMPLE_CH, OV_VIO_CONTROL_COMMANDS);

    // init overlay pipe
    pipe_info_t info3 = {
        OV_VIO_OVERLAY_NAME,        // name
        OV_VIO_OVERLAY_LOCATION,    // location
        "camera_image_metadata_t",  // type
        PROCESS_NAME,               // server_name
        CAM_PIPE_SIZE,              // size_bytes
        0                           // server_pid
    };

    if (pipe_server_create(OVERLAY_CH, info3, flags)) {
        _quit(-1);
    }

    pipe_server_set_connect_cb(OVERLAY_CH, _overlay_connect_cb, NULL);
    pipe_server_set_disconnect_cb(OVERLAY_CH, _overlay_disconnect_cb, NULL);
    pipe_server_set_control_cb(OVERLAY_CH, _control_pipe_cb, NULL);
    pipe_server_set_available_control_commands(OVERLAY_CH, OV_VIO_CONTROL_COMMANDS);

    return 0;
}

static int connect_client_pipes(void) {
    fprintf(stderr, "connecting client pipes\n");
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

    // heads up, this has major issues
    // when set, can cause camera-server to die unexpectedly
    // if (pipe_set_process_priority(THREAD_PRIORITY_RT_MED) < 0) {
    //     fprintf(stderr, "WARNING: failed to set process priority. Continuing anyways\n");
    // }

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

    // connect to our feature tracker
    pipe_client_set_disconnect_cb(FEATURE_CH, _feat_disconnect_cb, NULL);
    pipe_client_set_simple_helper_cb(FEATURE_CH, _new_feat_data_handler, NULL);

    if (pipe_client_open(FEATURE_CH, FEATURE_LOCATION, PROCESS_NAME, CLIENT_FLAG_EN_SIMPLE_HELPER, 1280 * 800 * 64) != 0) {
        _quit(-1);
    }

    // Create the server pipes
    if (create_server_pipes() < 0) _quit(0);

    // Create the VIO Manager
    vio_manager_options = generate_open_vins_manager_options();
    vio_manager = std::unique_ptr<ov_msckf::VioManager>(new ov_msckf::VioManager(vio_manager_options));

    // Connect to the client pipes and start getting data
    if (connect_client_pipes() < 0) _quit(0);

    /////////////////////////////////////////////////////////////////////////////////////////////
    // temp disable
    // open vins will occasionally stall out on init, and the health monitor
    // will then cause vvpx4 to switch out of position control
    // comment the below thread creation if this behavior occurs
    /////////////////////////////////////////////////////////////////////////////////////////////

    pthread_attr_t tattr;
    pthread_attr_init(&tattr);
    pthread_create(&health_thread, &tattr, _health_thread_func, NULL);

    // send a command BACK out to our feature tracker to startup
    pipe_client_send_control_cmd(FEATURE_CH, VFT_CMD_START);

    // run until start/stop module catches a signal and changes main_running to 0
    while (main_running) usleep(5000000);

    // Shutdown Nicely
    _quit(0);

    return 0;
}
