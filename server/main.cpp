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
#include "open_vins_server.h"
#include "cCharacter.h"

// server channels
#define EXTENDED_CH	0
#define SIMPLE_CH	1
#define OVERLAY_CH	2

// client channels and config
#define IMU_CH	0
#define CAMERA_CH_START_OFFSET 1
#define IMU_PIPE_MIN_PIPE_SIZE (1  * 1024 * 1024) // give ourselves huge buffers
#define CAM_PIPE_SIZE (256 * 1024 * 1024) // give ourselves huge buffers
#define PROCESS_NAME "open-vins-server"

// after 300ms with no response, the health monitor thread assumes mvVISLAM
// has locked up while processing a frame and starts sending messages indicating
// a stall has occured with a failed state
#define STALL_TIMEOUT_NS 300000000

// auto restart if the system fails to init after 2 seconds
#define INIT_FAILURE_TIMEOUT_NS 2000000000

// do not check for blowups until 1 second after VIO claims to have initialized
#define BLOWUP_DETECT_TIMEOUT_NS 1000000000

// not really sure if this will be needed
#define SILENT_STD(x) {\
    FILE *silentfd = fopen("/dev/null","w");     \
    int savedstdoutfd = dup(STDOUT_FILENO);      \
    fflush(stdout);                              \
    dup2(fileno(silentfd), STDOUT_FILENO);       \
    x                                            \
    fflush(stdout);                              \
    fclose(silentfd);                            \
    dup2(savedstdoutfd, STDOUT_FILENO);          \
    close(savedstdoutfd);                        \
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
static std::vector<int64_t> last_cam_timestamps_ns(MAX_CAMERAS);
static volatile int64_t last_real_pose_timestamp_ns = 0;
static volatile int64_t last_sent_timestamp_ns = 0;

// state of imu and camera connections
static volatile int is_imu_connected = 0;
static volatile int is_cam_connected = 0;

// flag set to 1 on reset to indicate to the blowup detector not to check
// for blowups until after VIO actually initializes
static volatile int hard_reset_blowup_flag = 1;

// flag and time when a reset is requested to indicate to the init failure
// detection how long VIO has been trying to init
static volatile int init_failure_detector_reset_flag = 1;
static volatile int64_t time_of_last_reset = 0;
static volatile int last_state = VIO_STATE_FAILED;
static volatile int blowup_detector_flag = 0;
static volatile int64_t time_of_first_okay = 0;

// openvins functions do not seem to be thread safe, protect all calls to the VioManager
std::unique_ptr<ov_msckf::VioManager> vio_manager;
static ov_msckf::VioManagerOptions vio_manager_options;
std::mutex vio_manager_mutex;
static int is_initialized = 1;
static int blank_counter = 0;
static int fade_counter = 0;
static int64_t last_time_alignment_ns = 0;
static int32_t last_frame_frame_id = 0;
static int64_t last_frame_timestamp_ns = 0;

// overlay image stream stuff
#define DRAW_BONUS_ROWS_TOP 32
#define DRAW_BONUS_ROWS_BOT 32
static uint8_t*                 draw_frame   = NULL;
static camera_image_metadata_t  draw_meta;

// set any error codes here for publishing in the data structure in addition
// to errors that come from mvVISLAM
static uint32_t global_error_codes = 0;

// function prototypes
static void _publish(camera_image_metadata_t meta, std::vector<cv::Mat> images);
static int _hard_reset(void);

static bool show_extra_points_on_overlay = true;
static int8_t verbosity_level{static_cast<uint8_t>(ov_core::Printer::PrintLevel::SILENT)};
std::string log_path = "";

// printed if some invalid argument was given
static void _print_usage(void)
{
    printf("\n\
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
-o, --debug-overlay         show extra points on the overlay, not just in-state ones\n\
                              this can be enabled in the config file also\n\
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
            {"timing-imu",no_argument, 0, 'i'},
            {"log_path", required_argument, 0, 'l'},
            {"debug-overlay", no_argument, 0, 'o'},
            {"position", no_argument, 0, 'p'},
            {"timing-cam", no_argument, 0, 't'},
            {"verbosity", required_argument, 0, 'v'},
            {0, 0, 0, 0}
        };

    // set default before we do anything else
    ov_core::Printer::setPrintLevel(ov_core::Printer::PrintLevel::SILENT);

    while (1) {
        int option_index = 0;
        int c = getopt_long(argc, argv, "cdhil:optv:", long_options, &option_index);

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

            case 'o':
                show_extra_points_on_overlay = 1;
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

    // Delete the vio manager so we can close cleanly and quickly
    {
        std::lock_guard<std::mutex> lg(vio_manager_mutex);
        vio_manager.reset(nullptr);
    }

    // Remove this process ID file
    remove_pid_file(PROCESS_NAME);

    if (ret == 0) printf("Exiting Cleanly\n");
    exit(ret);
    return;
}


///////////////////////////////////////////////////////////////////////////////
// this needs to become basically how "spread" are our features across the FOV
///////////////////////////////////////////////////////////////////////////////
// static float _qvio_quality(struct mvVISLAMPose pose)
// {
// 	// vio quality uses covariance and a reset counter
// 	// the whole covarience matrix should be zero in case of blowup
// 	if(pose.errCovPose[3][3]<=0.0f || pose.errCovPose[4][4]<=0.0f || pose.errCovPose[5][5]<=0.0f){
// 		return -1.0f;
// 	}

// 	// pick the maximum (worst) of the velocity diagonal entries
// 	float max = pose.errCovVelocity[0][0];
// 	if(pose.errCovVelocity[1][1]>max) max = pose.errCovVelocity[1][1];
// 	if(pose.errCovVelocity[2][2]>max) max = pose.errCovVelocity[2][2];

// 	// covarience is very small. Scale it by 10^5 to make the output more readable
// 	float conf = 0.00001f / max;

// 	// return normal value
// 	return conf;
// }



// pose data is published from the same thread that does the camera processing
// and pose estimation. That freezes, sometimes for over a second, during blowups
// so this thread exists to keep data coming out during that situation, warning
// consumers that there is an issue.
// this does NOT monitor for blowup criteria, that's done in the camera thread
// as soon as a new pose is calculated. This thread is to warn when that
// camera thread freezes.
static void* _health_thread_func(__attribute__((unused)) void* ctx)
{
    while(main_running){
        usleep(30000); // run about the same speed as the camera

        int64_t current_time = _apps_time_monotonic_ns();
        int64_t delay_ns = current_time - last_real_pose_timestamp_ns;

        if(init_failure_detector_reset_flag){
            uint64_t time_since_reset = current_time - time_of_last_reset;
            if(time_since_reset > INIT_FAILURE_TIMEOUT_NS){
                fprintf(stderr, "WARNING failed to init in time, trying again\n");
                _hard_reset();
                continue;
            }
        }

        // If last packet is recent enough, nothing to worry about.
        // Otherwise, send out failure packets, this inlcudes global error codes
        // indicating if we are waiting for cam or IMU data
        if(delay_ns < STALL_TIMEOUT_NS) continue;

        // flag that we've sent a packet with the current timestamp
        last_sent_timestamp_ns = current_time;

        qvio_data_t d;	// complete "extended" vio MPA packet
        vio_data_t s;	// simplified vio packet

        // make sure we start with clean data structs and apply any global error codes
        // full extended qvio packet
        memset(&d,0,sizeof(d));
        d.magic_number = QVIO_MAGIC_NUMBER;
        d.timestamp_ns = current_time;
        d.error_code = global_error_codes | ERROR_CODE_STALLED;
        d.state = VIO_STATE_FAILED;
        d.quality = -1.0f;
        // simple lib modal pipe standard vio packet
        memset(&s,0,sizeof(s));
        s.magic_number = VIO_MAGIC_NUMBER;
        s.timestamp_ns = current_time;
        s.error_code = global_error_codes | ERROR_CODE_STALLED;
        s.state = VIO_STATE_FAILED;
        s.quality = -1.0f;

        // send to both pipes
        pipe_server_write(EXTENDED_CH, (char*)&d, sizeof(qvio_data_t));
        pipe_server_write(SIMPLE_CH,   (char*)&s, sizeof(vio_data_t));

        // turn off dropped cam frame code now we have informed everyone.
        global_error_codes &= ~ERROR_CODE_DROPPED_CAM;
    }

    return NULL;
}


static int _hard_reset(void)
{
    // lock the mutex before calling any ov api calls
    std::lock_guard<std::mutex> lg(vio_manager_mutex);

    // stop it if it's running
    if(is_initialized){
        is_initialized = 0;
        vio_manager.reset(nullptr);
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

    if(vio_manager == NULL){
        fprintf(stderr, "Error creating vio_manager object\n");
        _quit(-1);
    }

    is_initialized = 1;
    return 0;
}


// convert roll/pitch/yaw in degrees from extrinsics config file to axis-angle
static int _tait_bryan_xyz_intrinsic_to_axis_angle(double tb_deg[3], float aa[3])
{
    if(tb_deg==NULL||aa==NULL){
        fprintf(stderr,"ERROR: in %s, received NULL pointer\n", __FUNCTION__);
        return -1;
    }

    // convert from degree to rad
    double tb_rad[3];
    tb_rad[0] = tb_deg[0]*M_PI/180.0;
    tb_rad[1] = tb_deg[1]*M_PI/180.0;
    tb_rad[2] = tb_deg[2]*M_PI/180.0;

    double c1 = cos(tb_rad[0]/2.0);
    double s1 = sin(tb_rad[0]/2.0);
    double c2 = cos(tb_rad[1]/2.0);
    double s2 = sin(tb_rad[1]/2.0);
    double c3 = cos(tb_rad[2]/2.0);
    double s3 = sin(tb_rad[2]/2.0);
    double c1c2 = c1*c2;
    double s1s2 = s1*s2;
    double s1c2 = s1*c2;
    double c1s2 = c1*s2;

    // XYZ
    double w = c1c2*c3 - s1s2*s3;
    double x = s1c2*c3 + c1s2*s3;
    double y = c1s2*c3 - s1c2*s3;
    double z = c1c2*s3 + s1s2*c3;

    double norm = x*x+y*y+z*z;
    if(norm < 0.0001){
        aa[0]=0.0f;
        aa[1]=0.0f;
        aa[2]=0.0f;
        return 0;
    }

    double angle = 2.0*acos(w);
    double scale = angle/sqrt(norm);
    aa[0] = x*scale;
    aa[1] = y*scale;
    aa[2] = z*scale;
    return 0;
}


// control listens for reset commands
static void _control_pipe_cb(__attribute__((unused)) int ch, char* string, \
                             int bytes, __attribute__((unused)) void* context)
{
    // remove the trailing newline from echo
    if(bytes>1 && string[bytes-1]=='\n'){
        string[bytes-1]=0;
    }

    // soft reset is not currently allowed
    // if(strncmp(string, RESET_VIO_SOFT, strlen(RESET_VIO_SOFT))==0){
    // 	printf("Client requested soft reset\n");
    // 	pthread_mutex_lock(&mv_mtx);
    // 	mvVISLAM_Reset(mv_vislam_ptr, 0);
    // 	pthread_mutex_unlock(&mv_mtx);
    // 	return;
    // }
    if(strncmp(string, RESET_VIO_HARD, strlen(RESET_VIO_HARD))==0){
        printf("Client requested hard reset\n");
        _hard_reset(); // close and restart the object
        return;
    }

    // not testing these conditions yet
    // if(strcmp(string, BLANK_VIO_CAM_COMMAND)==0){
    // 	printf("Client requested image blank test\n");
    // 	blank_counter = 90;
    // 	return;
    // }

    // if(strcmp(string, FADE_VIO_CAM_COMMAND)==0){
    // 	printf("Client requested image fade test\n");
    // 	fade_counter = 255;
    // 	return;
    // }

    printf("WARNING: Server received unknown command through the control pipe!\n");
    printf("got %d bytes. Command is: %s\n", bytes, string);
    return;
}


// print when a new client connects to us
static void _overlay_connect_cb(	__attribute__((unused))int ch, \
                            __attribute__((unused))int client_id,
                            char* client_name,\
                            __attribute__((unused)) void* context)
{
    printf("client \"%s\" connected to overlay\n", client_name);
    return;
}


// print when a client disconnects from us
static void _overlay_disconnect_cb(	__attribute__((unused))int ch, \
                            __attribute__((unused))int client_id,
                            char* client_name,\
                            __attribute__((unused)) void* context)
{
    printf("client \"%s\" disconnected from overlay\n", client_name);
    return;
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
    if(!is_cam_connected) return;
    if(!is_initialized) return;

    std::lock_guard<std::mutex> lg(vio_manager_mutex);

    // time this in debug mode
    int64_t time_before, process_time;
    if(en_debug_timing_imu) time_before = _apps_time_monotonic_ns();
    
    for (int i = 0; i < n_packets; i++) {
        // check if we somehow got an out-of-order imu sample and reject it
        if((int64_t)data_array[i].timestamp_ns <= last_imu_timestamp_ns){
            double dt = (last_imu_timestamp_ns - data_array[i].timestamp_ns)/1000000.0;
            fprintf(stderr, "WARNING out-of-order imu %fms before previous\n", dt);
            continue;
        }
        // Create the data struct that we will use for ingesting data into the vio manager
        ov_core::ImuData vio_manager_data;
        vio_manager_data.timestamp = data_array[i].timestamp_ns / 1000000000.0;  // (seconds)

        //send in our imu data with z flipped
        vio_manager_data.wm(0, 0) = data_array[i].gyro_rad[0];
        vio_manager_data.wm(1, 0) = -data_array[i].gyro_rad[1];
        vio_manager_data.wm(2, 0) = -data_array[i].gyro_rad[2];

        vio_manager_data.am(0, 0) = data_array[i].accl_ms2[0];
        vio_manager_data.am(1, 0) = -data_array[i].accl_ms2[1];
        vio_manager_data.am(2, 0) = -data_array[i].accl_ms2[2];

        vio_manager->feed_measurement_imu(vio_manager_data);

        last_imu_timestamp_ns = data_array[i].timestamp_ns;
    }

    if(en_debug_timing_imu){
        process_time = _apps_time_monotonic_ns() - time_before;
        printf("IMU proc time %6.2fms for %d samples\n", ((double)process_time)/1000000.0, n_packets);
    }
    return;
}


#ifdef PLATFORM_QRB5165
// for qrb5165 only (right now) set the camera processing thread to run on 
// CPU 7 which is the fastest core
static void _check_and_set_affinity(void)
{
    // only do this once
    static int has_set = 0;
    if(has_set) return;

    cpu_set_t cpuset;
    pthread_t thread;
    thread = pthread_self();

    /* Set affinity mask to include CPUs 7 only */
    CPU_ZERO(&cpuset);
    CPU_SET(7, &cpuset);
    if(pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset)){
        perror("pthread_setaffinity_np");
    }

    /* Check the actual affinity mask assigned to the thread */
    if(pthread_getaffinity_np(thread, sizeof(cpu_set_t), &cpuset)){
        perror("pthread_getaffinity_np");
    }
    printf("Camera processing thread is now locked to the following cores:");
    for (int j = 0; j < CPU_SETSIZE; j++){
        if(CPU_ISSET(j, &cpuset)) printf(" %d", j);
    }
    printf("\n");

    // only do this once on start
    has_set = 1;

    return;
}
#endif


static void _new_camera_data_handler(int ch, camera_image_metadata_t meta, char* frame, __attribute__((unused)) void* context) {

#ifdef PLATFORM_QRB5165
    _check_and_set_affinity();
#endif

    int camera_index = ch - CAMERA_CH_START_OFFSET;

    // Make the timestamp be the center of the exposure
    int64_t cam_timestamp_ns = meta.timestamp_ns;
    cam_timestamp_ns += meta.exposure_ns / 2;

    if (cam_timestamp_ns < (_apps_time_monotonic_ns() - 1000000000)) {
        global_error_codes |= ERROR_CODE_DROPPED_CAM;
        pipe_client_flush(ch);
        fprintf(stderr, "ERROR detected frame older than 1s, flushing cam pipe\n");
        return;
    }

    // flag that camera data is active, skip frame is imu is disconnected
    is_cam_connected = 1;
    global_error_codes &= ~ERROR_CODE_CAM_MISSING;
    if(!is_imu_connected) return;


    // don't let image go in until IMU has caught up
    // if cam-imu alignment is POSITIVE that means the camera timestamp is early
    // and the image was actually taken after the reported timestamp
    // need a way to get the cam imu dif
    while (last_imu_timestamp_ns < (cam_timestamp_ns + last_time_alignment_ns)) {
        // don't get stuck here forever
        if (!main_running) return;
        if (!is_imu_connected) return;
        if (cam_timestamp_ns < (_apps_time_monotonic_ns() - 300000000)) {
            global_error_codes |= ERROR_CODE_DROPPED_CAM;
            pipe_client_flush(ch);
            fprintf(stderr, "ERROR waited more than 0.3 seconds for imu to catch up, flushing camera pipe\n");
            return;
        }
        usleep(5000);
    }

    ov_core::CameraData vio_manager_data;

    vio_manager_data.timestamp = cam_timestamp_ns / 1000000000.0;
    vio_manager_data.sensor_ids.push_back(camera_index);


    ////////////////////////////////////////////////////////////////////
    // TODO
    // add a config file flag for bottom/top/both for stereo pairs
    ////////////////////////////////////////////////////////////////////
    if (meta.format == IMAGE_FORMAT_RAW8) {

        // Unpack the data into an opencv image Mat
        cv::Mat img(meta.height, meta.width, CV_8UC1, frame);
        // Create a mask for the ingestion.  We want the full image to be ingested
        cv::Mat mask(meta.height, meta.width, CV_8UC1, cv::Scalar(0));

        vio_manager_data.images.push_back(img);
        vio_manager_data.masks.push_back(mask);
    } else if (meta.format == IMAGE_FORMAT_STEREO_RAW8) {
        meta.width *= 2;
        vio_manager_data.sensor_ids.push_back(camera_index + 1);

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
    } else if (meta.format == IMAGE_FORMAT_STEREO_NV12 || meta.format == IMAGE_FORMAT_STEREO_NV21) {
        meta.width *= 2;
        meta.size_bytes = meta.height * meta.width;
        vio_manager_data.sensor_ids.push_back(camera_index + 1);

        // Unpack the data into opencv image Mats
        cv::Mat img(meta.height, meta.width, CV_8UC1, frame);
        cv::Mat img2(meta.height, meta.width, CV_8UC1, frame + (meta.width * meta.height * 3 / 2)); //I think this is right, images are coming out funky in from replay

        // Create masks for the ingestion. We want both full images to be ingested
        cv::Mat mask(meta.height, meta.width, CV_8UC1, cv::Scalar(0));
        cv::Mat mask2(meta.height, meta.width, CV_8UC1, cv::Scalar(0));

        vio_manager_data.images.push_back(img);
        vio_manager_data.masks.push_back(mask);
        vio_manager_data.images.push_back(img2);
        vio_manager_data.masks.push_back(mask2);
    }

    // Ingest the data
    std::lock_guard<std::mutex> lg(vio_manager_mutex);

    int64_t time_before = _apps_time_monotonic_ns();
    vio_manager->feed_measurement_camera(vio_manager_data);
    int64_t process_time = _apps_time_monotonic_ns() - time_before;

    // save details of the last image we passed into ov
    last_cam_timestamps_ns[camera_index] = cam_timestamp_ns;
    last_frame_frame_id = meta.frame_id;
    last_frame_timestamp_ns = meta.timestamp_ns;

    if(process_time > 500000000){
        // if image processing took more than half a second, something went wrong
        // and we should drop frames
        global_error_codes |= ERROR_CODE_DROPPED_CAM;
        pipe_client_flush(ch);
        fprintf(stderr, "ERROR slow image proc time: %6.2fms\n", ((double)process_time)/1000000.0);
        fprintf(stderr, "Flushing camera frames\n");
    }
    else if(process_time > 50000000){
        // warn if image processing was a little slow, this happens from time
        // to time and isn't a big deal
        fprintf(stderr, "WARNING slow image proc time: %6.2fms\n", ((double)process_time)/1000000.0);
    }

    // so basically when we call this, we need to make sure that we update the meta for the size of our entire concatted image
    // meta is going to be updated in the handling per image type, if stereo we gotta multiply the width by 2 and recalc size bytes if nv
    _publish(meta, vio_manager_data.images);
    return;
}


static void _publish(camera_image_metadata_t meta, std::vector<cv::Mat> images)
{
    std::shared_ptr<ov_msckf::State> current_state = {nullptr};
    qvio_data_t d;	// complete "extended" vio MPA packet
    vio_data_t s;	// simplified vio packet
    int nPoints;
    int n_good_points = 0;
    int n_oos_points = 0;
    int i,j;

    // make sure we start with clean data structs and apply any global error codes
    // full extended qvio packet
    memset(&d,0,sizeof(d));
    d.magic_number = QVIO_MAGIC_NUMBER;
    d.error_code = global_error_codes;
    // simple lib modal pipe standard vio packet
    memset(&s,0,sizeof(s));
    s.magic_number = VIO_MAGIC_NUMBER;
    s.error_code = global_error_codes;

    // Grab the state
    current_state = vio_manager->get_state();

    // get features
    // not sure how to calculate quality of these features, may need to expose another function for some
    // better access to the feature database
    std::vector<Eigen::Vector3d> msckf_last_features = vio_manager->get_good_features_MSCKF();
    std::vector<Eigen::Vector3d> curr_slam_features = vio_manager->get_features_SLAM();
    std::vector<std::pair<int, cv::Point2f>> curr_pixel_locs = vio_manager->get_pixel_loc_features();

    // fprintf(stderr, "msckf_last_features: %ld\n", msckf_last_features.size());
    // fprintf(stderr, "curr_slam_features: %ld\n", curr_slam_features.size());
    // fprintf(stderr, "active tracked features: %ld\n", feat_posinG.size());

    n_good_points = msckf_last_features.size() + curr_slam_features.size();

    // record that we just got a successful pose and point cloud
    last_real_pose_timestamp_ns = static_cast<int64_t>(current_state->_timestamp * 1e9);

    // if we can, get some out of state features here too???
    // TODO - check if the tracked features can be out of state or not

    // check if its initialized or not
    if(!vio_manager->initialized()) d.state = VIO_STATE_INITIALIZING;
    else d.state = VIO_STATE_OK;

    // TODO 
    // dont have direct access to covariances yet

    // // sometimes qvio will report covariance as invalid but state is still OKAY
    // // this is NOT alright, in this case manually set the state to failed.
    // if(p.errCovPose[3][3]<=0.0f || p.errCovPose[4][4]<=0.0f || p.errCovPose[5][5]<=0.0f){
    //     d.state=VIO_STATE_FAILED;
    // }

    // we finished initializing, no longer check for init timeout
    if(d.state == VIO_STATE_OK){
        init_failure_detector_reset_flag = 0;
    }

    // if we just went from good to failed, treat this like a reset for the
    // init failure detector so it can timeout the same as VIO tries to re-init
    // itself after it's own internal reset
    if(last_state!=VIO_STATE_FAILED && d.state==VIO_STATE_FAILED){
        init_failure_detector_reset_flag = 1;
        time_of_last_reset = static_cast<int64_t>(current_state->_timestamp * 1e9);
    }

    // record time when vio claimed to have initialized and only check for
    // for blowups some time after this
    if(last_state!=VIO_STATE_OK && d.state==VIO_STATE_OK){
        blowup_detector_flag = 1;
        time_of_first_okay = static_cast<int64_t>(current_state->_timestamp * 1e9);
    }
    last_state = d.state;

    // TODO 
    // dont have direct access to covariances yet

    // while VIO state is OK, do our own additional blowup checks if enough
    // time has passed since the init
    // int64_t time_since_first_okay = _apps_time_monotonic_ns() - time_of_first_okay;
    // if(blowup_detector_flag && time_since_first_okay>BLOWUP_DETECT_TIMEOUT_NS){
    //     int code = _check_for_blowup(p,n_good_points);
    //     if(code){
    //         pthread_mutex_unlock(&mv_mtx);
    //         _hard_reset();
    //         d.state = VIO_STATE_FAILED;
    //         d.error_code |= code;
    //     }
    // }


    // don't send packets from the past, this can happen when qvio stalls
    // during a reset
    if(static_cast<int64_t>(current_state->_timestamp * 1e9)<last_sent_timestamp_ns){
        fprintf(stderr, "WARNING skipping pose data from the past\n");
        return;
    }

    // All checks passed, after this point this function should not return
    // until the end
    last_sent_timestamp_ns = static_cast<int64_t>(current_state->_timestamp * 1e9);

    // populate some other data
    // d.quality = _qvio_quality(p); // quality metric is still undefined
    d.quality = 1;
    d.timestamp_ns = static_cast<int64_t>(current_state->_timestamp * 1e9);
    d.imu_cam_time_shift_s = current_state->_calib_dt_CAMtoIMU->value()(0);
    last_time_alignment_ns = current_state->_calib_dt_CAMtoIMU->value()(0) * 1e9;
    d.n_feature_points = n_good_points;  // not sure yet
    d.last_cam_frame_id = last_frame_frame_id;
    d.last_cam_timestamp_ns = last_frame_timestamp_ns;

    Eigen::MatrixXf::Map(reinterpret_cast<float*>(d.R_imu_to_vio), 3, 3) = current_state->_imu->Rot_fej().cast<float>();
    Eigen::MatrixXf::Map(d.T_imu_wrt_vio, 3, 1) = current_state->_imu->pos().cast<float>();
    Eigen::MatrixXf::Map(d.vel_imu_wrt_vio, 3, 1) = current_state->_imu->vel().cast<float>();
    Eigen::MatrixXf::Map(d.T_cam_wrt_imu, 3, 1) = current_state->_calib_IMUtoCAM[0]->pos().cast<float>();
    Eigen::MatrixXf::Map(reinterpret_cast<float*>(d.R_cam_to_imu), 3, 3) = ov_core::quat_2_Rot(current_state->_calib_IMUtoCAM[0]->quat()).cast<float>();
    Eigen::MatrixXf::Map(reinterpret_cast<float*>(d.gyro_bias), 3, 3) = current_state->_imu->bias_g_fej().cast<float>();
    Eigen::MatrixXf::Map(reinterpret_cast<float*>(d.accl_bias), 3, 3) = current_state->_imu->bias_a_fej().cast<float>();

    // GRAVITY ALIGNMENT WITH QVIO
    // Z AND Y AXES MUST BE FLIPPED
    d.T_imu_wrt_vio[1] = -d.T_imu_wrt_vio[1];
    d.T_imu_wrt_vio[2] = -d.T_imu_wrt_vio[2];

    // memcpy(d.pose_covariance,	p.errCovPose,		sizeof(float)*6*6);
    // memcpy(d.vel_covariance,	p.errCovVelocity,	sizeof(float)*3*3);
    // LEAVING OUT ANGULAR VELOCITY FOR NOW
    // memcpy(d.imu_angular_vel,	p.angularVelocity,	sizeof(float)*3);
    // memcpy(d.gravity_vector,	p.gravity,			sizeof(float)*3);
    // memcpy(d.gravity_covariance,p.errCovGravity,	sizeof(float)*3*3);

    // limit the number of features to what fits in our pipe packet
    d.n_total_features = curr_slam_features.size() + msckf_last_features.size();
    if(d.n_total_features > QVIO_MAX_REPORTED_FEATURES){
        d.n_total_features = QVIO_MAX_REPORTED_FEATURES;
    }
    unsigned int k;
    for (k = 0; k < curr_slam_features.size(); k++){
        if (k >= QVIO_MAX_REPORTED_FEATURES) break;
        d.features[k].tsf[0] = static_cast<float>(curr_slam_features[k].x());
        d.features[k].tsf[1] = static_cast<float>(curr_slam_features[k].y());
        d.features[k].tsf[2] = static_cast<float>(curr_slam_features[k].z());
    }
    if (k < QVIO_MAX_REPORTED_FEATURES){
        for (unsigned int i = 0; i < msckf_last_features.size();i++){
            if (k >= QVIO_MAX_REPORTED_FEATURES) break;
            d.features[i].tsf[0] = static_cast<float>(msckf_last_features[i].x());
            d.features[i].tsf[1] = static_cast<float>(msckf_last_features[i].y());
            d.features[i].tsf[2] = static_cast<float>(msckf_last_features[i].z());
            k++;
        }
    }
    // memcpy(d.features, pPoints, d.n_total_features*sizeof(qvio_feature_t));

    // also make a simplified struct for the common interface
    s.error_code				= d.error_code;
    s.quality					= d.quality;
    s.timestamp_ns				= d.timestamp_ns;
    memcpy(s.T_imu_wrt_vio,		d.T_imu_wrt_vio,	sizeof(float)*3);
    memcpy(s.R_imu_to_vio,		d.R_imu_to_vio,		sizeof(float)*3*3);
    memcpy(s.vel_imu_wrt_vio,	d.vel_imu_wrt_vio,	sizeof(float)*3);
    // memcpy(s.imu_angular_vel,	d.imu_angular_vel,	sizeof(float)*3);
    // memcpy(s.gravity_vector,	d.gravity_vector,	sizeof(float)*3);
    memcpy(s.T_cam_wrt_imu,		d.T_cam_wrt_imu,	sizeof(float)*3);
    memcpy(s.R_cam_to_imu,		d.R_cam_to_imu,		sizeof(float)*3*3);
    // s.n_feature_points			= d.n_feature_points;
    // s.state						= d.state;

    // send to both pipes
    pipe_server_write(EXTENDED_CH, (char*)&d, sizeof(qvio_data_t));
    pipe_server_write(SIMPLE_CH,   (char*)&s, sizeof(vio_data_t));

    // // for debug only
    // if(en_debug){
    //     printf("state: ");
    //     pipe_print_vio_state(d.state);
    //     printf(" err: ");
    //     pipe_print_vio_error(d.error_code);
    //     printf("\n");
    // }
    // if(en_debug_pos){
    //     printf("%6.3f %6.3f %6.3f ", (double)d.T_imu_wrt_vio[0],(double)d.T_imu_wrt_vio[1],(double)d.T_imu_wrt_vio[2]);
    //     printf("\n");
    // }

    // turn off dropped frame code now we have informed everyone.
    global_error_codes &= ~ERROR_CODE_DROPPED_CAM;

    // if someone has subscribed to the overlay, draw it
    if(pipe_server_get_num_clients(OVERLAY_CH) > 0){


        int orig_bytes = meta.height * meta.width;
        draw_meta = meta;
        draw_meta.format = IMAGE_FORMAT_RAW8;
        draw_meta.height = meta.height + DRAW_BONUS_ROWS_TOP + DRAW_BONUS_ROWS_BOT;
        draw_meta.size_bytes = draw_meta.width * draw_meta.height;

        // allocate memory for the overlay if this is the first time through
        if(draw_frame == NULL){
            draw_frame = (uint8_t *)malloc(draw_meta.size_bytes);
        }

        // blank out the top rows
        memset(draw_frame, 0, draw_meta.width * DRAW_BONUS_ROWS_TOP);

        //memcpy image to global
        uint8_t* start_of_img = draw_frame + (draw_meta.width * DRAW_BONUS_ROWS_TOP);

        // is this safe ???
        for (unsigned int i = 0; i < images.size(); i++){
            start_of_img = draw_frame + ((i+1) * (draw_meta.width * DRAW_BONUS_ROWS_TOP));
            memcpy(start_of_img, images[0].data, orig_bytes);
        }

        // blank out bottom rows
        uint8_t* start_of_bottom = draw_frame + (draw_meta.width * DRAW_BONUS_ROWS_TOP) + orig_bytes;
        memset(start_of_bottom, 0, draw_meta.width * DRAW_BONUS_ROWS_BOT);

        // write strings for top bar
        char output_string[128];
        sprintf(output_string, "Q: %9.5lf XYZ: %6.2lf %6.2lf %6.2lf #Pts: %3d",
                            (double)d.quality, (double)d.T_imu_wrt_vio[0], (double)d.T_imu_wrt_vio[1], (double)d.T_imu_wrt_vio[2], n_good_points);
        cCharacter_dwrite_white(draw_frame, draw_meta.width, draw_meta.height, output_string, 5, 9);

        // draw in-state and out-of-state points
        for (unsigned int i = 0; i < curr_pixel_locs.size(); i++){
            if (curr_pixel_locs[i].first == INS_FEAT_ID) cCharacter_draw_dchar(draw_frame, draw_meta.width, draw_meta.height, 'O', curr_pixel_locs[i].second.x - 4, curr_pixel_locs[i].second.y + DRAW_BONUS_ROWS_TOP - 6, 255);
            else if (show_extra_points_on_overlay){
               cCharacter_draw_plus(draw_frame, draw_meta.width, draw_meta.height, curr_pixel_locs[i].second.x, curr_pixel_locs[i].second.y + DRAW_BONUS_ROWS_TOP - 6);
               n_oos_points++;
            } 
        }

        // write string for bottom bar
        char oos_pts_string[32];
        if(show_extra_points_on_overlay){
            sprintf(oos_pts_string, "#Out of State Pts: %3d", n_oos_points);
        }
        else oos_pts_string[0]=0;

         sprintf(output_string, "ex(ms): %6.1f Gain: %5d %s",
                            draw_meta.exposure_ns/1000000.0, draw_meta.gain, oos_pts_string);
        cCharacter_dwrite_white(draw_frame, draw_meta.width, draw_meta.height, output_string, 5, (draw_meta.height - DRAW_BONUS_ROWS_BOT) + 9);

        // draw out to pipe
        pipe_server_write_camera_frame(OVERLAY_CH, draw_meta, (char*) draw_frame);
    }

    return;

}

// static void _publish_vio_data() {
//     int64_t next_time = _apps_time_monotonic_ns() + (int64_t)(1000000000 / odr_hz);
//     int64_t time_before, process_time;

//     while (main_running) {

//         // try to maintain output data rate (odr)
//         int64_t current_time = _apps_time_monotonic_ns();
//         next_time += (int64_t)(1000000000.0f / odr_hz);
//         // uh oh, we fell behind, warn and get back on track
//         if (next_time < current_time) {
//             fprintf(stderr, "WARNING: output data thread fell behind\n");
//             next_time = current_time + (int64_t)(1000000000.0f / odr_hz);
//             // continue;
//         }
//         _nanosleep(next_time - current_time);

//         // If there is no vio manager then there is nothing to publish..
//         if (!vio_manager) continue;

//         //  We want to fill in the data
//         vio_data_t vio_data;

//         vio_data.magic_number = VIO_MAGIC_NUMBER;
//         vio_data.error_code = global_error_codes;

//         // The gravity vector is fixed since the vio frame is already gravity aligned
//         vio_data.gravity_vector[0] = 0;
//         vio_data.gravity_vector[1] = 0;
//         vio_data.gravity_vector[2] = -1;

//         // Set the quality to be fixed to a positive number
//         vio_data.quality = 1;

//         // same kinda setup for doing the eval work
//         ov_eval_data eval_packet;

//         eval_packet.magic_number = VIO_MAGIC_NUMBER;
//         eval_packet.done = 0; // for open-vins-tuner to know when it can start terminating processes and fire up the next round

//         cv::Mat draw_image;
//         cv::Mat open_vins_overlay;

//         std::vector<Eigen::Vector3d> features_msckf;
//         std::vector<Eigen::Vector3d> features_slam;

//         std::shared_ptr<ov_msckf::State> current_state = {nullptr};

//         time_before = _apps_time_monotonic_ns();
//         // @todo Figure out a way to set the bad VIO state
//         if (vio_manager->initialized()) {
//             vio_data.state = VIO_STATE_OK;
//         } else {
//             vio_data.state = VIO_STATE_INITIALIZING;
//             pipe_server_write(SIMPLE_OUTPUT_CH, (char*)&vio_data, sizeof(vio_data_t));
//             continue;
//         }

//         // grab the pose and the feature points
//         {
//             std::lock_guard<std::mutex> lg2(cam_mutex);


//             open_vins_overlay = vio_manager->get_historical_viz_image();
//             // double temp;
//             // vio_manager->get_active_image(temp, draw_image);
//             // features_msckf = vio_manager->get_good_features_MSCKF();
//             // features_slam = vio_manager->get_features_SLAM();
//         }

//         process_time = _apps_time_monotonic_ns() - time_before;

//         if(process_time > 100000000){
//             fprintf(stderr, "WARNING slow image proc time: %6.2fms\n", ((double)process_time)/1000000.0);
//             // return;
//         }
//             current_state = vio_manager->get_state();

//         if(en_debug){
//             printf("CAM proc time %6.2fms\n", ((double)process_time)/1000000.0);
//         }

//         if (!en_eval){
//             if (open_vins_overlay.empty()) continue;
//             // for poor network, portal needs smaller images
//             if (open_vins_overlay.cols > 640) {
//                 cv::resize(open_vins_overlay, open_vins_overlay, cv::Size(), 0.5, 0.5);
//             }

//             camera_image_metadata_t meta_;
//             meta_.timestamp_ns = _apps_time_monotonic_ns();
//             meta_.magic_number = CAMERA_MAGIC_NUMBER;
//             meta_.width = open_vins_overlay.cols;
//             meta_.height = open_vins_overlay.rows;
//             meta_.size_bytes = meta_.width * meta_.height * 3;
//             meta_.stride = meta_.width * 3;
//             meta_.format = IMAGE_FORMAT_RGB;

//             pipe_server_write_camera_frame(OVERLAY_OUTPUT_CH, meta_, (char*)open_vins_overlay.data);
//         }

//         if (en_eval) {
//             eval_packet.timestamp_ns = static_cast<int64_t>(current_state->_timestamp * 1e9);

//             // GRAVITY ALIGNMENT WITH QVIO
//             // Z AND Y AXES MUST BE FLIPPED
//             eval_packet.T_imu_wrt_vio[0] = current_state->_imu->pos()[0];
//             eval_packet.T_imu_wrt_vio[1] = -current_state->_imu->pos()[1];
//             eval_packet.T_imu_wrt_vio[2] = -current_state->_imu->pos()[2];

//             eval_packet.q[0] = current_state->_imu->quat()[0];
//             eval_packet.q[1] = current_state->_imu->quat()[1];
//             eval_packet.q[2] = current_state->_imu->quat()[2];
//             eval_packet.q[3] = current_state->_imu->quat()[3];
//             last_time_alignment_ns = current_state->_calib_dt_CAMtoIMU->value()(0) * 1e9;

//             pipe_server_write(SIMPLE_EVAL_CH, (char*)&eval_packet, sizeof(ov_eval_data));
//             continue;
//         } else {
//             vio_data.timestamp_ns = static_cast<int64_t>(current_state->_timestamp * 1e9);
//             Eigen::MatrixXf::Map(vio_data.T_imu_wrt_vio, 3, 1) = current_state->_imu->pos().cast<float>();

//             // GRAVITY ALIGNMENT WITH QVIO
//             // Z AND Y AXES MUST BE FLIPPED
//             vio_data.T_imu_wrt_vio[1] = -vio_data.T_imu_wrt_vio[1];
//             vio_data.T_imu_wrt_vio[2] = -vio_data.T_imu_wrt_vio[2];

//             Eigen::MatrixXf::Map(reinterpret_cast<float*>(vio_data.R_imu_to_vio), 3, 3) = current_state->_imu->Rot_fej().cast<float>();
//             Eigen::MatrixXf::Map(vio_data.vel_imu_wrt_vio, 3, 1) = current_state->_imu->vel().cast<float>();
//             Eigen::MatrixXf::Map(vio_data.T_cam_wrt_imu, 3, 1) = current_state->_calib_IMUtoCAM[0]->pos().cast<float>();
//             Eigen::MatrixXf::Map(reinterpret_cast<float*>(vio_data.R_cam_to_imu), 3, 3) = ov_core::quat_2_Rot(current_state->_calib_IMUtoCAM[0]->quat()).cast<float>();
//             vio_data.n_feature_points = features_slam.size();

//             // LEAVING OUT ANGULAR VELOCITY FOR NOW
//             // rc_ov_t closest_imu_packet;
//             // int ret = rc_ov_ringbuf_get_ov_at_time(&imu_buf, vio_data.timestamp_ns, &closest_imu_packet);
//             // if (ret < 0) {
//             //     fprintf(stderr, "ERROR fetching from ringbuffer\n");
//             //     if (ret == -2) {
//             //         printf("there wasn't sufficient data in the buffer\n");
//             //     }
//             //     if (ret == -3) {
//             //         printf("the requested timestamp was too new\n");
//             //     }
//             //     if (ret == -4) {
//             //         printf("the requested timestamp was too old\n");
//             //     }
//             //     continue;
//             // }

//             if (vio_data.state == VIO_STATE_OK) {
//                 // update our last time_alignments, need to convert back down to nanoseconds
//                 last_time_alignment_ns = current_state->_calib_dt_CAMtoIMU->value()(0) * 1e9;

//                 // this is what it should look like, will be shifting over to the full qvio type eventually
//                 // vio_data.imu_cam_time_shift_s = current_state->_calib_dt_CAMtoIMU->value()(0);
//                 // last_time_alignment_ns = vio_data.imu_cam_time_shift_s * 1e9;
//             }

//             // try to do the overlay drawing ourselves - or maybe not...i just want it to be grayscale, so might just use opencv for now
//             // slam features get a plus
//             // for (unsigned int i = 0; i < features_msckf.size(); i++){
//             //     cCharacter_draw_plus(draw_image.data, draw_image.cols, draw_image.rows, features_msckf[i](0), features_msckf[i](1));
//             // }
//             // for (unsigned int i = 0; i < features_slam.size(); i++){
//             //     cCharacter_draw_plus_inverted(draw_image.data, draw_image.cols, draw_image.rows, features_slam[i](0), features_slam[i](1));
//             // }

//             // camera_image_metadata_t meta__;
//             // meta__.timestamp_ns = _apps_time_monotonic_ns();
//             // meta__.width = draw_image.cols;
//             // meta__.height = draw_image.rows;

//             // // known to be rgb image regardless of input
//             // meta__.size_bytes = meta__.width * meta__.height;
//             // meta__.stride = meta__.width;
//             // meta__.format = IMAGE_FORMAT_RAW8;

//             // pipe_server_write_camera_frame(OVERLAY_OUTPUT_CH, meta__, (char*)draw_image.data);
//             // LEAVING OUT ANGULAR VELOCITY FOR NOW
//             // Add the angular velocities as the IMU data with estimated biases subtracted
//             // vio_data.imu_angular_vel[0] = closest_imu_packet.last_angular_velocity_data[0] - static_cast<float>(current_state->_imu->bias_g()(0, 0));
//             // vio_data.imu_angular_vel[1] = closest_imu_packet.last_angular_velocity_data[1] - static_cast<float>(current_state->_imu->bias_g()(0, 1));
//             // vio_data.imu_angular_vel[2] = closest_imu_packet.last_angular_velocity_data[2] - static_cast<float>(current_state->_imu->bias_g()(0, 2));

//             pipe_server_write(SIMPLE_OUTPUT_CH, (char*)&vio_data, sizeof(vio_data_t));
//         }
//     }
// }






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

    /// IMU NOISE OPTIONS ///
    vio_manager_options.imu_noises.sigma_w =  imu_sigma_w;
    vio_manager_options.imu_noises.sigma_wb = imu_sigma_wb;
    vio_manager_options.imu_noises.sigma_a =  imu_sigma_a;
    vio_manager_options.imu_noises.sigma_ab = imu_sigma_ab;

    vio_manager_options.imu_noises.sigma_w_2 =  std::pow(vio_manager_options.imu_noises.sigma_w, 2);
    vio_manager_options.imu_noises.sigma_wb_2 = std::pow(vio_manager_options.imu_noises.sigma_wb, 2);
    vio_manager_options.imu_noises.sigma_a_2 =  std::pow(vio_manager_options.imu_noises.sigma_a, 2);
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
    vio_manager_options.num_pts = num_pts;
    vio_manager_options.fast_threshold = fast_threshold;
    vio_manager_options.grid_x = grid_x;
    vio_manager_options.grid_y = grid_y;
    vio_manager_options.min_px_dist = min_px_dist;
    vio_manager_options.knn_ratio = knn_ratio;
    vio_manager_options.downsample_cameras = downsample_cams;
    vio_manager_options.use_multi_threading = use_multithreading;

    /// FEATURE INITIALIZER OPTIONS ///
    vio_manager_options.featinit_options.triangulate_1d = false;
    vio_manager_options.featinit_options.refine_features = true;
    vio_manager_options.featinit_options.max_runs = 3;
    vio_manager_options.featinit_options.init_lamda = 1e-3;
    vio_manager_options.featinit_options.max_lamda = 1e6;
    vio_manager_options.featinit_options.min_dx = 1e-6;
    vio_manager_options.featinit_options.min_dcost = 1e-6;
    vio_manager_options.featinit_options.lam_mult = 10;
    vio_manager_options.featinit_options.min_dist = 0.1;
    vio_manager_options.featinit_options.max_dist = 10;
    vio_manager_options.featinit_options.max_baseline = 10;
    vio_manager_options.featinit_options.max_cond_number = 1000;

    /// CAMERA INTRINSICS + EXTRINSICS ///
    int actual_index = 0;
    std::shared_ptr<ov_core::CamBase> cam_calib_intrinsic;
    for (size_t i = 0; i < MAX_CAMERAS; i++) {
        // Set the camera type
        if (cam_info_vec[i].enable) {
            // ov uses camequi model to represent fisheye cameras, and the radtan model for standard lenses
            if (cam_info_vec[i].is_fisheye) {
                cam_calib_intrinsic = std::make_shared<ov_core::CamEqui>(cam_info_vec[i].cam_calib_intrinsic(8, 0), cam_info_vec[i].cam_calib_intrinsic(9, 0));
            }
            else {
                cam_calib_intrinsic = std::make_shared<ov_core::CamRadtan>(cam_info_vec[i].cam_calib_intrinsic(8, 0), cam_info_vec[i].cam_calib_intrinsic(9, 0));
            }
            // The camera intrinsics
            cam_calib_intrinsic->set_value(cam_info_vec[i].cam_calib_intrinsic);
            vio_manager_options.camera_intrinsics[actual_index] = cam_calib_intrinsic;
            // fprintf(stderr, "vio manager intrinsics size: %d\n", vio_manager_options.camera_intrinsics.size());
            vio_manager_options.camera_extrinsics[actual_index] = cam_info_vec[i].cam_wrt_imu;
            last_cam_timestamps_ns.push_back(0);

            actual_index++;
            // break;
        }
    }
    vio_manager_options.init_options.camera_intrinsics = vio_manager_options.camera_intrinsics;
    vio_manager_options.init_options.camera_extrinsics = vio_manager_options.camera_extrinsics;
    // this needs to be set in both locations, for some reason the updaters address both in different stages
    vio_manager_options.init_options.num_cameras =  actual_index;
    vio_manager_options.state_options.num_cameras = actual_index;
    return vio_manager_options;
}

// #define CONTROL_COMMANDS (RESET_VIO_HARD)
// // control listens for reset commands
// static void _control_pipe_cb(__attribute__((unused)) int ch, char* string, int bytes, __attribute__((unused)) void* context) {
//     // remove the trailing newline from echo
//     if (bytes > 1 && string[bytes - 1] == '\n') {
//         string[bytes - 1] = 0;
//     }

//     if (strncmp(string, RESET_VIO_HARD, strlen(RESET_VIO_HARD)) == 0) {
//         printf("Client requested hard reset\n");
//         std::lock_guard<std::mutex> lg(cam_mutex);
//         std::lock_guard<std::mutex> lg2(vio_manager_mutex);
//         ov_msckf::VioManagerOptions vio_manager_options = generate_open_vins_manager_options();
//         // HARD RESET
//         vio_manager.reset(new ov_msckf::VioManager(vio_manager_options));
//         return;
//     }

//     printf("WARNING: Server received unknown command through the control pipe!\n");
//     printf("got %d bytes. Command is: %s\n", bytes, string);
//     return;
// }

static void _cam_disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context) {
    fprintf(stderr, "WARNING: disconnected from camera server, resetting VIO\n");

    // if (en_eval) {
    //     ov_eval_data error_packet;
    //     error_packet.done = 1;
    //     error_packet.magic_number = VIO_MAGIC_NUMBER;
    //     error_packet.timestamp_ns = _apps_time_monotonic_ns();
    //     pipe_server_write(SIMPLE_EVAL_CH, (char*)&error_packet, sizeof(ov_eval_data));
    // }

    global_error_codes |= ERROR_CODE_CAM_MISSING;
    // last_cam_timestamp_ns = 0;
    is_cam_connected = 0;
    // std::lock_guard<std::mutex> lg(cam_mutex);
    std::lock_guard<std::mutex> lg2(vio_manager_mutex);
    ov_msckf::VioManagerOptions vio_manager_options = generate_open_vins_manager_options();
    // HARD RESET
    vio_manager.reset(new ov_msckf::VioManager(vio_manager_options));
    return;
}

static void _imu_disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context) {
    fprintf(stderr, "WARNING: disconnected from imu server, resetting VIO\n");

    // if (en_eval) {
    //     ov_eval_data error_packet;
    //     error_packet.done = 1;
    //     error_packet.magic_number = VIO_MAGIC_NUMBER;
    //     error_packet.timestamp_ns = _apps_time_monotonic_ns();
    //     pipe_server_write(SIMPLE_EVAL_CH, (char*)&error_packet, sizeof(ov_eval_data));
    // }

    global_error_codes |= ERROR_CODE_IMU_MISSING;
    last_imu_timestamp_ns = 0;
    is_imu_connected = 0;
    // std::lock_guard<std::mutex> lg(cam_mutex);
    std::lock_guard<std::mutex> lg2(vio_manager_mutex);
    ov_msckf::VioManagerOptions vio_manager_options = generate_open_vins_manager_options();
    // HARD RESET
    vio_manager.reset(new ov_msckf::VioManager(vio_manager_options));
    return;
}

static int create_server_pipes(void) {
       int flags = SERVER_FLAG_EN_CONTROL_PIPE;

    // init extended pipe
    pipe_info_t info1 = { \
        QVIO_EXTENDED_NAME,			// name
        QVIO_EXTENDED_LOCATION,		// location
        "qvio_data_t",				// type
        PROCESS_NAME,				// server_name
        QVIO_RECOMMENDED_PIPE_SIZE,	// size_bytes
        0							// server_pid
    };

    if(pipe_server_create(EXTENDED_CH, info1, flags)){
        _quit(-1);
    }

    // add in optional fields to the info JSON file
    cJSON* json = pipe_server_get_info_json_ptr(EXTENDED_CH);
    cJSON_AddStringToObject(json, "imu", imu_name);
    // cJSON_AddStringToObject(json, "cam", cam_name);
    pipe_server_update_info(EXTENDED_CH);
    pipe_server_set_control_cb(EXTENDED_CH, _control_pipe_cb, NULL);
    pipe_server_set_available_control_commands(EXTENDED_CH, QVIO_CONTROL_COMMANDS);


    // init simple pipe
    pipe_info_t info2 = { \
        QVIO_SIMPLE_NAME,			// name
        QVIO_SIMPLE_LOCATION,		// location
        "vio_data_t",				// type
        PROCESS_NAME,				// server_name
        VIO_RECOMMENDED_PIPE_SIZE,	// size_bytes
        0							// server_pid
    };

    if(pipe_server_create(SIMPLE_CH, info2, flags)){
        _quit(-1);
    }

    // add in optional fields to the info JSON file
    json = pipe_server_get_info_json_ptr(SIMPLE_CH);
    cJSON_AddStringToObject(json, "imu", imu_name);
    // cJSON_AddStringToObject(json, "cam", cam_name);
    pipe_server_update_info(SIMPLE_CH);
    pipe_server_set_control_cb(SIMPLE_CH, _control_pipe_cb, NULL);
    pipe_server_set_available_control_commands(SIMPLE_CH, QVIO_CONTROL_COMMANDS);


    // init overlay pipe
    pipe_info_t info3 = { \
        QVIO_OVERLAY_NAME,			// name
        QVIO_OVERLAY_LOCATION,		// location
        "camera_image_metadata_t",		// type
        PROCESS_NAME,				// server_name
        CAM_PIPE_SIZE,				// size_bytes
        0							// server_pid
    };

    if(pipe_server_create(OVERLAY_CH, info3, flags)){
        _quit(-1);
    }

    pipe_server_set_connect_cb(OVERLAY_CH, _overlay_connect_cb, NULL);
    pipe_server_set_disconnect_cb(OVERLAY_CH, _overlay_disconnect_cb, NULL);
    pipe_server_set_control_cb(OVERLAY_CH, _control_pipe_cb, NULL);
    pipe_server_set_available_control_commands(OVERLAY_CH, QVIO_CONTROL_COMMANDS);

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
            if (pipe_client_open(channel_number, full_pipe, PROCESS_NAME, flags, 0) != 0) {
                fprintf(stderr, "ERROR: FAILED TO OPEN %s\n", full_pipe);
                return -1;
            }

            // if stereo, the right camera is going to use id+1 for its images, so we need to make space for that
            if (cam_info_vec[i].is_stereo) {
                actual_index += 1;
                // also need to skip the NEXT camera in the vector, since it should be the same topic, just a pair
                i += 1;
            }
            actual_index += 1;  // regular bump
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

    // rc_ov_ringbuf_alloc(&imu_buf, 1000);

    // pipe_set_process_priority(90);

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
    // publish_vio_data_thread = std::thread(_publish_vio_data);

    // run until start/stop module catches a signal and changes main_running to 0
    while (main_running) usleep(5000000);

    // Shutdown Nicely
    _quit(0);

    return 0;
}
