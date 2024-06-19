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
#include <state/StateHelper.h>
#include <utils/quat_ops.h>
#include <atomic>
#include <modal_json.h>
#include <stdio.h>
#include <voxl_common_config.h>
//#include <Eigen/Dense>
//#include <vft_interface.h>  // TODO need this in common SDK

#include <c_library_v2/common/mavlink.h> // include before modal_pipe !!

#include <iostream>
#include <thread>
#include <random>

#include "cCharacter.h"

#include "config_file.h"
#include "cam_config_file.h"
#include "quality.h"
#include "common.h"
#include "img_ringbuffer.h"
#include "rc_transform.h"
#include "cpu_monitor_interface.h"

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

#define OV_STATUS_NAME "ov_status"
#define OV_STATUS_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR OV_STATUS_NAME "/"

// server channels
#define EXTENDED_CH 0
#define SIMPLE_CH 1
#define OVERLAY_CH 2
#define OVSTATUS_CH 3

// client channels and config
#define IMU_CH 0
#define FEATURE_CH 5
#define FEAT_OVERLAY_CH 2
#define BARO_CH 3
#define CPU_CH 4

#define CAMERA_CH_START_OFFSET 1
#define IMU_PIPE_MIN_PIPE_SIZE (4 * 640 * 640)  // give ourselves huge buffers
#define CAM_PIPE_SIZE (15 * 1280 * 800)         // give ourselves huge buffers
#define PROCESS_NAME "voxl-open-vins-server"
#define FEATURE_NAME "tracked_feats"
#define FEATURE_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR FEATURE_NAME "/"
#define FEATURE_OVERLAY_NAME "feat_overlay"
#define FEATURE_OVERLAY_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR FEATURE_OVERLAY_NAME "/"

#define CPU_PIPE_DIR MODAL_PIPE_DEFAULT_BASE_DIR  "cpu_monitor/"

// after 300ms with no response, the health monitor thread assumes mvVISLAM
// has locked up while processing a frame and starts sending messages indicating
// a stall has occured with a failed state
#define STALL_TIMEOUT_NS 300000000

// auto restart if the system fails to init after 5 seconds
#define INIT_FAILURE_TIMEOUT_NS 5000000000

// force a timeout if I'm in init for a long time, like 5 seconds
#define INIT_TOO_LONG_TIMEOUT_NS_ARMED 25000000000
#define INIT_TOO_LONG_TIMEOUT_NS_IDLE 5000000000
                                                                                
// init after  image frames recevoied at start up
#define INIT_TIMEOUT_FRAMES 30

// not really sure if this will be needed
#define SILENT_STD(x)                             \
    {                                             \
        FILE* silentfd = fopen("/dev/null", "w"); \
        int savedstdoutfd = dup(STDOUT_FILENO);   \
        fflush(stdout);                           \
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
static int set_tracker_mode = 0;
static int offline = 0;
static bool bypass_reset_checks = false;
static bool is_thermal = false;
static bool pause_qmin = false;
static uint16_t resume_processing = 0;
static pthread_t health_thread;
static pthread_t overlay_thread;

static double last_feature_time;
static double last_imu_time;
static int64_t last_cam_time;
static int64_t time_avg;
static int avg = 0;
static uint8_t thresh_level = 127;
static volatile int perf_limit = 1;
static int start_idx = 0;
static int cameras_used = 0;
static int camera_pipe_channels[10];
static int gravity_vector_direction = -1;

static volatile int every_other = 0;
static volatile double T_uncertainty = 0;
static volatile double R_uncertainty = 0;
static volatile float alt_z = 0.0;
static volatile bool changed_motion_state = false;
static volatile bool imu_moved = false;
static volatile int throttle_state = 0;

static volatile bool is_armed = false;
static volatile bool is_resetting = false;
static volatile bool use_takeoff_cam  = true;
static volatile bool has_idle_images  = false;
static volatile bool ext_blind_take_off_force = false;

// TODO TBD static volatile int takeoff_horizon = 0;

// these are the last timestamps that have completely passed into mvvislam
// cam time is middle of frame. Also last pose to have been received from mvvislam
static volatile int64_t last_imu_timestamp_ns = 0;
static volatile int64_t last_feat_timestamp_ns = 0;
std::mutex feat_ts_mutex;
// this will need to be populated per camera before we start referencing it
static volatile int64_t last_real_pose_timestamp_ns = 0;
static volatile int64_t last_sent_timestamp_ns = 0;
static volatile int64_t last_cep_timestamp_ns = 0;

// state of imu and camera connections
static volatile int is_imu_connected = 0;
static volatile int is_cam_connected = 0;
static volatile int is_init = 0;
static volatile int init_pass_frames = 0;
static volatile int gravity_axis = 2; // 0=x,1=y,2=z

// flag set to 1 on reset to indicate to the blowup detector not to check
// for blowups until after VIO actually initializes
static volatile int hard_reset_blowup_flag = 1;

// flag and time when a reset is requested to indicate to the init failure
// detection how long VIO has been trying to init
static volatile int init_failure_detector_reset_flag = 0;
static volatile int64_t time_of_last_reset = 0;
static volatile int blowup_detector_flag = 0;
static volatile int64_t time_of_first_okay = 0;

// openvins functions do not seem to be thread safe, protect all calls to the VioManager
std::unique_ptr<ov_msckf::VioManager> vio_manager;
static ov_msckf::VioManagerOptions vio_manager_options;
std::mutex vio_manager_mutex;
static Eigen::Matrix3d flu_ned_correction_mat = Eigen::Matrix3d::Identity();
static Eigen::Matrix<double, 4, 1> default_level_horizon (0,0,0,1); 

std::mutex cam_lock_mutex;
std::mutex publish_lock_mutex;
std::mutex imu_lock_mutex;
std::mutex baro_lock_mutex;

static volatile int is_initialized = 0;
static volatile int idler_limit = -1;
static int blank_counter = 0;
static int fade_counter = 0;
static int64_t last_time_alignment_ns = 0;
static int32_t last_frame_frame_id = 0;
static int64_t last_frame_timestamp_ns = 0;

// Thermal stuff
cv::Mat last_img8;
cv::Mat last_img;

// idle image stuff
cv::Mat idle_image1;
cv::Mat idle_image2;


// overlay image stream stuff
#define DRAW_BONUS_ROWS_TOP 64
#define DRAW_BONUS_ROWS_BOT 64
static cv::Mat draw_frame;
static camera_image_metadata_t draw_meta;

static cv::Mat cached_overlay;
static camera_image_metadata_t cached_meta;
static bool not_displaying = true;

std::mutex overlay_mutex;

static cv::Mat world_correction;
std::deque<ov_core::CameraData> camera_queue;
std::mutex camera_queue_mtx;

// set any error codes here for publishing in the data structure
static uint32_t global_error_codes = 0;

std::vector<camera_info> cam_info_vec;
static char imu_name[CHAR_BUF_SIZE] = "imu";
static char baro_name[CHAR_BUF_SIZE] = "mavlink_onboard";
static char vft_name[CHAR_BUF_SIZE] = "tracking_feats";

static size_t num_cams = 0;
static int max_width = 0;
static int max_height = 0;

static uint8_t update_slots = 0;   // 8 cameras max

// function prototypes
static void _publish(double vio_dt);
static void _publish_default(double pose_timestamp);
static bool show_extra_points_on_overlay = true;
static bool pause_cam_states[6];

// FUNCTIONS PROTOTSYPES
static int _hard_reset(bool is_locked);
static int connect_client_pipes(void);
void reset_states();

/// State initializer
std::shared_ptr<ov_init::InertialInitializer> vins_initializer;

static int8_t verbosity_level
{ 
	static_cast<uint8_t>(ov_core::Printer::PrintLevel::SILENT) 
};

static ext_vio_data_t d;  // complete "extended" vio MPA packet
static vio_data_t s;      // simplified vio packet

static ext_vio_data_t d_copy;  // complete "extended" vio MPA packet

typedef struct vft_feature_set{
    uint8_t cam_id;
    float timestamp;
	std::vector<ov_core::ExtFeature> features;  // max support of 99 features
} __vft_feature_set;

std::deque<vft_feature_set> feature_queue;
std::mutex feature_queue_mtx;
vft_feature_set feat_set;
vft_feature_set feat_set_zero;
std::vector<vft_feature> features;
static uint8_t slot_image_pixels[MAX_IMAGE_SIZE];   // image pixels

// structs for inspection
vft_feature_packet* feature_packet_from_vft;
vft_feature* features_from_vft;

typedef struct _bucket
{
	int64_t timestamp;
	ext_vio_data_t d;
	int used_pts;
	int not_used_pts;
	int q;
	double cep;
	double rerr;
} display_bucket_t;
static display_bucket_t display_snapshot;

typedef struct _klt_feature_t
{
	uint32_t id;               ///< unique ID for feature point
	int32_t cam_id; ///< ID of camera which the point was seen from (typically first)
	float pix_loc[2];           ///< pixel location in the last frame
//    float depth_error_stddev;   ///< depth error in meters
} klt_feature_t;

typedef struct _ov_status_t
{
	uint32_t magic_number; ///< Unique 32-bit number used to signal the beginning of a VIO packet while parsing a data stream.
	int64_t timestamp_ns; ///< Timestamp in clock_monotonic system time of the provided pose.
	int32_t quality;
	float p_dop;
	float r_dop;
	int num_features;
	klt_feature_t features[VIO_MAX_REPORTED_FEATURES];
} ov_status_t;
static ov_status_t ov_status;

std::string log_path = "";
int is_color = 0; // assume mono
int is_single = 0;  // assume alway dual

static RingBuffer *img_ringbuf = new RingBuffer(5);

std::atomic<bool> thread_update_running;
std::atomic<bool> image_update_running;
std::atomic<bool> overlay_update_running;
boost::posix_time::ptime pT1, pT2, cT1, cT2, zeroTimeOut;
static double ref_zero_alt = -9999.0;
static double baro_alt = 0.;
static double d_baro = 0.;

/////////////////////////
/////////////////////////
#define sampleFreq      1000.0f                 // sample frequency in Hz
#define twoKpDef        (2.0f * 0.01f)   // 2 * proportional gain
#define twoKiDef        (2.0f * 0.0f)   // 2 * integral gain
volatile float twoKp = twoKpDef;                   // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;                       // 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay,
		float az, float *phi, float *theta, float *psi);
float invSqrt(float x);
static void dead_reckon_position(double dt, float ax, float ay, float az,
		float gx, float gy, float gz);
static float _odometry_imu[3] =
{ 0 };
static float _vel_imu[3] =
{ 0 };
static float _accel_bias[3] =
{ 0 };
static float _gyro_bias[3] =
{ 0 };
static float _last_gyro[3] =
{ 0 };
static float _last_vel[3] =
{ 0 }; 
static float _last_odometry_imu[3] =
{ 0 };
/////////////////////////
/////////////////////////

// printed if some invalid argument was given
static void _print_usage(void)
{
	printf(
			"\n\
This is meant to run in the background as a systemd service, but can be\n\
run manually with the following debug options\n\
\n\
-c, --config   [0|1|2|3]  only parse/create the config file and exit, don't run. Options are: \n\t\t\t0=dual mono, 1=dual color,\n\t\t\t2=single mono, 3=single color, 4=qvio replacement\n\
-m, --mode   [0|1|2] type of feature tracking, 0=leave as-is (default), 1=change to internal tracker, 2=change to external tracker\n\
-d, --debug                 enable debug prints\n\
-h, --help                  print this help message\n\
-i, --timing-imu            show timing prints for imu processing\n\
-l, --log_path              enable using calibration files from a log instead of defaults\n\
                              this is only necessary if cal files have changed or if the\n\
                              log is from a different setup.\n\
                              log path shoulf be absolute to the start of the dir\n\
                              i.e. /data/voxl-logger/log0001 (with or w/out last /)\n\
-o, --offline               set is_armed to always be true and prevent updates\n\
-p, --position              print position and rotation\n\
-s, --debug-crash           print lots of numbers to track down location of crashes\n\
-t, --timing-cam            enable timing prints for camera processing\n\
-x, --thresh   <0-255)>        Threshold Value\n\
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

static bool _parse_opts(int argc, char *argv[])
{
	
	static struct option long_options[] =
	{
	{ "config", required_argument, 0, 'c' },
	{ "mode", required_argument, 0, 'm' },
	{ "debug", no_argument, 0, 'd' },
	{ "help", no_argument, 0, 'h' },
	{ "timing-imu", no_argument, 0, 'i' },
	{ "log_path", required_argument, 0, 'l' },
	{ "offline", no_argument, 0, 'o' },
	{ "position", no_argument, 0, 'p' },
	{ "thresh", no_argument, 0, 'x' },
	{ "timing-cam", no_argument, 0, 't' },
	{ "verbosity", required_argument, 0, 'v' },
	{ 0, 0, 0, 0 } };

	std::string tmp_str = "";

	// set default before we do anything else
//	ov_core::Printer::setPrintLevel(ov_core::Printer::PrintLevel::ALL);
	ov_core::Printer::setPrintLevel(ov_core::Printer::PrintLevel::SILENT);

	while (1)
	{
		int option_index = 0;
		int c = getopt_long(argc, argv, "cdhil:mpotvx:", long_options,
				&option_index);

		// Detect the end of the options.
		if (c == -1)
		{
			break;
		}
		int camera_setup = 0;
		switch (c)
		{
		case 0:
			// for long args without short equivalent that just set a flag nothing left to do so just break.
			if (long_options[option_index].flag != 0)
				break;
			break;

		case 'm':
			set_tracker_mode = static_cast<uint8_t>(std::atoi(optarg));
			break;
			
			
		case 'c':
			camera_setup = static_cast<uint8_t>(std::atoi(optarg));
			en_config_only = 1;
			
			if (camera_setup == 0)
			{
				is_color = 0;
				is_single = 0;
			}
			else if (camera_setup == 1)
			{
				is_color = 1;
				is_single = 0;
			}
			else if (camera_setup == 2)
			{
				is_color = 0;
				is_single = 1;
			}
			else if (camera_setup == 3)
			{
				is_color = 1;
				is_single = 1;
			}
			else if (camera_setup == 4)   // QVIO replacement
			{
				is_color = -1;
				is_single = -1;
			}

						
			printf("[INFO] color camera? %s\n", is_color ? "true" : "false");
			printf("[INFO] single camera? %s\n", is_single ? "true" : "false");
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
			if (log_path.back() == '/')
				log_path.pop_back();
			printf("Using log path of: %s\n", log_path.data());
			break;

		case 'i':
			printf("Enabling debug imu timing mode\n");
			en_debug_timing_imu = 1;
			break;

		case 'p':
			en_debug_pos = 1;
			break;

		case 'o':
            offline = 1;
			is_armed = true;
			break;

		case 'x':
			thresh_level = static_cast<uint8_t>(std::atoi(optarg));
			printf("===> Threshold: %d\n", thresh_level);
			break;

		case 't':
			en_debug_timing_cam = 1;
			break;

		case 'v':
			verbosity_level = static_cast<uint8_t>(std::atoi(optarg));
			switch (verbosity_level)
			{
			case static_cast<uint8_t>(ov_core::Printer::PrintLevel::ALL):
				ov_core::Printer::setPrintLevel(
						ov_core::Printer::PrintLevel::ALL);
				break;
			case static_cast<uint8_t>(ov_core::Printer::PrintLevel::DEBUG):
				ov_core::Printer::setPrintLevel(
						ov_core::Printer::PrintLevel::DEBUG);
				break;
			case static_cast<uint8_t>(ov_core::Printer::PrintLevel::INFO):
				ov_core::Printer::setPrintLevel(
						ov_core::Printer::PrintLevel::INFO);
				break;
			case static_cast<uint8_t>(ov_core::Printer::PrintLevel::WARNING):
				ov_core::Printer::setPrintLevel(
						ov_core::Printer::PrintLevel::WARNING);
				break;
			case static_cast<uint8_t>(ov_core::Printer::PrintLevel::ERROR):
				ov_core::Printer::setPrintLevel(
						ov_core::Printer::PrintLevel::ERROR);
				break;
			case static_cast<uint8_t>(ov_core::Printer::PrintLevel::SILENT):
				ov_core::Printer::setPrintLevel(
						ov_core::Printer::PrintLevel::SILENT);
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

static void _nanosleep(uint64_t ns)
{
	struct timespec req, rem;
	req.tv_sec = ns * 1e-09;
	req.tv_nsec = ns % 1000000000;
	// loop untill nanosleep sets an error or finishes successfully
	errno = 0;  // reset errno to avoid false detection
	while (nanosleep(&req, &rem) && errno == EINTR)
	{
		req.tv_sec = rem.tv_sec;
		req.tv_nsec = rem.tv_nsec;
	}
	return;
}

// call this instead of return when it's time to exit to cleans up everything
static void _quit(int ret)
{
	
	printf("QUIT REQUESTED\n");
	
	// Close all the open pipe connections
	pipe_server_close_all();
	pipe_client_close_all();

	// Remove this process ID file
	remove_pid_file(PROCESS_NAME);

	if (img_ringbuf)
		delete img_ringbuf;
	
	if (ret == 0)
		printf("Exiting Cleanly\n");
	else
		printf("error code: %d\n", ret);
	exit(ret);
	return;
}

static cv::Mat rot2euler(const Eigen::Matrix3d &rm)
{
	cv::Mat euler(3, 1, CV_64F);

	double roll = atan2(rm(2, 1), rm(2, 2));
	double pitch = asin(-rm(2, 0));
	double yaw = atan2(rm(1, 0), rm(0, 0));

	if (fabs(pitch - M_PI_2) < 0.001)
	{
		roll = 0.0;
		pitch = atan2(rm(1, 2), rm(0, 2));
	}
	else if (fabs(pitch + M_PI_2) < 0.001)
	{
		roll = 0.0;
		pitch = atan2(-rm(1, 2), -rm(0, 2));
	}

	euler.at<double>(0) = roll;         // roll
	euler.at<double>(1) = pitch;     // pitch
	euler.at<double>(2) = yaw;      // yaw

	return euler;
}

static void quaternionToRPY(double qx, double qy, double qz, double qw,
		double &roll, double &pitch, double &yaw)
{
	// Convert quaternion to Euler angles (roll-pitch-yaw)
	double sinr_cosp = 2.0 * (qw * qx + qy * qz);
	double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
	roll = std::atan2(sinr_cosp, cosr_cosp);

	double sinp = 2.0 * (qw * qy - qz * qx);
	if (std::abs(sinp) >= 1.0)
		pitch = std::copysign(M_PI / 2.0, sinp); // use 90 degrees if out of range
	else
		pitch = std::asin(sinp);

	double siny_cosp = 2.0 * (qw * qz + qx * qy);
	double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
	yaw = std::atan2(siny_cosp, cosy_cosp);
}

void reset_states()
{
	 last_feature_time = 0;
	 last_imu_time = 0;
	 last_cam_time = 0;
	 perf_limit = 1;
	 start_idx = 0;
	 every_other = 0;
	 T_uncertainty = 0;
	 R_uncertainty = 0;
	alt_z = 0.0;
	changed_motion_state = false;
	imu_moved = false;
	is_armed = false;
	use_takeoff_cam  = true;
	has_idle_images  = false;
	last_imu_timestamp_ns = 0;
	last_feat_timestamp_ns = 0;
	last_real_pose_timestamp_ns = 0;
	last_sent_timestamp_ns = 0;
	last_cep_timestamp_ns = 0;
	is_init = 0;

	init_failure_detector_reset_flag = 0;
	time_of_last_reset = 0;

	is_initialized = 0;
	blank_counter = 0;
	fade_counter = 0;
	idler_limit = -1;
	last_time_alignment_ns = 0;
	last_frame_frame_id = 0;
	last_frame_timestamp_ns = 0;
}


static int _hard_reset_(bool fast_reset)
{
	
	if (is_resetting)
		return 0;
	
	// Lock all external data handlers not related to VINS core instance
	imu_lock_mutex.lock();
	cam_lock_mutex.lock();
	baro_lock_mutex.lock();
	publish_lock_mutex.lock();
	
	// update my state
	is_resetting = true;

	printf("[INFO] restarting managers\n");
	
	camera_queue.clear();

	if (!en_force_init && is_armed && imu_moved)
	{
		vio_manager->zero_state();
		reset_states();
	}
	else
	{
		// stop it if it's running
		if (is_initialized)
		{
			is_initialized = 0;
			vio_manager.reset();  // delete VINS core instance!
		}

		camera_queue.clear();

		// now start again
		double oe_init_window_time =
				vio_manager_options.init_options.init_window_time;
		double oe_init_imu_thresh =
				vio_manager_options.init_options.init_imu_thresh;
		double oe_zupt_max_vel = vio_manager_options.zupt_max_velocity;
		double oe_init_max_disparity= vio_manager_options.init_options.init_max_disparity;
		double oe_zupt_max_disparity= vio_manager_options.zupt_max_disparity;
		bool oe_init_dyn_use = vio_manager_options.init_options.init_dyn_use;

		if (is_armed)
		{
			vio_manager_options.init_options.init_imu_thresh = 2.5;
		}
		
		if (fast_reset)
		{
			printf("FAST RESET requested at Alt.\n");
			vio_manager_options.init_options.init_window_time = 0.25;
	//		vio_manager_options.init_options.init_max_disparity += 1;
			vio_manager_options.zupt_max_disparity += 0.6;
	//		vio_manager_options.zupt_max_velocity = 1.0;
	//		vio_manager_options.init_options.init_dyn_use = true;
		}
		
		reset_states();
		
		usleep(50);
		
		vio_manager.reset(new ov_msckf::VioManager(vio_manager_options));
		if (vio_manager == NULL)
		{
			fprintf(stderr, "Error creating vio_manager object\n");
			_quit(-1);
		}
		vio_manager_options.zupt_max_velocity = oe_zupt_max_vel;
		vio_manager_options.init_options.init_window_time = oe_init_window_time;
		vio_manager_options.init_options.init_imu_thresh = oe_init_imu_thresh;
		vio_manager_options.init_options.init_max_disparity = oe_init_max_disparity;
		vio_manager_options.zupt_max_disparity = oe_zupt_max_disparity;
		vio_manager_options.init_options.init_dyn_use = oe_init_dyn_use;
	}

	printf("[INFO] unlocking managers\n");

	usleep(50);

	imu_lock_mutex.unlock();
	cam_lock_mutex.unlock();
	baro_lock_mutex.unlock();
	publish_lock_mutex.unlock();

	is_resetting = false;
	
	return 1;
}

#ifdef BUILD_QRB5165
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

// control listens for reset commands
static void _control_pipe_cb(__attribute__((unused)) int ch, char *string,
		int bytes, __attribute__((unused)) void *context)
{
	// remove the trailing newline from echo
	if (bytes > 1 && string[bytes - 1] == '\n')
	{
		string[bytes - 1] = 0;
	}

	if (strncmp(string, RESET_VIO_HARD, strlen(RESET_VIO_HARD)) == 0)
	{
		resume_processing = 0;
		fprintf(stderr, "[ERROR] Client requested hard reset\n");
		init_failure_detector_reset_flag = 3;  // close and restart the object
		return;
	}
	else if (strncmp(string, RESET_VIO_SOFT, strlen(RESET_VIO_SOFT)) == 0)
	{
		resume_processing = 0;
		fprintf(stderr,"[ERROR] Client requested reset\n");
		init_failure_detector_reset_flag = 2;  // close and restart the object
		return;
	}
	else if (strncmp(string, "forcearm", strlen("forcearm")) == 0)
	{
		is_armed = 1;
		fprintf(stderr,"[ERROR] Client requested forced arm\n");
		return;
	}
	else if (strncmp(string, "forceq", strlen("forceq")) == 0)
	{
		pause_qmin = true;
		fprintf(stderr,"[WARN] Pausing VIO data (QMIN = 0)\n");
		return;
	}
	else if (strncmp(string, "normq", strlen("normq")) == 0)
	{
		pause_qmin = false;
		fprintf(stderr,"[WARN] Playing VIO data (QMIN = 0)\n");
		return;
	}
	else if (strncmp(string, "flip1", strlen("flip1")) == 0)
	{
		pause_cam_states[0] = !pause_cam_states[0];
        printf("Client requested cam 0 stream to be %s\n", pause_cam_states[0] ? "paused" : "running");
		return;
	}
	else if (strncmp(string, "flip2", strlen("flip2")) == 0)
	{
		pause_cam_states[1] = !pause_cam_states[1];
        printf("Client requested cam 1 stream to be %s\n", pause_cam_states[1] ? "paused" : "running");
		return;
	}
	
	printf(
			"WARNING: Server received unknown command through the control pipe!\n");
	printf("got %d bytes. Command is: %s\n", bytes, string);
	return;
}

// print when a new client connects to us
static void _overlay_connect_cb(__attribute__((unused)) int ch,
		__attribute__((unused)) int client_id, char *client_name,
		__attribute__((unused)) void *context)
{
	printf("client \"%s\" connected to overlay\n", client_name);
	return;
}

// print when a client disconnects from us
static void _overlay_disconnect_cb(__attribute__((unused)) int ch,
		__attribute__((unused)) int client_id, char *client_name,
		__attribute__((unused)) void *context)
{
	printf("client \"%s\" disconnected from overlay\n", client_name);
	return;
}


static void _new_feat_data_default_handler(__attribute__((unused)) int ch, char* data, int bytes,
                       __attribute__((unused)) void* context) 
{
	if (validate_vft_data(ch, data, bytes, &feature_packet_from_vft, &features_from_vft)) {
	    printf("ERROR parsing vft data from pipe...\n");
		return;
	}
	
	double set_time = (double) feature_packet_from_vft->timestamp_ns[0] * 1e-9;
			
	static double last_set_time = set_time;
	static int n_total_features = 0;

	double dt_f = set_time-last_set_time;
	last_set_time = set_time;

	if (dt_f == 0)
		return;

	if (is_resetting)
		return;
	
	// testing logic
	int test_offset = 0;
	if (pause_cam_states[0] || pause_cam_states[1])
	{
		if (pause_cam_states[0])
		{
			test_offset = feature_packet_from_vft->num_feats[0];
			feature_packet_from_vft->num_feats[0] = 0;
		}
		if (pause_cam_states[1])
		{
			feature_packet_from_vft->num_feats[1] = 0;
		}
	}

	
	if (!vio_manager->initialized() || imu_moved || en_vio_always_on)
	{
		n_total_features = 0;
		
		if (use_takeoff_cam && (is_armed || en_vio_always_on) && (double) alt_z < takeoff_threshold) // turn off, we are in the air
		{
			if (!en_vio_always_on)
				printf(
					"Detected Takeoff, going to multicamera VINS normal operations\n");
			use_takeoff_cam = false;
		}			
		
		int n_cams = feature_packet_from_vft->n_cams;
				
		for (int i = 0; i < n_cams; i++) {
			if (use_takeoff_cam && i == takeoff_cam)
			{
				n_total_features = 0;
				n_total_features = feature_packet_from_vft->num_feats[i];
				break;
			}
			else
			{
				n_total_features += feature_packet_from_vft->num_feats[i];
			}
		}
  
		int cam_num = 0;
		int ctn = 0;
		bool has_feats = 0;
		
		feat_set.features.resize(n_total_features);
		feat_set_zero.features.resize(n_total_features);
		
		for (int i = 0; i < n_total_features; i++)
		{
			if (use_takeoff_cam && cam_num != takeoff_cam)
			{
				continue;
			}

			if (ctn >= feature_packet_from_vft->num_feats[cam_num])  
			{
				ctn = 0;
				cam_num++;
			}

			double cam_time =  (double) feature_packet_from_vft->timestamp_ns[cam_num] * 1e-9;

			int tmp_offset = test_offset + i;
			feat_set.timestamp = cam_time;							
			feat_set.cam_id = cam_num;
			feat_set.features[i].cam_id = cam_num;
			feat_set.features[i].id = features_from_vft[tmp_offset].id;
			feat_set.features[i].u = features_from_vft[tmp_offset].x;
			feat_set.features[i].v = features_from_vft[tmp_offset].y;  // TODO subtract from height?
			memcpy(feat_set.features[i].descriptor, features_from_vft[tmp_offset].descriptor, 32);
			
			feat_set_zero.timestamp = cam_time;							
			feat_set_zero.cam_id = cam_num;
			feat_set_zero.features[i].cam_id = cam_num;
			feat_set_zero.features[i].id = features_from_vft[tmp_offset].id;
			feat_set_zero.features[i].u = features_from_vft[tmp_offset].x;
			feat_set_zero.features[i].v = features_from_vft[tmp_offset].y;  // TODO subtract from height?
			memcpy(feat_set_zero.features[i].descriptor, features_from_vft[tmp_offset].descriptor, 32);
			
			ctn++;
		}        

		std::lock_guard < std::mutex > lck(feature_queue_mtx);			

		feature_queue.push_back(feat_set);		
	}
	else
	{
		for (int i = 0; i < (int) feat_set_zero.features.size(); i++)
		{
			double cam_time =  (double) feature_packet_from_vft->timestamp_ns[0] * 1e-9;

			feat_set.timestamp = cam_time;							
			feat_set.cam_id = feat_set_zero.cam_id;
			feat_set.features[i].cam_id = feat_set_zero.features[i].cam_id;
			feat_set.features[i].id = feat_set_zero.features[i].id;
			feat_set.features[i].u = feat_set_zero.features[i].u;
			feat_set.features[i].v = feat_set_zero.features[i].v;  
			memcpy(feat_set.features[i].descriptor, feat_set_zero.features[i].descriptor, 32);
		}
		std::lock_guard < std::mutex > lck(feature_queue_mtx);	

		feature_queue.push_back(feat_set);
	}

	is_cam_connected = true;
}

// helper callback for cams we are using in the system
static void _cam_helper_cb(__attribute__((unused)) int ch,
		camera_image_metadata_t meta, char *frame, void *context)
{

	static int idler_ctn = 0;
	
	if (!en_ext_feature_tracker)
		is_cam_connected = true;

	// camera working, reset errors
	global_error_codes &= ~ERROR_CODE_CAM_MISSING;

	if (!is_imu_connected || !main_running || image_update_running
			|| resume_processing >= 1)
	{
		if (image_update_running)
		{
			is_initialized = true;
		}
		else
		{
			is_initialized = false;
		}

 		memcpy(&d.v, &s, sizeof(vio_data_t));
		// send to both pipes
		pipe_server_write(EXTENDED_CH, (char*) &d, sizeof(ext_vio_data_t));
		pipe_server_write(SIMPLE_CH, (char*) &s, sizeof(vio_data_t));
		
		// TODO replace with histogram analysis
		if (resume_processing++ > 12)
		{
			if(en_debug)
				printf("[THERMAL] Resume processing done\n");
			
			resume_processing = 0;
		}
		return;
	}

	if (!en_ext_feature_tracker)
	{
		if (idler_limit > 0 && idler_ctn++ < idler_limit)
		{
			last_cam_time = _apps_time_monotonic_ns();
			return;
		}
		else
		{
			idler_ctn = 0;
		}
	}
	
	std::lock_guard < std::mutex > cam_lg(cam_lock_mutex);
	
	// try to lock to bigger cores if we can
#ifdef BUILD_QRB5165
    _check_and_set_affinity();
#endif

	camera_mode *cm = (camera_mode*) context;

	img_ringbuf_packet *curr_message = new img_ringbuf_packet;
	curr_message->camid = ch;
	curr_message->metadata = meta;
	
	if (en_debug_timing_cam)
	{		
		if (cameras_used > 1)
		{
			static double cams_ts[2];
			static int ctn_cams_ts[2];
			
			printf("Cam %d ts: %f", ch, curr_message->metadata.timestamp_ns * 1e-09);

			if ((ctn_cams_ts[0] + ctn_cams_ts[1]) >= 2)  // todo add 3rd cam
			{
				printf(" Ch1 vs Ch2 delta: %f (ms)\n", (cams_ts[1] - cams_ts[0])* 1e3);
				ctn_cams_ts[0] = 0;
				ctn_cams_ts[1] = 0;
				cams_ts[0] = 0;
				cams_ts[1] = 0;
			}
			else
				printf("\n");
			
			ctn_cams_ts[ch-1] = 1;
			cams_ts[ch-1] = curr_message->metadata.timestamp_ns * 1e-09;
		}

	}
	
	try
	{
		if (meta.format == IMAGE_FORMAT_RAW8)
		{
			if (cameras_used == 1)
			{
				memcpy(curr_message->image_pixels, (uint8_t*) frame,
						meta.size_bytes);			

				img_ringbuf->insert_data(curr_message);
				
				cv::Mat internal_img(meta.height, meta.width, CV_8UC1,
					(uint8_t*) curr_message->image_pixels);
	
				ov_core::CameraData message;
				message.timestamp = curr_message->metadata.timestamp_ns * 1e-09;
				message.sensor_ids.push_back(0);

				//TODO handling exception!
				static cv::Mat zero_image = imread("/data/modalai/ov/zero_ref.jpg", cv::IMREAD_GRAYSCALE);
				static cv::Mat ignore_mask(internal_img.rows,
						internal_img.cols, CV_8UC1, cv::Scalar(255));
				static cv::Mat use_mask(internal_img.rows,
						internal_img.cols, CV_8UC1, cv::Scalar(0));
	
				if (zero_image.rows != internal_img.rows)
					resize(zero_image, zero_image, cv::Size(internal_img.cols, internal_img.rows), cv::INTER_NEAREST);
	
				if (!vio_manager->initialized())
				{
					if (en_debug)
						printf("Initializing\n");
					
					message.images.push_back(zero_image);
				}
				else
				{
					if (!imu_moved)
					{
						if (en_debug)
							printf("Idling\n");
	
						if (!has_idle_images)
						{
							if (en_debug)
								printf("Capture new idle images\n");											
							idle_image1 = internal_img.clone();
							has_idle_images = true;
						}
						
						message.images.push_back(idle_image1);
					}
					else
					{
						message.images.push_back(internal_img.clone());
					}
				}
				
				if (use_takeoff_cam)
				{
					message.masks.push_back(use_mask);
					if (is_armed && (double) alt_z < takeoff_threshold) // turn off, we are in the air
					{
						printf(
								"Detected Takeoff, going to VINS normal operations\n");
						use_takeoff_cam = false;
	
						// if we're disarmed, and running load  throttling, turn off and go fullt throttle
						idler_limit = -1;
					}
				}
				else
				{
					message.masks.push_back(use_mask);
				}
	
				std::lock_guard < std::mutex > lck(camera_queue_mtx);
				camera_queue.push_back(message);
				std::sort(camera_queue.begin(), camera_queue.end());
			}
			else
			{
				
				// combine and sim multi camera				
				if (curr_message->camid == 1)
				{
					memcpy(slot_image_pixels, (uint8_t*) frame,
							meta.size_bytes);			
				    update_slots |= (1 << 7);
				}
				else if (curr_message->camid == 2)
				{
					memcpy(slot_image_pixels + meta.size_bytes, 
							(uint8_t*) frame,
							meta.size_bytes);			
				    update_slots |= (1 << 6);
				}

				if (((update_slots >> 7) & 1) && ((update_slots >> 6) & 1))
				{
					curr_message->camid = ch;
					curr_message->metadata = meta;
					memcpy(curr_message->image_pixels, (uint8_t*) slot_image_pixels,
							meta.size_bytes*cameras_used);			
					update_slots = 0;
					img_ringbuf->insert_data(curr_message);
				}

			}
			is_thermal = false;
			
		}
		else if (meta.format == IMAGE_FORMAT_RAW16)
		{
			cv::Mat internal_img(meta.height, meta.width, CV_16UC1,
					(uint8_t*) frame);

			/////////////////////// THERMAL ONLY ////////////////////////
			cv::Mat img_8;

			//need to convert 16 bit image to 8 bit
			double min_pixel, max_pixel;
			cv::minMaxLoc(internal_img, &min_pixel, &max_pixel);

			bool use_last = false;  // TODO make a param

			if (use_last && !last_img.empty())
			{
				//also consider the previous image when determining the conversion
				double last_min_pixel, last_max_pixel;
				cv::minMaxLoc(last_img, &last_min_pixel, &last_max_pixel);

				max_pixel = std::max(max_pixel, last_max_pixel);
				min_pixel = std::min(min_pixel, last_min_pixel);
			}

			double min_temp_delta = 400.0;
			double pixel_diff = std::max(max_pixel - min_pixel,
					(double) min_temp_delta);

			double alpha_convert = 255.0 / pixel_diff;
			double beta_convert = -min_pixel * alpha_convert;

			internal_img.convertTo(img_8, CV_8UC1, alpha_convert, beta_convert);
			internal_img = img_8;

			threshold(internal_img, internal_img, thresh_level, 0,
					cv::THRESH_TOZERO);

//        cv::Mat mask = cv::Mat(internal_img.size(), CV_8UC1);
//        threshold(internal_img, mask, thresh_level, 255, cv::THRESH_BINARY);

			if (use_last && !last_img.empty())
			{
				last_img.convertTo(last_img8, CV_8UC1, alpha_convert,
						beta_convert);
			}

			/////////////////////// THERMAL ONLY ////////////////////////
			memcpy(curr_message->image_pixels, (uint8_t*) internal_img.data,
					internal_img.total() * internal_img.elemSize());

			img_ringbuf->insert_data(curr_message);
			
			ov_core::CameraData message;
			message.timestamp = curr_message->metadata.timestamp_ns * 1e-09;
			message.sensor_ids.push_back(0);

			message.images.push_back(internal_img.clone());
			message.masks.push_back(
					cv::Mat::zeros(internal_img.rows, internal_img.cols,
							CV_8UC1));
///		message.masks.push_back(mask.clone());

			std::lock_guard < std::mutex > lck(camera_queue_mtx);
			camera_queue.push_back(message);

			is_thermal = true;

			std::sort(camera_queue.begin(), camera_queue.end());
		}
		else if (meta.format == IMAGE_FORMAT_STEREO_RAW8)
		{
			if (*cm == STEREO)
			{
				// NOTE: the stereo image comes in as a concatenated single image via vertical tiled (width = width, height = img_ctn * height)
//			printf("STEREO CH %d %f\n", ch, curr_message->metadata.timestamp_ns * 1e-09);
				memcpy(curr_message->image_pixels, (uint8_t*) frame,
						meta.size_bytes);

				img_ringbuf->insert_data(curr_message);
				
				cv::Mat internal_img_1(meta.height, meta.width, CV_8UC1,
						(uint8_t*) curr_message->image_pixels);
				cv::Mat internal_img_2(meta.height, meta.width, CV_8UC1,
						(uint8_t*) curr_message->image_pixels
								+ meta.size_bytes / 2);

				ov_core::CameraData message;
				message.timestamp = curr_message->metadata.timestamp_ns * 1e-09;
				message.sensor_ids.push_back(0);
				message.sensor_ids.push_back(1);

				// TODO dynamic apeture
#ifdef DYN_AP				
				static cv::Mat idle_mask1(internal_img_1.rows,
						internal_img_1.cols, CV_8UC1, cv::Scalar(0));
//				static int rectX1 = (internal_img_1.cols - 640) / 2;
//				static int rectY1 = (internal_img_1.rows - 620) / 2;
//				cv::rectangle(idle_mask1, cv::Rect(rectX1, rectY1, 640, 180), cv::Scalar(255), -1);  	
				cv::rectangle(idle_mask1, cv::Rect(426, 266, internal_img_1.cols-426, internal_img_1.rows-266), cv::Scalar(255), -1);  	
				
				static cv::Mat idle_mask2(internal_img_2.rows,
						internal_img_2.cols, CV_8UC1, cv::Scalar(0));
//				static int rectX2 = (internal_img_2.cols -640 ) / 2;
//				static int rectY2 = (internal_img_2.rows - 620) / 2;
//				cv::rectangle(idle_mask2, cv::Rect(rectX2, rectY2, 640, 180), cv::Scalar(255), -1);  	
				cv::rectangle(idle_mask2, cv::Rect(426, 266, internal_img_2.cols-426, internal_img_2.rows-266), cv::Scalar(255), -1);  	
#endif
				//TODO handling exception!
				static cv::Mat zero_image = imread("/data/modalai/ov/zero_ref.jpg", cv::IMREAD_GRAYSCALE);
				if (zero_image.rows != internal_img_1.rows)
					resize(zero_image, zero_image, cv::Size(internal_img_1.cols, internal_img_1.rows), cv::INTER_NEAREST);

				static cv::Mat ignore_mask1(internal_img_1.rows,
						internal_img_1.cols, CV_8UC1, cv::Scalar(255));
				static cv::Mat ignore_mask2(internal_img_2.rows,
						internal_img_2.cols, CV_8UC1, cv::Scalar(255));
				
				static cv::Mat use_mask1(internal_img_1.rows,
						internal_img_1.cols, CV_8UC1, cv::Scalar(0));
				static cv::Mat use_mask2(internal_img_2.rows,
						internal_img_2.cols, CV_8UC1, cv::Scalar(0));

				if (!vio_manager->initialized())
				{
					if (en_debug)
						printf("Initializing\n");
					
					message.images.push_back(zero_image);
					message.images.push_back(zero_image);
				}
				else 
				{
					if (!imu_moved)
					{
						if (en_debug)
							printf("Idling\n");

						if (!has_idle_images)
						{
							if (en_debug)
								printf("Capture new idle images\n");											
							idle_image1 = internal_img_1.clone();
							idle_image2 = internal_img_2.clone();													
							has_idle_images = true;
						}
						
						message.images.push_back(idle_image1);
						message.images.push_back(idle_image2);	
					}
					else
					{
						// use real time raw imagery
						message.images.push_back(internal_img_1.clone());
						message.images.push_back(internal_img_2.clone());
					}
				}
				
				if (use_takeoff_cam)
				{
					if (takeoff_cam == 0)
					{
						if (en_debug)
							printf("in takeoff %f\n", (double)alt_z);
						message.masks.push_back(use_mask1);
						message.masks.push_back(ignore_mask1);
					}
					else if (takeoff_cam == 1)
					{
						if (en_debug)
							printf("in takeoff %f\n", (double)alt_z);
						message.masks.push_back(ignore_mask2);
						message.masks.push_back(use_mask2);
					}
					else
					{
						message.masks.push_back(use_mask1);
						message.masks.push_back(use_mask2);
					}

					if ((is_armed || en_vio_always_on) && (double) alt_z < takeoff_threshold) // turn off, we are in the air
					{
						printf(
								"Detected Takeoff, going to multicamera VINS normal operations\n");
						use_takeoff_cam = false;

						// if we're disarmed, and running load  throttling, turn off and go fullt throttle
						idler_limit = -1;
					}
				}
				else
				{
					message.masks.push_back(use_mask1);
					message.masks.push_back(use_mask2);
				}

				std::lock_guard < std::mutex > lck(camera_queue_mtx);
				camera_queue.push_back(message);
				std::sort(camera_queue.begin(), camera_queue.end());

				is_thermal = false;

			}
			else //MONO
			{
				memcpy(curr_message->image_pixels, (uint8_t*) frame,
						meta.size_bytes);

				img_ringbuf->insert_data(curr_message);
				
				int offset = 0;
				if (single_cam_in_use == 1)
				{
					offset = meta.size_bytes / 2;
				}				
				cv::Mat internal_img(meta.height, meta.width, CV_8UC1,
						(uint8_t*) curr_message->image_pixels + offset);
				
				ov_core::CameraData message;
				message.timestamp = curr_message->metadata.timestamp_ns * 1e-09;
				message.sensor_ids.push_back(0);

				//TODO handling exception!
				static cv::Mat zero_image = imread("/data/modalai/ov/zero_ref.jpg", cv::IMREAD_GRAYSCALE);
				if (zero_image.rows != internal_img.rows)
					resize(zero_image, zero_image, cv::Size(internal_img.cols, internal_img.rows), cv::INTER_NEAREST);

				static cv::Mat ignore_mask(internal_img.rows,
						internal_img.cols, CV_8UC1, cv::Scalar(255));
				
				static cv::Mat use_mask(internal_img.rows,
						internal_img.cols, CV_8UC1, cv::Scalar(0));

				if (!vio_manager->initialized())
				{
					if (en_debug)
						printf("Initializing\n");
					
					message.images.push_back(zero_image);
				}
				else
				{
					if (!imu_moved)
					{
						if (en_debug)
							printf("Idling\n");

						if (!has_idle_images)
						{
							if (en_debug)
								printf("Capture new idle images\n");											
							idle_image1 = internal_img.clone();
							has_idle_images = true;
						}
						
						message.images.push_back(idle_image1);
					}
					else
					{
						message.images.push_back(internal_img.clone());
					}
				}
				
				if (use_takeoff_cam)
				{
					message.masks.push_back(use_mask);
					if ((is_armed || en_vio_always_on) && (double) alt_z < takeoff_threshold) // turn off, we are in the air
					{
						printf(
								"Detected Takeoff, going to multicamera VINS normal operations\n");
						use_takeoff_cam = false;

						// if we're disarmed, and running load  throttling, turn off and go fullt throttle
						idler_limit = -1;
					}
				}
				else
				{
					message.masks.push_back(use_mask);
				}

				std::lock_guard < std::mutex > lck(camera_queue_mtx);
				camera_queue.push_back(message);
				std::sort(camera_queue.begin(), camera_queue.end());

				is_thermal = false;
				
			}
		}

		last_cam_time = _apps_time_monotonic_ns();

	} catch (const std::out_of_range &e)
	{
		fprintf(stderr, "CAM Process error!\n");
	}

	thread_update_running = false;

	delete curr_message;
}

// imu callback registered to the imu server
static void _new_imu_data_default_handler(__attribute__((unused)) int ch,
		char *data, int bytes, __attribute__((unused)) void *context)
{	
	double cam_imu_time_delta = (double) (_apps_time_monotonic_ns()
			- last_cam_time) * 1e-9;

	if (is_initialized && idler_limit <= 0)
	{
		if (!bypass_reset_checks && cam_imu_time_delta > 0.15)
		{
			printf("-----------> UNSTABLE [THERMAL?]  <----------------\n");
			bypass_reset_checks = true;
		}
		else if (bypass_reset_checks && cam_imu_time_delta < 0.1)
		{
			printf("-----------> STABLE [THERMAL?]  <----------------\n");
			bypass_reset_checks = false;
			resume_processing = 1;
		}
	}

	std::lock_guard < std::mutex > imu_lg(imu_lock_mutex);

//	if (en_debug)
//			printf("_new_imu_data_default_handler entrance: %f\n",  _apps_time_monotonic_ns() * 1e-9);

	int n_packets;
	imu_data_t *data_array = pipe_validate_imu_data_t(data, bytes, &n_packets);
	if (data_array == NULL)
	{
		printf("ERROR: no IMU data\n");
		return;
	}

	if (n_packets <= 0)
	{
		printf("ERROR: no IMU n_packets extracted from imu data\n");
		return;
	}

	// flag that imu data is active, remove the error, skip data if camera is disconnected
	is_imu_connected = 1;
	global_error_codes &= ~ERROR_CODE_IMU_MISSING;

	if (!is_cam_connected || !main_running)
	{
		memcpy(&d.v, &s, sizeof(vio_data_t));
		is_initialized = false;
		// send to both pipes
		pipe_server_write(EXTENDED_CH, (char*) &d, sizeof(ext_vio_data_t));
		pipe_server_write(SIMPLE_CH, (char*) &s, sizeof(vio_data_t));

		return;
	}

	try
	{

		is_init = vio_manager->initialized();

		// time this in debug mode
		int64_t process_time;

		static double last_accel_mag = 0, last_gyro_mag = 0;
		static int last_perf_limit = 0;

		// TODO current IMU is setup to batch send imu values. This has a conflict with OV's internal processing system
		// by pausing caluclation while the publishing loop runs with new timestamps.
		// So on average use every other packet.
		if (!vio_manager->is_moving())
		{
			perf_limit = 5; //6
		}
		else
		{

			if (n_packets > 100)
				perf_limit = 10;  //3
			else if (n_packets > 15)
				perf_limit = 3;
			else
				perf_limit = 1;

			if (!changed_motion_state)
			{
				// ARM CHECK?
				fprintf(stderr,
						"[WARN] Motion detected, going to full processing\n");
				changed_motion_state = true;
			}
		}

		if (en_debug && perf_limit != last_perf_limit)
		{
			printf("WARNING: IMU processing rate changed: %d\n", perf_limit);
		}

		last_perf_limit = perf_limit;

		for (int i = 0; i < n_packets; i += perf_limit)
		{

//			auto rT0_1 = boost::posix_time::microsec_clock::local_time();

			int64_t  dt_long =  (data_array[i].timestamp_ns-last_imu_timestamp_ns);
			double dt = (double) dt_long * 1e-09;

			// check if we somehow got an out-of-order imu sample and reject it
			if ((int64_t) data_array[i].timestamp_ns <= last_imu_timestamp_ns)
			{
				fprintf(stderr,
						"WARNING out-of-order imu %fms before previous\n", dt);
				continue;
			}
			else
			{

				// Create the data struct that we will use for ingesting data into the vio manager
				ov_core::ImuData vio_manager_data;
				vio_manager_data.timestamp = data_array[i].timestamp_ns * 1e-09; // (seconds)

				Eigen::Matrix<double, 3, 1> t_wm;
				Eigen::Matrix<double, 3, 1> t_am;

				t_wm(2, 0) = data_array[i].gyro_rad[2];
				t_wm(1, 0) = data_array[i].gyro_rad[1];
				t_wm(0, 0) = data_array[i].gyro_rad[0];
				t_am(2, 0) = data_array[i].accl_ms2[2];
				t_am(1, 0) = data_array[i].accl_ms2[1];
				t_am(0, 0) = data_array[i].accl_ms2[0];
				
				if (!imu_moved && dt <0.01)
				{
					static int ramp = 0;
					static double prev_accel = 0;
					double accel_mag = t_am.norm();
					static double base_accel = accel_mag;

					if (ramp++ < 256)
					{
						base_accel = accel_mag;
					}
					else
					{
						double d_accel = accel_mag-base_accel;
						d_accel  = (0.8 * d_accel) - (0.2 * prev_accel);
						prev_accel = d_accel; 		
						if (d_accel > init_imu_thresh_accel && (throttle_state > 1 || en_vio_always_on))
						{

							if (ramp < 258)
								printf("[INFO] IMU has moved, JERK detected\n");
							
							
							imu_moved = true;
							vio_manager->force_moving();
						}
					}

					if (is_initialized)
					{
					    for (int k = 0; k<3; ++k) 
					    {
					        if (fabs(data_array[i].accl_ms2[k]) > fabs(data_array[i].accl_ms2[gravity_axis])) 
					        {
					        	gravity_axis = k;
					        }
					    }
					}
					else
					{
						gravity_axis = 2;
					}
				}
								
				// NED to FLU systems as per VINS.
				t_wm = flu_ned_correction_mat * t_wm;
				t_am = flu_ned_correction_mat * t_am;

				// FLU set data for VINS
				vio_manager_data.wm(0, 0) = t_wm(0, 0); // roll
				vio_manager_data.wm(1, 0) = t_wm(1, 0);  // pitch
				vio_manager_data.wm(2, 0) = t_wm(2, 0);  //yaw
				vio_manager_data.am(0, 0) = t_am(0, 0);  // X axis
				vio_manager_data.am(1, 0) = t_am(1, 0);  // Y axis
				vio_manager_data.am(2, 0) = t_am(2, 0);  // Z axis

				vio_manager->feed_measurement_imu(vio_manager_data);
				
				last_imu_timestamp_ns = data_array[i].timestamp_ns;
				last_imu_time = vio_manager_data.timestamp;

				if (thread_update_running)
					return;

				thread_update_running = true;

				double timestamp_imu_inC = vio_manager_data.timestamp
						- vio_manager->get_state()->_calib_dt_CAMtoIMU->value()(
								0);

				std::thread thread ([&]
				{

					std::lock_guard < std::mutex > lck(camera_queue_mtx);
					std::lock_guard < std::mutex > feat_lck(feature_queue_mtx);
					std::lock_guard < std::mutex > pub_lg(publish_lock_mutex);

					if (en_ext_feature_tracker)
					{
						while (!feature_queue.empty() && feature_queue.at(0).timestamp< timestamp_imu_inC)
						{
							vft_feature_set fst = feature_queue.at(0);
							vio_manager->feed_measurement_feature(fst.timestamp,fst.features);
							_publish_default(last_imu_time);
							feature_queue.pop_front();
						}
					}
					else
					{
						while (!camera_queue.empty() && camera_queue.at(0).timestamp< timestamp_imu_inC)
						{
							vio_manager->feed_measurement_camera(camera_queue.at(0));
							_publish_default(last_imu_time);
							camera_queue.pop_front();
						}						
					}
					thread_update_running = false;
				}
				);

				thread.join();

			}

//			auto rT0_2 = boost::posix_time::microsec_clock::local_time();
//			double time_total = (rT0_2 - rT0_1).total_microseconds() * 1e-6;

		}

	} catch (const std::out_of_range &e)
	{
		fprintf(stderr, "IMU Process error!\n");
	}

}

static double map_double(double x, double in_min, double in_max, double out_min,
		double out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Looks at quality over time
static bool stable_quality(int cur_qual)
{
	static double last_good_qual_ts = _apps_time_monotonic_ns();
	bool is_bad = false;

	if (cur_qual >= 1)
		last_good_qual_ts = _apps_time_monotonic_ns();

	// if quality is less than acceptable for more than 2 sec
	double ts = (_apps_time_monotonic_ns() - last_good_qual_ts) * 1e-9;
	if (ts > auto_reset_max_v_cov_timeout_s)
	{
		printf("[ERROR] quality was bad for a long time!\n");
		is_bad = true;
	}

	return is_bad;
}

// looks at state during initialization to avoid false positive restarts
static bool stable_state(int the_state)
{
	static double last_good_state_ts = _apps_time_monotonic_ns();
	static bool is_ready = false;

	if (is_ready)
		return is_ready;

	double ts = (_apps_time_monotonic_ns() - last_good_state_ts) * 1e-9;

	// if quality is less than acceptable for more than 2 sec
	if (the_state != VIO_STATE_OK)
	{
		last_good_state_ts = _apps_time_monotonic_ns();
	}
	else if (ts > 3)
	{
		is_ready = true;
	}

	return is_ready;
}

// Looks at quality over time
static bool stable_features(int cur_feats)
{
	static bool wait_for_features = true;
	
	if (wait_for_features)
	{
		if (cur_feats > 3)
		{
			wait_for_features = false;
		}
		return false;
	}

	static double last_good_feat_ts = _apps_time_monotonic_ns();
	bool is_bad = false;
	// if quality is less than acceptable for more than 2 sec

	int min_good_feat_thresh = 1;
	
	if (ext_blind_take_off_force)
	{
		min_good_feat_thresh = 12;
	}	
	
	// TODO use auto_reset_min_features!!!	
	if (cur_feats > min_good_feat_thresh)
		last_good_feat_ts = _apps_time_monotonic_ns();

	double ts = (_apps_time_monotonic_ns() - last_good_feat_ts) * 1e-9;

	if (ts > auto_reset_min_feature_timeout_s)
	{
		printf("ERROR: features were 0 for a long time!\n");
		is_bad = true;
		wait_for_features = true;
		ext_blind_take_off_force = false;
	}

	return is_bad;
}

#define OVERLAY_RES_W_X 320
#define OVERLAY_RES_H_Y 240

static void _post_snapshot()
{
	static int skip_cnt = 0;
	static int target_id = -1;
	static float font_size = 1.0;
	static float border_scale = 1;
	static cv::Mat overlay_cp;
	static char reinit_str[32];
	static char cam_off_str[32];
	static int reinit_time = 0;
	static int reinit_disp_time = 0;

	if (pipe_server_get_num_clients(OVERLAY_CH) > 0) // if someone has subscribed to the overlay, draw it
	{
		not_displaying = false;
		{
			img_ringbuf_packet *curr_imgs = new img_ringbuf_packet;

//			feat_ts_mutex.lock();
			// TODO May not exists, then what
			//double target_time = pose_timestamp*1e9;
			int ret = img_ringbuf->get_data_at_position(0, curr_imgs);
			//int ret = img_ringbuf->get_data_at_time((int64_t)display_snapshot.timestamp, curr_imgs);

			if (ret < 0)
			{
//				feat_ts_mutex.unlock();
				fprintf(stderr,
						"FAILED TO FETCH IMG RINGBUF at time %ld %ld %ld %ld %ld %ld\n",
						display_snapshot.timestamp,
						img_ringbuf->get_timestamp_at_position(0),
						img_ringbuf->get_timestamp_at_position(1),
						img_ringbuf->get_timestamp_at_position(2),
						img_ringbuf->get_timestamp_at_position(3),
						img_ringbuf->get_timestamp_at_position(4));
				return;
			}
//			else if (en_debug)
//			{
//				printf("Time Delta: %f\n", (display_snapshot.timestamp - img_ringbuf->get_timestamp_at_position(0))*1e-9);
//			}

			// now, construct some cv::Mats with our images (we know them to be greyscale)
			std::vector < cv::Mat > img_set;

			if (curr_imgs->metadata.format == IMAGE_FORMAT_STEREO_RAW8)
			{
				font_size = 0.55;
				if (!en_debug_pos)
				{
					cv::Mat img(curr_imgs->metadata.height,
							curr_imgs->metadata.width, CV_8UC1,
							curr_imgs->image_pixels);
					cv::Mat img2(curr_imgs->metadata.height,
							curr_imgs->metadata.width, CV_8UC1,
							curr_imgs->image_pixels
									+ (curr_imgs->metadata.width
											* curr_imgs->metadata.height));
					img_set.push_back(img);
					img_set.push_back(img2);
				}
				else
				{
					cv::Mat img(
							cv::Mat::zeros(curr_imgs->metadata.height,
									curr_imgs->metadata.width, CV_8UC1));
					cv::Mat img2(
							cv::Mat::zeros(curr_imgs->metadata.height,
									curr_imgs->metadata.width, CV_8UC1));
					img_set.push_back(img);
					img_set.push_back(img2);
				}

			}
			else
			{
				font_size = 0.33;
				border_scale = 2;

				if (cameras_used == 1)
				{
					cv::Mat img(curr_imgs->metadata.height,
							curr_imgs->metadata.width, CV_8UC1,
							curr_imgs->image_pixels);
					img_set.push_back(img);
				}
				else
				{					
					cv::Mat img(curr_imgs->metadata.height,
							curr_imgs->metadata.width, CV_8UC1,
							curr_imgs->image_pixels);
					cv::Mat img2(curr_imgs->metadata.height,
							curr_imgs->metadata.width, CV_8UC1,
							curr_imgs->image_pixels
									+ (curr_imgs->metadata.width
											* curr_imgs->metadata.height));
					img_set.push_back(img);
					img_set.push_back(img2);
				}
			}

//			feat_ts_mutex.unlock();

			static int16_t trk_id = -1;
			static std::vector<float> trk_history_x;
			static std::vector<float> trk_history_y;

			if (trk_history_x.size() > 100)
			{
				trk_history_x.clear();
				trk_history_y.clear();
			}

			if (trk_id < 0 && display_snapshot.d.n_total_features > 0)
			{
				trk_id = display_snapshot.d.features[0].id;
				if (en_debug)
					printf("Monitoring id: %d\n", trk_id);
			}

			{
				for (size_t i = 0; i < display_snapshot.d.n_total_features; i++)
				{
					int cam_idx = display_snapshot.d.features[i].cam_id;
				
					// OVERRIDE
					if (single_cam_in_use > 0)
						cam_idx = single_cam_in_use;
					
					if (!en_debug_pos)
					{
						if (display_snapshot.d.features[i].point_quality
								== (int) OV_RE_HIGH)
						{
							cv::drawMarker(
									img_set[cam_idx],
									cv::Point2f(
											display_snapshot.d.features[i].pix_loc[0],
											display_snapshot.d.features[i].pix_loc[1]),
									cv::Scalar(245), cv::MARKER_SQUARE, 8, 2);
						}
						// slam landmark
						else if (display_snapshot.d.features[i].point_quality
								== (int) OV_HIGH)
						{
							cv::drawMarker(
									img_set[cam_idx],
									cv::Point2f(
											display_snapshot.d.features[i].pix_loc[0],
											display_snapshot.d.features[i].pix_loc[1]),
									cv::Scalar(255), cv::MARKER_SQUARE, 20, 2);
							
						}
						// tracked feature
						else if (display_snapshot.d.features[i].point_quality
								== (int) OV_MEDIUM)
						{
							cv::drawMarker(
									img_set[cam_idx],
									cv::Point2f(
											display_snapshot.d.features[i].pix_loc[0],
											display_snapshot.d.features[i].pix_loc[1]),
									cv::Scalar(190), cv::MARKER_SQUARE, 8, 2);
						}
						else if (show_extra_points_on_overlay)
						{
							cv::drawMarker(
									img_set[cam_idx],
									cv::Point2f(
											display_snapshot.d.features[i].pix_loc[0],
											display_snapshot.d.features[i].pix_loc[1]),
									cv::Scalar(100), cv::MARKER_SQUARE, 8, 2);
						}
					}
					else
					{
						if (trk_id >= 0
								&& display_snapshot.d.features[i].id
										== (uint16_t) trk_id)
						{
							trk_history_x.push_back(
									display_snapshot.d.features[i].pix_loc[0]);
							trk_history_y.push_back(
									display_snapshot.d.features[i].pix_loc[1]);

							for (size_t z = 0; z < trk_history_x.size(); z++)
							{
								cv::drawMarker(
										img_set[cam_idx],
										cv::Point2f(trk_history_x[z],
												trk_history_y[z]),
										cv::Scalar(255), cv::MARKER_DIAMOND, 24,
										2);
							}
						}
					}
				}
				
//				for (size_t i = 0; i < vft_features.size(); i++)
//				{
//					int cam_idx = display_snapshot.d.features[i].cam_id;
//					
//					// OVERRIDE
//					if (single_cam_in_use > 0)
//						cam_idx = single_cam_in_use;
//					
//					cv::drawMarker(
//							img_set[cam_idx],
//							cv::Point2f(
//									vft_features[i].x,
//									vft_features[i].y),
//							cv::Scalar(200), cv::MARKER_DIAMOND, 10, 2);
//				}
				
			}

			cv::resize(img_set[0], img_set[0],
					cv::Size(OVERLAY_RES_W_X, OVERLAY_RES_H_Y));

			if (img_set.size() == 1)
			{
				overlay_cp = img_set[0];
			}
			else
			{
				cv::resize(img_set[1], img_set[1],
						cv::Size(OVERLAY_RES_W_X, OVERLAY_RES_H_Y));
				cv::hconcat(img_set[0], img_set[1], overlay_cp);
			}

			if (overlay_cp.cols <= OVERLAY_RES_W_X)
			{
				border_scale = 1;
			}

			cv::copyMakeBorder(overlay_cp, overlay_cp,
			DRAW_BONUS_ROWS_TOP / border_scale,
			DRAW_BONUS_ROWS_BOT / border_scale, 0, 0, cv::BORDER_CONSTANT,
					cv::Scalar(0));

			draw_meta = curr_imgs->metadata;
			draw_meta.width = overlay_cp.cols;
			draw_meta.height = overlay_cp.rows;
			draw_meta.size_bytes = draw_meta.width * draw_meta.height;
			draw_meta.format = IMAGE_FORMAT_RAW8;

			char str[256];
			sprintf(str, "XYZ: %6.2lf %6.2lf %6.2lf  CEP: %3.3f  Rerr: %3.2f",
					(double) s.T_imu_wrt_vio[0], (double) s.T_imu_wrt_vio[1],
					(double) s.T_imu_wrt_vio[2], display_snapshot.cep,
					display_snapshot.rerr * 180 / M_PI);
			int baseline = 0;
			cv::Size text_size = cv::getTextSize(str, cv::FONT_HERSHEY_COMPLEX,
					font_size, font_size, &baseline);

			cv::putText(
					overlay_cp, //target image
					str, //text
					cv::Point((overlay_cp.cols - text_size.width) / 2,
							text_size.height * 3 / 2), //top-left position
					cv::FONT_HERSHEY_COMPLEX, font_size,
					cv::Scalar(255, 255, 255), //font color
					font_size, cv::LINE_AA);

			char oos_pts_string[96];

			if (show_extra_points_on_overlay)
				sprintf(oos_pts_string, "#pts: %2d  (%2d)",
						display_snapshot.used_pts,
						display_snapshot.not_used_pts);
			else
				oos_pts_string[0] = 0;

			sprintf(str, "Q: %3d %s  %s  ex(ms): %6.1f Gain: %5d", s.quality,
					(s.state == VIO_STATE_OK ? "OK" : "INIT"), oos_pts_string,
					draw_meta.exposure_ns / 1000000.0, draw_meta.gain);

			cv::putText(
					overlay_cp, //target image
					str, //text
					cv::Point((overlay_cp.cols - text_size.width) / 2,
							overlay_cp.rows - text_size.height / 2 + 2), //top-left position
					cv::FONT_HERSHEY_COMPLEX, font_size,
					cv::Scalar(255, 255, 255), //font color
					font_size, cv::LINE_AA);

			if (init_failure_detector_reset_flag && !is_initialized)
			{
				reinit_disp_time++;
				sprintf(reinit_str, "RE-INITIALIZING [%d] ", reinit_disp_time);
				cv::putText(
						overlay_cp, //target image
						reinit_str, //text
						cv::Point((overlay_cp.cols * 0.1),
								overlay_cp.rows * 0.5), //top-left position
						cv::FONT_HERSHEY_COMPLEX, 1.0,
						cv::Scalar(255, 255, 255), //font color
						1.5, cv::LINE_AA);
			}
			
			if (pause_cam_states[0] || pause_cam_states[1])
			{
				strcpy(cam_off_str, "Cameras OFF:");
				
				if (pause_cam_states[0])
					strcat(cam_off_str, "   1 ");
				if (pause_cam_states[1])
					strcat(cam_off_str, "  2 ");
				
				cv::putText(
						overlay_cp, //target image
						cam_off_str, //text
						cv::Point((overlay_cp.cols * 0.4),
								overlay_cp.rows * 0.94), //top-left position
						cv::FONT_HERSHEY_COMPLEX, 0.3,
						cv::Scalar(255, 255, 255), //font color
						1.5, cv::LINE_AA);
			}

			

			// draw out to pipe
			pipe_server_write_camera_frame(OVERLAY_CH, draw_meta,
					(char*) overlay_cp.data);
			delete curr_imgs;
		}

		not_displaying = true;

	}

}

#ifdef XXX
static void _post_snapshot()
{
	static int skip_cnt = 0;
	static int target_id = -1;
	static float font_size = 1.0;
	static float border_scale = 1;
	static cv::Mat overlay_cp;
	static char reinit_str[32];
	static int reinit_time = 0;
	static int reinit_disp_time = 0;

	if (pipe_server_get_num_clients(OVERLAY_CH) > 0) // if someone has subscribed to the overlay, draw it
	{
		not_displaying = false;
		{
			img_ringbuf_packet *curr_imgs = new img_ringbuf_packet;
			int ret = img_ringbuf->get_data_at_position(0, curr_imgs);
			if (ret < 0)
			{
				fprintf(stderr,
						"FAILED TO FETCH IMG RINGBUF at time %ld %ld %ld %ld %ld %ld\n",
						display_snapshot.timestamp,
						img_ringbuf->get_timestamp_at_position(0),
						img_ringbuf->get_timestamp_at_position(1),
						img_ringbuf->get_timestamp_at_position(2),
						img_ringbuf->get_timestamp_at_position(3),
						img_ringbuf->get_timestamp_at_position(4));
				return;
			}
			
			std::vector < cv::Mat > img_set;

			// now, construct some cv::Mats with our images (we know them to be greyscale)

			if (curr_imgs->metadata.format == IMAGE_FORMAT_STEREO_RAW8)
			{
				font_size = 0.55;
				if (!en_debug_pos)
				{
					cv::Mat img(curr_imgs->metadata.height,
							curr_imgs->metadata.width, CV_8UC1,
							curr_imgs->image_pixels);
					cv::Mat img2(curr_imgs->metadata.height,
							curr_imgs->metadata.width, CV_8UC1,
							curr_imgs->image_pixels
									+ (curr_imgs->metadata.width
											* curr_imgs->metadata.height));
					img_set.push_back(img);
					img_set.push_back(img2);
				}
				else
				{
					cv::Mat img(
							cv::Mat::zeros(curr_imgs->metadata.height,
									curr_imgs->metadata.width, CV_8UC1));
					cv::Mat img2(
							cv::Mat::zeros(curr_imgs->metadata.height,
									curr_imgs->metadata.width, CV_8UC1));
					img_set.push_back(img);
					img_set.push_back(img2);
				}
			}
			else
			{
				font_size = 0.33;
				border_scale = 2;

				if (cameras_used == 1)
				{
					cv::Mat img(curr_imgs->metadata.height,
							curr_imgs->metadata.width, CV_8UC1,
							curr_imgs->image_pixels);
					img_set.push_back(img);
				}
				else
				{					
					cv::Mat img(curr_imgs->metadata.height,
							curr_imgs->metadata.width, CV_8UC1,
							curr_imgs->image_pixels);
					cv::Mat img2(curr_imgs->metadata.height,
							curr_imgs->metadata.width, CV_8UC1,
							curr_imgs->image_pixels
									+ (curr_imgs->metadata.width
											* curr_imgs->metadata.height));
					img_set.push_back(img);
					img_set.push_back(img2);
				}
			}

//			feat_ts_mutex.unlock();

			static int16_t trk_id = -1;
			static std::vector<float> trk_history_x;
			static std::vector<float> trk_history_y;

			if (trk_history_x.size() > 100)
			{
				trk_history_x.clear();
				trk_history_y.clear();
			}

			if (trk_id < 0 && display_snapshot.d.n_total_features > 0)
			{
				trk_id = display_snapshot.d.features[0].id;
				if (en_debug)
					printf("Monitoring id: %d\n", trk_id);
			}

			{
				for (size_t i = 0; i < display_snapshot.d.n_total_features; i++)
				{
					int cam_idx = display_snapshot.d.features[i].cam_id;					
					// OVERRIDE
					if (single_cam_in_use > 0)
						cam_idx = single_cam_in_use;
					
					if (!en_debug_pos)
					{
						if (display_snapshot.d.features[i].point_quality
								== (int) OV_RE_HIGH)
						{
							cv::drawMarker(
									img_set[cam_idx],
									cv::Point2f(
											display_snapshot.d.features[i].pix_loc[0],
											display_snapshot.d.features[i].pix_loc[1]),
									cv::Scalar(245), cv::MARKER_SQUARE, 8, 2);
						}
						// slam landmark
						else if (display_snapshot.d.features[i].point_quality
								== (int) OV_HIGH)
						{
							cv::drawMarker(
									img_set[cam_idx],
									cv::Point2f(
											display_snapshot.d.features[i].pix_loc[0],
											display_snapshot.d.features[i].pix_loc[1]),
									cv::Scalar(255), cv::MARKER_SQUARE, 20, 2);
							
						}
						// tracked feature
						else if (display_snapshot.d.features[i].point_quality
								== (int) OV_MEDIUM)
						{
							cv::drawMarker(
									img_set[cam_idx],
									cv::Point2f(
											display_snapshot.d.features[i].pix_loc[0],
											display_snapshot.d.features[i].pix_loc[1]),
									cv::Scalar(190), cv::MARKER_SQUARE, 8, 2);
						}
						else if (show_extra_points_on_overlay)
						{
							cv::drawMarker(
									img_set[cam_idx],
									cv::Point2f(
											display_snapshot.d.features[i].pix_loc[0],
											display_snapshot.d.features[i].pix_loc[1]),
									cv::Scalar(100), cv::MARKER_SQUARE, 8, 2);
						}
					}
					else
					{
						if (trk_id >= 0
								&& display_snapshot.d.features[i].id
										== (uint16_t) trk_id)
						{
							trk_history_x.push_back(
									display_snapshot.d.features[i].pix_loc[0]);
							trk_history_y.push_back(
									display_snapshot.d.features[i].pix_loc[1]);

							for (size_t z = 0; z < trk_history_x.size(); z++)
							{
								cv::drawMarker(
										img_set[cam_idx],
										cv::Point2f(trk_history_x[z],
												trk_history_y[z]),
										cv::Scalar(255), cv::MARKER_DIAMOND, 20,
										2);
							}
						}
					}
				}
				
//				for (size_t i = 0; i < vft_features.size(); i++)
//				{
//					int cam_idx = display_snapshot.d.features[i].cam_id;
//					
//					// OVERRIDE
//					if (single_cam_in_use > 0)
//						cam_idx = single_cam_in_use;
//					
//					cv::drawMarker(
//							img_set[cam_idx],
//							cv::Point2f(
//									vft_features[i].x,
//									vft_features[i].y),
//							cv::Scalar(200), cv::MARKER_DIAMOND, 10, 2);
//				}
				
			}

			cv::resize(img_set[0], img_set[0],
					cv::Size(OVERLAY_RES_W_X, OVERLAY_RES_H_Y));

			if (img_set.size() == 1)
			{
				overlay_cp = img_set[0];
			}
			else
			{
				cv::resize(img_set[1], img_set[1],
						cv::Size(OVERLAY_RES_W_X, OVERLAY_RES_H_Y));
				cv::hconcat(img_set[0], img_set[1], overlay_cp);
			}

			if (overlay_cp.cols <= OVERLAY_RES_W_X)
			{
				border_scale = 1;
			}

			cv::copyMakeBorder(overlay_cp, overlay_cp,
			DRAW_BONUS_ROWS_TOP / border_scale,
			DRAW_BONUS_ROWS_BOT / border_scale, 0, 0, cv::BORDER_CONSTANT,
					cv::Scalar(0));

			draw_meta = curr_imgs->metadata;
			draw_meta.width = overlay_cp.cols;
			draw_meta.height = overlay_cp.rows;
			draw_meta.size_bytes = draw_meta.width * draw_meta.height;
			draw_meta.format = IMAGE_FORMAT_RAW8;

			char str[256];
			sprintf(str, "XYZ: %6.2lf %6.2lf %6.2lf  CEP: %3.3f  Rerr: %3.2f",
					(double) s.T_imu_wrt_vio[0], (double) s.T_imu_wrt_vio[1],
					(double) s.T_imu_wrt_vio[2], display_snapshot.cep,
					display_snapshot.rerr * 180 / M_PI);
			int baseline = 0;
			cv::Size text_size = cv::getTextSize(str, cv::FONT_HERSHEY_COMPLEX,
					font_size, font_size, &baseline);

			cv::putText(
					overlay_cp, //target image
					str, //text
					cv::Point((overlay_cp.cols - text_size.width) / 2,
							text_size.height * 3 / 2), //top-left position
					cv::FONT_HERSHEY_COMPLEX, font_size,
					cv::Scalar(255, 255, 255), //font color
					font_size, cv::LINE_AA);

			char oos_pts_string[96];

			if (show_extra_points_on_overlay)
				sprintf(oos_pts_string, "#pts: %2d  (%2d)",
						display_snapshot.used_pts,
						display_snapshot.not_used_pts);
			else
				oos_pts_string[0] = 0;

			sprintf(str, "Q: %3d %s  %s  ex(ms): %6.1f Gain: %5d", s.quality,
					(s.state == VIO_STATE_OK ? "OK" : "INIT"), oos_pts_string,
					draw_meta.exposure_ns / 1000000.0, draw_meta.gain);

			cv::putText(
					overlay_cp, //target image
					str, //text
					cv::Point((overlay_cp.cols - text_size.width) / 2,
							overlay_cp.rows - text_size.height / 2 + 2), //top-left position
					cv::FONT_HERSHEY_COMPLEX, font_size,
					cv::Scalar(255, 255, 255), //font color
					font_size, cv::LINE_AA);

			if (init_failure_detector_reset_flag && !is_initialized)
			{
				reinit_disp_time++;
				sprintf(reinit_str, "RE-INITIALIZING [%d] ", reinit_disp_time);
				cv::putText(
						overlay_cp, //target image
						reinit_str, //text
						cv::Point((overlay_cp.cols * 0.1),
								overlay_cp.rows * 0.5), //top-left position
						cv::FONT_HERSHEY_COMPLEX, 1.0,
						cv::Scalar(255, 255, 255), //font color
						1.5, cv::LINE_AA);
			}

			// draw out to pipe
			pipe_server_write_camera_frame(OVERLAY_CH, draw_meta,
					(char*) overlay_cp.data);
			
			
			if (curr_imgs != NULL)
			{
				delete curr_imgs;
			}
			
		}

		not_displaying = true;

	}

}
#endif

// pose data is published from the same thread that does the camera processing
// and pose estimation. That freezes, sometimes for over a second, during blowups
// so this thread exists to keep data coming out during that situation, warning
// consumers that there is an issue.
// this does NOT monitor for blowup criteria, that's done in the camera thread
// as soon as a new pose is calculated. This thread is to warn when that
// camera thread freezes.
static void* _health_thread_func(__attribute__((unused)) void *ctx)
{
	int64_t last_was_running_time = _apps_time_monotonic_ns();

	
	while (main_running)
	{
		// 66Hz
		usleep(33333);  
		int64_t current_time = _apps_time_monotonic_ns();

		if (init_failure_detector_reset_flag)
		{
			uint64_t time_since_reset = current_time - time_of_last_reset;
			if (time_since_reset > INIT_FAILURE_TIMEOUT_NS)
			{
				if (en_debug)
					fprintf(stderr, "[ERROR] Triggered RESET type: %d\n", init_failure_detector_reset_flag);
				
				if (init_failure_detector_reset_flag == 1)
					_hard_reset_(!is_thermal);
				else if (init_failure_detector_reset_flag == 2)
					_hard_reset_(false);
				else if (init_failure_detector_reset_flag == 3)
					_hard_reset_(true);
										
				time_of_last_reset = _apps_time_monotonic_ns();

				init_failure_detector_reset_flag = 0;
			}
		}
		
		if (!vio_manager->initialized())
		{
			uint64_t time_in_init = current_time - last_was_running_time;
			
			if (is_armed)
			{
				if (time_in_init > INIT_TOO_LONG_TIMEOUT_NS_ARMED && !init_failure_detector_reset_flag)
				{
					fprintf(stderr, "[ERROR] IN FLIGHT and In init too long, timeout, retry  RESET %ld\n", time_in_init);
					init_failure_detector_reset_flag = 1;
				}
			}
			else if (time_in_init > INIT_TOO_LONG_TIMEOUT_NS_IDLE && !init_failure_detector_reset_flag) 
			{
				fprintf(stderr, "[ERROR] In init too long, timeout, retry  RESET %ld\n", time_in_init);
				init_failure_detector_reset_flag = 1;
			}
		}
		else
		{
			last_was_running_time = current_time;
		}
		
	}

	return NULL;
}

static void* _overlay_thread_func(__attribute__((unused)) void *ctx)
{
	sched_param sch_params;
	sch_params.sched_priority = 19;
	pthread_setschedparam(pthread_self(), SCHED_OTHER, &sch_params);

	while (main_running)
	{
		double u_freq = (1.0 / publish_frequency) * 1e6;
		usleep(u_freq);  // run about the same speed as the camera
		_post_snapshot();
	}

	return NULL;
}



static void _publish_default(double pose_timestamp)
{
    static Eigen::Matrix3d rot_global_to_imu_frame = Eigen::Matrix3d::Identity();

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

	// simple lib modal pipe standard vio packet
	memset(&ov_status, 0, sizeof(ov_status));
	ov_status.magic_number = VIO_MAGIC_NUMBER;

	// record that we just got a successful pose and point cloud
	last_real_pose_timestamp_ns = static_cast<int64_t>(pose_timestamp * 1e9);

	if (!vio_manager->initialized())
	{
		s.state = VIO_STATE_INITIALIZING;
		memcpy(&d.v, &s, sizeof(vio_data_t));
		is_initialized = false;
		// send to both pipes
		pipe_server_write(EXTENDED_CH, (char*) &d, sizeof(ext_vio_data_t));
		pipe_server_write(SIMPLE_CH, (char*) &s, sizeof(vio_data_t));
		return;
	}
	else
	{
		s.state = VIO_STATE_OK;
		is_initialized = true;
	}
	

#ifdef EXPERIMENTAL // replaces above
	// check if its initialized or not
	if (!en_force_init)
	{
		if (!vio_manager->initialized())
		{
			s.state = VIO_STATE_INITIALIZING;
			memcpy(&d.v, &s, sizeof(vio_data_t));
			is_initialized = false;
			// send to both pipes
			pipe_server_write(EXTENDED_CH, (char*) &d, sizeof(ext_vio_data_t));
			pipe_server_write(SIMPLE_CH, (char*) &s, sizeof(vio_data_t));
			
			return;
		}
		else
		{
			vio_manager_options.init_options.init_dyn_use = true;
			s.state = VIO_STATE_OK;
			is_initialized = true;
		}
	}
	else  // FORCE INITIALIZATION -- only for non-mission, non-autonomy use, for pilot use
	{
		 if (imu_moved)
		 {
			s.state = VIO_STATE_OK;
			is_initialized = true;
		 }
		 else
		 {
				s.state = VIO_STATE_OK;
				memcpy(&d.v, &s, sizeof(vio_data_t));
				is_initialized = false;
				// send to both pipes
				pipe_server_write(EXTENDED_CH, (char*) &d, sizeof(ext_vio_data_t));
				pipe_server_write(SIMPLE_CH, (char*) &s, sizeof(vio_data_t));
				return;
		 } 
	}
#endif
	
	std::shared_ptr < ov_msckf::State > current_state =
			vio_manager->get_state(); // contains a few extra pieces we need

	std::vector < std::shared_ptr < ov_type::Type >> statevars;
	statevars.push_back(current_state->_imu->p());
	statevars.push_back(current_state->_imu->q());
	statevars.push_back(current_state->_imu->v());

	Eigen::Matrix<double, 9, 9> covariance_posori =
			ov_msckf::StateHelper::get_marginal_covariance(current_state,
					statevars);
	// Row-major representation of the 6x6 covariance matrix
	// The orientation parameters use a fixed-axis representation.
	// In order, the parameters are:
	// (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
	// float64[36] covariance

	Eigen::VectorXd cov_varis = covariance_posori.diagonal();

	double T_uncertainty = 0.0;
	T_uncertainty += cov_varis(0, 0) * cov_varis(0, 0);
	T_uncertainty += cov_varis(1, 1) * cov_varis(1, 1);
	T_uncertainty += cov_varis(2, 2) * cov_varis(2, 2);
	T_uncertainty = sqrt(T_uncertainty);

	double R_uncertainty = 0.0;
	R_uncertainty += cov_varis(3, 3) * cov_varis(3, 3);
	R_uncertainty += cov_varis(4, 4) * cov_varis(4, 4);
	R_uncertainty += cov_varis(5, 5) * cov_varis(5, 5);
	R_uncertainty = sqrt(R_uncertainty);

	double V_uncertainty = 0.0;
	V_uncertainty += cov_varis(6, 6) * cov_varis(6, 6);
	V_uncertainty += cov_varis(7, 7) * cov_varis(7, 7);
	V_uncertainty += cov_varis(8, 8) * cov_varis(8, 8);
	V_uncertainty = sqrt(V_uncertainty);

//	if (en_debug)
//		printf("Uncertainty in the robot's pose: xyz: %f R:%f V: %f\n", T_uncertainty, R_uncertainty, V_uncertainty);

#ifdef MONTE
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covariance_posori);
	Eigen::MatrixXd transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
	static std::mt19937 gen{ std::random_device{}() };
	static std::normal_distribution<> dist;
	Eigen::VectorXd uncertain = transform * Eigen::VectorXd{ 6 }.unaryExpr([&](auto x) { return dist(gen); });
	Eigen::Vector3d  T_sigma = uncertain.segment(0,2);
	Eigen::Vector3d  R_sigma = uncertain.segment(3,5);
	T_uncertainty = (T_uncertainty * 0.6) + (0.4 * (sqrt((T_sigma.array() - T_sigma.mean()).square().sum() / (T_sigma.size() - 1))));
	R_uncertainty = (R_uncertainty * 0.6) + (0.4* (sqrt((R_sigma.array() - R_sigma.mean()).square().sum() / (R_sigma.size() - 1))));

	if (std::isnan(T_uncertainty))
	{
		T_uncertainty = 0;
	}

	if (std::isnan(R_uncertainty))
	{
		R_uncertainty = 0;
	}

	printf("Uncertainty in the robot's pose: xyz: %f R:%f\n", T_uncertainty, R_uncertainty);
#endif

//	printf("Get the uncertainty in the robot's pose: (%dx%d) %f\n", uncertain.rows(), uncertain.cols(), std_dev);
//	printf("(%d): ", (int)cov_varis.size());
//	for (int i=0; i<cov_varis.size(); i++)
//	{
//		printf("%f ", cov_varis(i));
//	}
//	printf("\n");

	// get features
	// this function will give us back as much info as available for features in various stages of the overall state
	std::vector < output_feature > curr_pixel_locs;
	int sz = vio_manager->get_pixel_loc_features(curr_pixel_locs);
	
	for (size_t d = 0; d < curr_pixel_locs.size(); d++)
	{
		if (curr_pixel_locs[d].pix_loc[0] > 0.0f
				&& curr_pixel_locs[d].pix_loc[1] > 0.0f)
		{
			if (curr_pixel_locs[d].point_quality == OV_HIGH)
			{
				n_good_points++;
			}
			else if (curr_pixel_locs[d].point_quality == OV_MEDIUM)
			{
				// UNUSED points
				n_oos_points++;
			}
		}
	}

	// sometimes qvio will report covariance as invalid but state is still OKAY
	// this is NOT alright, in this case manually set the state to failed.
	//
	// Rotation
	if (cov_varis(3, 3) < 0.0f || cov_varis(4, 4) < 0.0f
			|| cov_varis(5, 5) < 0.0f)
	{
		fprintf(stderr, "ERROR: diagonal went negative\n");
		s.state = VIO_STATE_FAILED;
	}

	// don't send packets from the past, this can happen when qvio stalls
	// during a reset
	if (static_cast<int64_t>(current_state->_timestamp * 1e9)
			< last_sent_timestamp_ns)
	{
		fprintf(stderr, "WARNING: skipping pose data from the past %f %ld\n",
				current_state->_timestamp * 1e9, last_sent_timestamp_ns);
		return;
	}

	// All checks passed, after this point this function should not return
	// until the end
	last_sent_timestamp_ns = static_cast<int64_t>(current_state->_timestamp
			* 1e9);

	// populate some other data
	s.timestamp_ns = static_cast<int64_t>(current_state->_timestamp * 1e9);
	d.imu_cam_time_shift_s = current_state->_calib_dt_CAMtoIMU->value()(0);
	last_time_alignment_ns = current_state->_calib_dt_CAMtoIMU->value()(0)
			* 1e9;
	s.n_feature_points = n_good_points;
	d.last_cam_frame_id = last_frame_frame_id;
	d.last_cam_timestamp_ns = last_frame_timestamp_ns;

	// gather all the VINS output
	Eigen::Matrix<double, 3, 1> imu_wrt_wio_holder = current_state->_imu->pos();
	Eigen::Matrix<double, 3, 1> vel_imu_wrt_vio_holder = current_state->_imu->vel();
	// Not used anymore!
	//	Eigen::Matrix3d rot_fej = current_state->_imu->Rot_fej();
	Eigen::Matrix<double, 4, 1> ned_q = current_state->_imu->quat_fej();

    if (gravity_axis == 0 && gravity_vector_direction < 0)
    {
            rot_global_to_imu_frame = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
                            * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    }

	//
	// Translation x y z
	//
	imu_wrt_wio_holder = flu_ned_correction_mat * imu_wrt_wio_holder;
	imu_wrt_wio_holder = rot_global_to_imu_frame * imu_wrt_wio_holder;
	Eigen::MatrixXf::Map(s.T_imu_wrt_vio, 3, 1) = imu_wrt_wio_holder.cast<float>();

	//
	// Translation velocity vx vy vz
	//
	vel_imu_wrt_vio_holder = flu_ned_correction_mat * vel_imu_wrt_vio_holder;
	vel_imu_wrt_vio_holder = rot_global_to_imu_frame * vel_imu_wrt_vio_holder;
	Eigen::MatrixXf::Map(s.vel_imu_wrt_vio, 3, 1) = vel_imu_wrt_vio_holder.cast<float>();


	// Rotations

	// TODO make more efficient
	// TODO maybe use body extrinsics to set a flag if VOXL2 is mounted irregularly
	// TODO which is what this is for
	static Eigen::Matrix<double, 4, 1> rot_corrected(default_level_horizon);
   if (!imu_moved && !is_armed)
   {
	   double angle_q = std::acos(default_level_horizon.dot(ned_q));
	   if (std::abs(angle_q * 180.0 / M_PI) > 30.0)
	   {
//		   printf("[INFO] dynamic horizon alignment\n");
		   rot_corrected = ov_core::Inv(ned_q);
	   }
	   else
	   {
//		   printf("[INFO] static IMU horizon alignment\n");
		   rot_corrected = default_level_horizon;
	   }
   }   
   
   // dynamic zeroing horizon as needed
   Eigen::Matrix<double, 4, 1> new_rot = ov_core::quat_multiply(ned_q, rot_corrected);
    
    // Convert ot NED
    new_rot.y() *= -1;
    new_rot.z() *= -1;
    
    Eigen::Matrix3d final_out_ned = ov_core::quat_2_Rot(new_rot);
    
    if (en_debug)
    {
        Eigen::Matrix<double, 3, 1>  rpy =  ov_core::rot2rpy(final_out_ned);
    	printf("[INFO]  %f, %f, %f --- %f, %f, %f\n",
                    rpy(0)/M_PI*180,
                    rpy(1)/M_PI*180,
                    rpy(2)/M_PI*180,
                    imu_wrt_wio_holder(0),
                    imu_wrt_wio_holder(1),
                    imu_wrt_wio_holder(2));
    }
    
	Eigen::MatrixXf::Map(reinterpret_cast<float*>(s.R_imu_to_vio), 3, 3) =
			final_out_ned.cast<float>();

	// camera position here is a bit funky, since open vins outputs imu to cam and we want cam to imu
	Eigen::Matrix3d cam_out = ov_core::quat_2_Rot(
			current_state->_calib_IMUtoCAM[0]->quat()).transpose();

	Eigen::MatrixXf::Map(reinterpret_cast<float*>(s.R_cam_to_imu), 3, 3) =
			cam_out.cast<float>();

	Eigen::MatrixXf::Map(s.T_cam_wrt_imu, 3, 1) = ((ov_core::quat_2_Rot(
			current_state->_calib_IMUtoCAM[0]->quat().transpose())
			* current_state->_calib_IMUtoCAM[0]->pos()) * -1).cast<float>();
	Eigen::MatrixXf::Map(reinterpret_cast<float*>(d.gyro_bias), 3, 1) =
			current_state->_imu->bias_g_fej().cast<float>();
	Eigen::MatrixXf::Map(reinterpret_cast<float*>(d.accl_bias), 3, 1) =
			current_state->_imu->bias_a_fej().cast<float>();

	// pose covariance diagonals, 6 entries
	s.pose_covariance[0] = (float) cov_varis(0, 0);
	s.pose_covariance[6] = (float) cov_varis(1, 1);
	s.pose_covariance[11] = (float) cov_varis(2, 2);
	s.pose_covariance[15] = (float) cov_varis(3, 3);
	s.pose_covariance[18] = (float) cov_varis(4, 4);
	s.pose_covariance[20] = (float) cov_varis(5, 5);

	// velocity covariance diagonals, 3 entries
	s.velocity_covariance[0] = (float) cov_varis(6, 6);
	s.velocity_covariance[6] = (float) cov_varis(7, 7);
	s.velocity_covariance[11] = (float) cov_varis(8, 8);

	// open vins does not estimate this, but reports it
	double imu_angular_vel[3];
	Eigen::Matrix<double, 3, 1> imu_vel = current_state->_imu->vel();
	imu_angular_vel[0] = imu_vel(0);
	imu_angular_vel[1] = imu_vel(1);
	imu_angular_vel[2] = imu_vel(2);

	Eigen::Matrix<double, 3, 1> imu_angular_vel_holder(imu_angular_vel);
	imu_angular_vel_holder = flu_ned_correction_mat * imu_angular_vel_holder;

	//imu_angular_vel_holder = world_correction_mat * imu_angular_vel_holder;
	s.imu_angular_vel[0] = imu_angular_vel_holder(0);
	s.imu_angular_vel[1] = imu_angular_vel_holder(1);
	s.imu_angular_vel[2] = imu_angular_vel_holder(2);

	// STATE mgmt
	alt_z = imu_wrt_wio_holder(gravity_axis);   // TODO FIX

	// since open vins does the gravity alignment internally, gravity vec is always 0,0,1 and cov is 0'd out BUT
	// voxl flips it to actual
	if (gravity_axis == 0)
	{
		// NOTE -1 to flip gravity from body coordindates!
		float grav_vec[3] =
			{ (float) -1 * gravity_vector_direction, 0, 0 };
		memcpy(s.gravity_vector, grav_vec, sizeof(float) * 3);
	}
	else if (gravity_axis == 2)
	{
		float grav_vec[3] =
			{ 0, 0, (float) gravity_vector_direction };
		memcpy(s.gravity_vector, grav_vec, sizeof(float) * 3);
	}

	// limit the number of features to what fits in our pipe packet
	d.n_total_features = (int) curr_pixel_locs.size();
	if (d.n_total_features > VIO_MAX_REPORTED_FEATURES)
	{
		d.n_total_features = VIO_MAX_REPORTED_FEATURES;
	}
	
	memcpy(d.features, curr_pixel_locs.data(),
			d.n_total_features * sizeof(vio_feature_t));

	s.quality = map_double(T_uncertainty, 0.004375, max_allowable_cep, 100, 0);

	if (s.quality > 100)
		s.quality = 100;
	
	if (s.quality < 1)
		s.quality = 0;
	
	if (pause_qmin)
		s.quality = 1;

	if (en_auto_reset && !init_failure_detector_reset_flag
			&& stable_state(s.state))
	{
	
		if (fabs(baro_alt) < 1.0 && fabs(d_baro) < 1.0 && fabs(s.T_imu_wrt_vio[2]) > 5.0)
			printf("INFO: MAYBE ON GROUND, POSITION LOCK? -- INVOKE ESTOP! (This is just a warning) %f %f %f\n",
					baro_alt, 	d_baro, s.T_imu_wrt_vio[2]);
		
		bool vio_manager_bad = current_state->error_flag == VIO_STATE_FAILED;
		bool quality_bad = imu_moved && s.quality < 1;
		bool stable_quality_bad = imu_moved && stable_quality(s.quality);
		bool stable_features_bad = imu_moved && stable_features(n_good_points);
		bool too_fast = imu_vel.norm() > auto_reset_max_velocity;
		bool too_uncertain = is_armed && V_uncertainty > auto_reset_max_v_cov_instant;
		
		if (	vio_manager_bad
				|| quality_bad
				|| stable_quality_bad 
				|| stable_features_bad
				|| too_fast
				|| too_uncertain)
		{
			if (vio_manager_bad)
			{
				fprintf(stderr,"[VIO_BAD_STATE] State flag was set to FAIL.\n");
			}
			else if (quality_bad)
			{
				fprintf(stderr,"[VIO_BAD_STATE] Quality less than 1.\n");
			}
			else if (stable_quality_bad)
			{
				fprintf(stderr,"[VIO_BAD_STATE] Quality was not STABLE.\n");
			}
			else if (stable_features_bad)
			{
				fprintf(stderr,"[VIO_BAD_STATE] No Stable feature.\n");
			}
			else if (too_fast)
			{
				fprintf(stderr,"[VIO_BAD_STATE] Exceeded MAX IMU Velocity %f vs %f\n", imu_vel.norm(),
						auto_reset_max_velocity);
			}
			else if (too_uncertain)
			{
				fprintf(stderr,"[VIO_BAD_STATE] Exceeded V_uncertainty. %f vs %f\n", V_uncertainty,
						auto_reset_max_v_cov_instant);
			}
			else
				fprintf(stderr,"[CRITICAL] UNKNOWN RESET TRIGGERED\n");
			
			fprintf(stderr,
					"WARNING auto-resetting, Bad VIO, State: %s   Q: %d   Vel: %f Uncert: %f State: %d(%d)\n",
							(current_state->error_flag == VIO_STATE_FAILED) ? "false" : " true", 
							s.quality, 
							imu_vel.norm(),
							V_uncertainty,
							s.state,
							stable_state(s.state));
			s.state = VIO_STATE_FAILED;
			
			init_failure_detector_reset_flag = 1;
		}
	}
	
	// fill in simplified struct inside the extended packet
	memcpy(&d.v, &s, sizeof(vio_data_t));
	
	if ((pipe_server_get_num_clients(OVSTATUS_CH) > 0) && en_ov_stats)
	{
		ov_status.timestamp_ns = s.timestamp_ns;
		ov_status.quality = s.quality;
		ov_status.p_dop = T_uncertainty;
		ov_status.r_dop = R_uncertainty;
		ov_status.num_features = curr_pixel_locs.size();

		unsigned int max_sz = curr_pixel_locs.size();
		if (max_sz >= VIO_MAX_REPORTED_FEATURES)
			max_sz = VIO_MAX_REPORTED_FEATURES - 1;

		for (size_t b = 0; b < max_sz; b++)
		{
			ov_status.features[b].id = curr_pixel_locs[b].id;
			ov_status.features[b].cam_id = curr_pixel_locs[b].cam_id;
			ov_status.features[b].pix_loc[0] = curr_pixel_locs[b].pix_loc[0];
			ov_status.features[b].pix_loc[1] = curr_pixel_locs[b].pix_loc[1];
		}
		pipe_server_write(OVSTATUS_CH, (char*) &ov_status, sizeof(ov_status_t));
	}

	if (en_debug_pos)
	{
		printf("%6.3f %6.3f %6.3f ", (double) s.T_imu_wrt_vio[0],
				(double) s.T_imu_wrt_vio[1], (double) s.T_imu_wrt_vio[2]);
		printf("\n");
	}

	// turn off dropped frame code now we have informed everyone.
	global_error_codes &= ~ERROR_CODE_DROPPED_CAM;

	display_snapshot.d = d;
	display_snapshot.timestamp = last_real_pose_timestamp_ns;
	display_snapshot.used_pts = n_good_points;
	display_snapshot.not_used_pts = n_oos_points;
	display_snapshot.q = s.quality;
	display_snapshot.cep = T_uncertainty;
	display_snapshot.rerr = R_uncertainty;

	pipe_server_write(EXTENDED_CH, (char*) &d, sizeof(ext_vio_data_t));
	pipe_server_write(SIMPLE_CH, (char*) &s, sizeof(vio_data_t));

	return;
}

static ov_msckf::VioManagerOptions generate_open_vins_manager_options()
{
	ov_msckf::VioManagerOptions vio_manager_options;

	/// STATE OPTIONS ///
	vio_manager_options.state_options.do_fej = do_fej;
	vio_manager_options.state_options.imu_avg = imu_avg;
	vio_manager_options.state_options.use_rk4_integration = use_rk4_integration;
	vio_manager_options.state_options.do_calib_camera_pose =
			cam_to_imu_refinement;
	vio_manager_options.state_options.do_calib_camera_intrinsics =
			cam_intrins_refinement;
	vio_manager_options.state_options.do_calib_camera_timeoffset =
			cam_imu_ts_refinement;
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
	vio_manager_options.init_options.init_max_disparity = zupt_max_disparity;
	vio_manager_options.init_options.init_dyn_use = init_dyn_use;
	
	/// IMU NOISE OPTIONS ///
	vio_manager_options.imu_noises.sigma_w = imu_sigma_w;
	vio_manager_options.imu_noises.sigma_wb = imu_sigma_wb;
	vio_manager_options.imu_noises.sigma_a = imu_sigma_a;
	vio_manager_options.imu_noises.sigma_ab = imu_sigma_ab;

	vio_manager_options.imu_noises.sigma_w_2 = std::pow(
			vio_manager_options.imu_noises.sigma_w, 2);
	vio_manager_options.imu_noises.sigma_wb_2 = std::pow(
			vio_manager_options.imu_noises.sigma_wb, 2);
	vio_manager_options.imu_noises.sigma_a_2 = std::pow(
			vio_manager_options.imu_noises.sigma_a, 2);
	vio_manager_options.imu_noises.sigma_ab_2 = std::pow(
			vio_manager_options.imu_noises.sigma_ab, 2);

	/// FEATURE OPTIONS - all use the same struct, can be dif per feature set ///
	// msckf
	vio_manager_options.msckf_options.chi2_multipler = msckf_chi2_multiplier;
	vio_manager_options.msckf_options.sigma_pix = msckf_sigma_px;
	vio_manager_options.msckf_options.sigma_pix_sq = std::pow(
			vio_manager_options.msckf_options.sigma_pix, 2);

	// slam
	vio_manager_options.slam_options.chi2_multipler = slam_chi2_multiplier;
	vio_manager_options.slam_options.sigma_pix = slam_sigma_px;
	vio_manager_options.slam_options.sigma_pix_sq = std::pow(
			vio_manager_options.slam_options.sigma_pix, 2);
	// zupt
	vio_manager_options.zupt_options.chi2_multipler = zupt_chi2_multiplier; // set to 0 for only display based zupt
	vio_manager_options.zupt_options.sigma_pix = zupt_sigma_px;
	vio_manager_options.zupt_options.sigma_pix_sq = std::pow(
			vio_manager_options.zupt_options.sigma_pix, 2);

	/// ZUPT OPTIONS ///
	vio_manager_options.try_zupt = try_zupt;
	vio_manager_options.zupt_max_velocity = zupt_max_velocity;
	vio_manager_options.zupt_only_at_beginning = zupt_only_at_beginning;
	vio_manager_options.zupt_noise_multiplier = zupt_noise_multiplier;
	vio_manager_options.zupt_max_disparity = zupt_max_disparity; // set to 0 for only imu based zupt

	/// GENERAL OPTIONS ///
	vio_manager_options.use_stereo = use_stereo;
	vio_manager_options.use_mask = use_mask;
	vio_manager_options.use_aruco = false; //MODALAI ONLY

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

	vio_manager_options.grid_x = grid_x;
	vio_manager_options.grid_y = grid_y;
	vio_manager_options.pyramid_levels = pyramid_levels;

	vio_manager_options.downsample_cameras = false; // TBD
	vio_manager_options.num_opencv_threads = num_opencv_threads;
	vio_manager_options.num_pts = num_features_to_track;
	vio_manager_options.fast_threshold = fast_threshold;
	vio_manager_options.min_px_dist = min_pix_dist;

	vio_manager_options.histogram_method = histogram_method;
	vio_manager_options.knn_ratio = knn_ratio;
	vio_manager_options.track_frequency = track_frequency;

	/// CAMERA INTRINSICS + EXTRINSICS ///

	std::lock_guard < std::mutex > lg(vio_manager_mutex);

	std::shared_ptr < ov_core::CamBase > cam_calib_intrinsic;
	for (size_t i = 0; i < cam_info_vec.size(); i++)
	{
		// ov uses camequi model to represent fisheye cameras, and the radtan model for standard lenses
		if (cam_info_vec[i].is_fisheye)
		{
			std::cout << "OpenVINS using fisheye camera" << std::endl;

			cam_calib_intrinsic =
					std::make_shared < ov_core::CamEqui
							> (cam_info_vec[i].cam_calib_intrinsic(8, 0), cam_info_vec[i].cam_calib_intrinsic(
									9, 0));
		}
		else
		{
			std::cout << "OpenVINS using plumb-bob camera" << std::endl;

			cam_calib_intrinsic =
					std::make_shared < ov_core::CamRadtan
							> (cam_info_vec[i].cam_calib_intrinsic(8, 0), cam_info_vec[i].cam_calib_intrinsic(
									9, 0));
		}
		// The camera intrinsics
		cam_calib_intrinsic->set_value(cam_info_vec[i].cam_calib_intrinsic);
		vio_manager_options.camera_intrinsics[i] = cam_calib_intrinsic;
		vio_manager_options.camera_extrinsics[i] = cam_info_vec[i].cam_wrt_imu;

		std::cout << "OpenVINS FINAL Cam extrinsics to IMU for "
				<< cam_info_vec[i].name << ": "
				<< cam_info_vec[i].cam_wrt_imu.size() << " \n "
				<< cam_info_vec[i].cam_wrt_imu << std::endl;
	}

	vio_manager_options.init_options.camera_intrinsics =
			vio_manager_options.camera_intrinsics;
	vio_manager_options.init_options.camera_extrinsics =
			vio_manager_options.camera_extrinsics;
	// this needs to be set in both locations, for some reason the updaters address both in different stages
	vio_manager_options.init_options.num_cameras = cam_info_vec.size();
	vio_manager_options.state_options.num_cameras = cam_info_vec.size();

	std::cout << " ++++++++++++++++++++++++++\n\n"
			<< "OpenVINS reports number of cameras being used, regardless of delivery method (pipe, merge, etc...):  "
			<< vio_manager_options.init_options.num_cameras
			<< "\n\n ++++++++++++++++++++++++++" << std::endl;

	return vio_manager_options;
}

static void _feat_disconnect_cb(__attribute__((unused)) int ch,
		__attribute__((unused)) void *context)
{
    destroy_vft_memory(&feature_packet_from_vft, &features_from_vft);
	return;
}


//static void _feat_connect_cb(__attribute__((unused)) int ch,
//                        __attribute__((unused)) void* context) {
//    // get some space for the features
//    create_vft_memory(&feature_packet_from_vft, &features_from_vft);
//    return;
//}

// called whenever we disconnect from the server

static void _imu_disconnect_cb(__attribute__((unused)) int ch,
		__attribute__((unused)) void *context)
{
	std::lock_guard < std::mutex > lg(imu_lock_mutex);
	return;
}

static void _baro_disconnect_cb(__attribute__((unused)) int ch,
		__attribute__((unused)) void *context)
{
	std::lock_guard < std::mutex > baro_lg(baro_lock_mutex);
	return;
}

// TODO:: change naming from BARO to MAVLINK
static void _new_baro_data_default_handler(__attribute__((unused)) int ch,
		char *data, int bytes, __attribute__((unused)) void *context)
{
	static double last_val = -99999;
	static bool base_init = false;
	static int a_ctn = 0;
	static double zero_alt = 0;

	std::lock_guard < std::mutex > baro_lg(baro_lock_mutex);

	mavlink_message_t *baro_msg = (mavlink_message_t*) data;

 	if (use_baro && (baro_msg->msgid == MAVLINK_MSG_ID_SCALED_PRESSURE))
	{
		// basic sanity checks
		if (bytes < 0)
		{
			fprintf(stderr,
					"ERROR validating BARO data received through pipe: number of bytes = %d\n",
					bytes);
			return;
		}
		if (data == NULL)
		{
			fprintf(stderr,
					"ERROR validating BARO data received through pipe: got NULL data pointer\n");
			return;
		}

		uint32_t baro_time_ms = mavlink_msg_scaled_pressure_get_time_boot_ms(
				baro_msg);
		float pressure = mavlink_msg_scaled_pressure_get_press_abs(baro_msg)
				* 100;
		int16_t temp = mavlink_msg_scaled_pressure_get_temperature(baro_msg)
				/ 100 - 10;

		//static auto rT0_begin = _apps_time_monotonic_ns();
		//auto rT0_end = _apps_time_monotonic_ns();
//		static auto rT0_begin = baro_time_ms;
//		auto rT0_end = baro_time_ms;

		double dist = (log(pressure / 101325) * 8.31432 * (temp + 273.15))
				/ (-9.80665 * 0.0289644);
//			printf("Pressure: %f temp: %d\n", pressure, temp);
//		printf("Baro height: %f (%f)\n", dist, log(pressure/101325));

		if (!base_init)
		{
			zero_alt += dist;
			vio_manager->add_constraint_baro(0, 0);

			if (++a_ctn > 10)
			{
				zero_alt /= a_ctn;

				printf("ZERO ALT: %f\n", zero_alt);
				base_init = true;
			}

		}
		else
		{
			baro_alt = dist - zero_alt;
			if (last_val != -99999)
			{
				baro_alt = (0.7 * baro_alt) + (0.3 * last_val);

				//v = dx/dt
//					double baro_vel_z = (baro_alt - last_val) / 0.1;   // 20Hz
//					double time_total = (rT0_end - rT0_begin) * 1e-9;

//				double time_total = (rT0_end - rT0_begin) * 1e-3;
				double baro_vel_z = (baro_alt - last_val) / 0.1;  // 10Hz

//					printf("Baro Vel (%f) m/s ---  Alt: %f m\n", baro_vel_z, baro_alt);
//					printf("Got Baro data %f %f (%f) meters at %f (m/s)\n", time_total, baro_alt, zero_alt, baro_vel_z);

				// TODO FIX, recently broken
				// vio_manager->add_constraint_baro(baro_alt, baro_vel_z);
			}
			
			d_baro = baro_alt - last_val;
			last_val = baro_alt;
		}

//		rT0_begin = rT0_end;
	}

 	if (baro_msg->msgid == MAVLINK_MSG_ID_HEARTBEAT && !offline)
 	{
 		// get ARMED state
 		 is_armed = mavlink_msg_heartbeat_get_system_status(baro_msg) == MAV_STATE_ACTIVE;
 	}

 	if (baro_msg->msgid == MAVLINK_MSG_ID_VFR_HUD)
 	{
 		throttle_state = mavlink_msg_vfr_hud_get_throttle(baro_msg);
 	}
 	
 	// TODO make more elegant, this is a bad indication using non-throttle  autonomous modes
 	if (en_ext_feature_tracker  && !is_armed && throttle_state == 0)
 	{
 		use_takeoff_cam = true;
 		ext_blind_take_off_force = true;
 	}	
 	else
 	{
 		ext_blind_take_off_force = false;
 	}
}

static void _simple_cpu_cb(__attribute__((unused))int ch, char *raw_data,
		int bytes, __attribute__((unused)) void *context)
{

	int n_packets;
	cpu_stats_t *data_array = modal_cpu_validate_pipe_data(raw_data, bytes,
			&n_packets);
	if (data_array == NULL)
		return;

	// only use most recent packet
	cpu_stats_t data = data_array[n_packets - 1];

#ifdef USE_ACTIVE_CPU_MANAGEMENT
	bool is_standby  = data.flags & CPU_STATS_FLAG_STANDBY_ACTIVE;
	if (is_standby && vio_manager_options.zupt_max_disparity == zupt_max_disparity)
	{
		vio_manager_options.init_options.init_max_disparity =  zupt_max_disparity + 2;
		vio_manager_options.zupt_max_disparity = zupt_max_disparity + 2;
	}
	if (data.cpu_t_max > 85.0 || is_standby)
#endif
		
	if (data.cpu_t_max > 85.0 || !is_armed)
	{
		if (!changed_motion_state)
			idler_limit = 3; // 12Hz
		else 
			idler_limit = 0;
	}
	else
	{
		idler_limit = 0;
	}
	
#ifdef DEV
    for(int i=0; i < data.num_cpu; i++){
            printf("cpu%d %12.1f %s%8.1f %s%8.2f%s\n",
                    i,
                    (double)data.cpu_freq[i],
                    GET_COLOR_GT(data.cpu_t[i], TEMP_RED_THRESH, TEMP_YLW_THRESH),
                    (double)data.cpu_t[i],
                    GET_COLOR_GT(data.cpu_load[i], LOAD_RED_THRESH, LOAD_YLW_THRESH),
                    (double)data.cpu_load[i],
                    RESET_FONT);
    }


    // Standby
    if(data.flags&CPU_STATS_FLAG_STANDBY_ACTIVE){
            printf("Standby Active\n");
    }
    else printf("Standby Not Active\n");

    // overload flags
    if(data.flags&CPU_STATS_FLAG_CPU_OVERLOAD){
            printf(COLOR_RED "CPU OVERLOAD WARNING" RESET_FONT "\n");
    }
    if(data.flags&CPU_STATS_FLAG_CPU_OVERHEAT){
            printf(COLOR_RED "CPU OVERHEAT WARNING" RESET_FONT "\n");
    }
#endif

	return;
}

static void _disconnect_cpu_cb(__attribute__((unused)) int ch,
		__attribute__((unused)) void *context)
{
	return;
}

static int create_server_pipes(void)
{
	int flags = SERVER_FLAG_EN_CONTROL_PIPE;

	// init extended pipe
	pipe_info_t
	info1 =
	{
		OV_VIO_EXTENDED_NAME,       // name
		OV_VIO_EXTENDED_LOCATION,// location
		"ext_vio_data_t",// type
		PROCESS_NAME,// server_name
		EXT_VIO_RECOMMENDED_READ_BUF_SIZE,// size_bytes
		0			  // server_pid
	};

	if (pipe_server_create(EXTENDED_CH, info1, flags))
	{
		printf("pipe_server_create(EXTENDED_CH, info1, flags) failed\n");
		_quit(-1);
	}

	// add in optional fields to the info JSON file
	cJSON *json = pipe_server_get_info_json_ptr(EXTENDED_CH);
	cJSON_AddStringToObject(json, "imu", imu_name);
	pipe_server_update_info(EXTENDED_CH);
	pipe_server_set_control_cb(EXTENDED_CH, _control_pipe_cb, NULL);
	pipe_server_set_available_control_commands(EXTENDED_CH,
			OV_VIO_CONTROL_COMMANDS);

	// init simple pipe
	pipe_info_t
	info2 =
	{
		OV_VIO_SIMPLE_NAME,         // name
		OV_VIO_SIMPLE_LOCATION,// location
		"vio_data_t",// type
		PROCESS_NAME,// server_name
		VIO_RECOMMENDED_PIPE_SIZE,// size_bytes
		0// server_pid
	};

	if (pipe_server_create(SIMPLE_CH, info2, flags))
	{
		printf("pipe_server_create(SIMPLE_CH, info2, flags) failed\n");
		_quit(-1);
	}

	// add in optional fields to the info JSON file
	json = pipe_server_get_info_json_ptr(SIMPLE_CH);
	cJSON_AddStringToObject(json, "imu", imu_name);
	
	pipe_server_update_info(SIMPLE_CH);
	pipe_server_set_control_cb(SIMPLE_CH, _control_pipe_cb, NULL);
	pipe_server_set_available_control_commands(SIMPLE_CH,
			OV_VIO_CONTROL_COMMANDS);

	// init overlay pipe
	pipe_info_t
	info3 =
	{
		OV_VIO_OVERLAY_NAME,        // name
		OV_VIO_OVERLAY_LOCATION,// location
		"camera_image_metadata_t",// type
		PROCESS_NAME,// server_name
		CAM_PIPE_SIZE,// size_bytes
		0// server_pid
	};

	if (pipe_server_create(OVERLAY_CH, info3, flags))
	{
		printf("pipe_server_create(OVERLAY_CH, info3 failed\n");

		_quit(-1);
	}

	pipe_server_set_connect_cb(OVERLAY_CH, _overlay_connect_cb, NULL);
	pipe_server_set_disconnect_cb(OVERLAY_CH, _overlay_disconnect_cb, NULL);
	pipe_server_set_control_cb(OVERLAY_CH, _control_pipe_cb, NULL);
	pipe_server_set_available_control_commands(OVERLAY_CH,
			OV_VIO_CONTROL_COMMANDS);

	// init simple pipe
	pipe_info_t
	info4 =
	{
		OV_STATUS_NAME,         // name
		OV_STATUS_LOCATION,// location
		"ov_status_t",// type
		PROCESS_NAME,// server_name
		VIO_RECOMMENDED_PIPE_SIZE,// size_bytes
		0// server_pid
	};

	if (pipe_server_create(OVSTATUS_CH, info4, flags))
	{
		printf("pipe_server_create(SIMPLE_CH, info2, flags) failed\n");
		_quit(-1);
	}

	return 0;
}

static int read_external_configs_from_file(void)
{
	fprintf(stderr,
			"=====> imu to read from: %s\n",
			imu_name);

	int ret = json_fetch_string(cam_json, "imu", imu_name, 128);

	bool is_wrldc_set = false;

	cJSON *cams = cJSON_GetObjectItem(cam_json, "cameras");
	if (cams == NULL)
	{
		fprintf(stderr, "failed to get cameras\n");
	}

	int i = 0;
	size_t cam_id = 0;
	while (ret == 0)
	{
		cJSON *curr_cam = cJSON_GetArrayItem(cams, cam_id);
		if (curr_cam == NULL)
		{
			// fprintf(stderr, "failed to get curr_cam\n");
			break;
		}

		double arr_cam_wrt_imu[7];
		double arr_cam_calib[10];
		double arr_world_correction[9];
		int is_fisheye;

		ret = json_fetch_fixed_vector(curr_cam, "ov_cam_wrt_imu",
				arr_cam_wrt_imu, 7);
		ret = json_fetch_fixed_vector(curr_cam, "ov_cam_cal", arr_cam_calib,
				10);
		if (!is_wrldc_set)
			ret = json_fetch_fixed_vector(curr_cam, "ov_world_correction",
					arr_world_correction, 9);
		ret = json_fetch_bool(curr_cam, "fisheye", &is_fisheye);

		Eigen::Matrix<double, 7, 1> curr_cam_wrt_imu(arr_cam_wrt_imu);
		Eigen::Matrix<double, 10, 1> curr_cam_calib_intrinsic(arr_cam_calib);

		if (curr_cam_calib_intrinsic(8, 0) > max_width)
			max_width = curr_cam_calib_intrinsic(8, 0);
		if (curr_cam_calib_intrinsic(9, 0) > max_height)
			max_height = curr_cam_calib_intrinsic(9, 0);

		if (!is_wrldc_set)
		{
			world_correction = cv::Mat(3, 3, CV_64F);
			memcpy(world_correction.data, arr_world_correction,
					3 * 3 * sizeof(double));
			is_wrldc_set = true;
		}
		// now populate our vector with this information
		camera_info curr_info;
		curr_info.is_fisheye = is_fisheye;
		curr_info.cam_wrt_imu = curr_cam_wrt_imu;
		curr_info.cam_calib_intrinsic = curr_cam_calib_intrinsic;
		curr_info.cam_id = cam_id;
		cam_id++;

		// fetch the name as well directly into this packet
		ret = json_fetch_string(curr_cam, "cam name", curr_info.name, 128);

		// fetch the mode
		char mode_buf[128];
		ret = json_fetch_string(curr_cam, "cam mode", mode_buf, 128);
		curr_info.mode = string_camera_mode_to_enum(mode_buf);

		cam_info_vec.push_back(curr_info);
	}

	free (cam_json);

	return 0;
}

////////////////////////////////////////////////////////////////
#ifdef DEPRECATED_SINCE_SDK_1_1_0

static int read_external_configs(void)
{

	// connect to our feature tracker
	pipe_client_set_disconnect_cb(FEATURE_CH, _feat_disconnect_cb, NULL);
	pipe_client_set_simple_helper_cb(FEATURE_CH, _new_feat_data_default_handler,
			NULL);
	
	if (pipe_client_open(FEATURE_CH, FEATURE_LOCATION, PROCESS_NAME,
			CLIENT_FLAG_EN_SIMPLE_HELPER, 1280 * 800 * 1) != 0)
	{
		printf(
				"pipe_client_open(FEATURE_CH, FEATURE_LOCATION, PROCESS_NAME, .... failed\n");
		_quit(-1);
	}
	usleep(5000);

	// grab the json from voxl-feature-tracker's info file

	// TODO FIX this as it requires freature tracker to get camera intrinsics/extrinsics
	cJSON *json = pipe_client_get_info_json(FEATURE_CH);

	int ret = json_fetch_string(json, "imu", imu_name, 128);

	bool is_wrldc_set = false;

	cJSON *cams = cJSON_GetObjectItem(json, "cameras");
	if (cams == NULL)
	{
		fprintf(stderr, "failed to get cameras\n");
	}

	int i = 0;
	size_t cam_id = 0;
	while (ret == 0)
	{

		cJSON *curr_cam = cJSON_GetArrayItem(cams, cam_id);
		if (curr_cam == NULL)
		{
			// fprintf(stderr, "failed to get curr_cam\n");
			break;
		}

		double arr_cam_wrt_imu[7];
		double arr_cam_calib[10];
		double arr_world_correction[9];
		int is_fisheye;

		ret = json_fetch_fixed_vector(curr_cam, "ov_cam_wrt_imu",
				arr_cam_wrt_imu, 7);
		ret = json_fetch_fixed_vector(curr_cam, "ov_cam_cal", arr_cam_calib,
				10);
		if (!is_wrldc_set)
			ret = json_fetch_fixed_vector(curr_cam, "ov_world_correction",
					arr_world_correction, 9);
		ret = json_fetch_bool(curr_cam, "fisheye", &is_fisheye);

		Eigen::Matrix<double, 7, 1> curr_cam_wrt_imu(arr_cam_wrt_imu);
		Eigen::Matrix<double, 10, 1> curr_cam_calib_intrinsic(arr_cam_calib);

		if (curr_cam_calib_intrinsic(8, 0) > max_width)
			max_width = curr_cam_calib_intrinsic(8, 0);
		if (curr_cam_calib_intrinsic(9, 0) > max_height)
			max_height = curr_cam_calib_intrinsic(9, 0);

		if (!is_wrldc_set)
		{
			world_correction = cv::Mat(3, 3, CV_64F);
			memcpy(world_correction.data, arr_world_correction,
					3 * 3 * sizeof(double));
			is_wrldc_set = true;
		}
		// now populate our vector with this information
		camera_info curr_info;
		curr_info.is_fisheye = is_fisheye;
		curr_info.cam_wrt_imu = curr_cam_wrt_imu;
		curr_info.cam_calib_intrinsic = curr_cam_calib_intrinsic;
		curr_info.cam_id = cam_id;
		cam_id++;

		// fetch the name as well directly into this packet
		ret = json_fetch_string(curr_cam, "cam name", curr_info.name, 128);

		// fetch the mode
		char mode_buf[128];
		ret = json_fetch_string(curr_cam, "cam mode", mode_buf, 128);
		curr_info.mode = string_camera_mode_to_enum(mode_buf);

		cam_info_vec.push_back(curr_info);
	}

	// free up the json we got
	free(json);

	return ret;
}
#endif
////////////////////////////////////////////////////////////////

static bool connect_vft_service()
{
	// connect to our feature tracker
	pipe_client_set_disconnect_cb(FEATURE_CH, _feat_disconnect_cb, NULL);
	pipe_client_set_simple_helper_cb(FEATURE_CH, _new_feat_data_default_handler,
			NULL);
	create_vft_memory(&feature_packet_from_vft, &features_from_vft);
	
	if (pipe_client_open(FEATURE_CH, vft_name, PROCESS_NAME, EN_PIPE_CLIENT_SIMPLE_HELPER /*CLIENT_FLAG_EN_SIMPLE_HELPER*/, 
			sizeof(vft_feature_packet)) != 0)
	{
		printf(
				"pipe_client_open(FEATURE_CH, FEATURE_LOCATION, PROCESS_NAME, .... failed\n");
		_quit(-1);
	}

	printf("VFT connected\n");

	return true;
}


static bool connect_mavlink_service()
{

	// connect to baro
	pipe_client_set_disconnect_cb(BARO_CH, _baro_disconnect_cb, NULL);
	pipe_client_set_simple_helper_cb(BARO_CH,
			_new_baro_data_default_handler, NULL);
	if (pipe_client_open(BARO_CH, baro_name, PROCESS_NAME, CLIENT_FLAG_EN_SIMPLE_HELPER,
			IMU_RECOMMENDED_READ_BUF_SIZE) != 0)
	{
		fprintf(stderr, "failed to open MAVLINK and Baro\n");
		_quit(-1);
	}

	return true;
}

static int connect_client_pipes(void)
{
	fprintf(stderr, "connecting client pipes\n");
	// connect to imu
	pipe_client_set_disconnect_cb(IMU_CH, _imu_disconnect_cb, NULL);
	pipe_client_set_simple_helper_cb(IMU_CH, _new_imu_data_default_handler,
			NULL);
	int flags = CLIENT_FLAG_EN_SIMPLE_HELPER;
	if (pipe_client_open(IMU_CH, imu_name, PROCESS_NAME, flags,
			IMU_RECOMMENDED_READ_BUF_SIZE) != 0)
	{
		fprintf(stderr, "failed to open\n");
		return -1;
	}

	// connect to all configured cameras
	cameras_used = cam_info_vec.size();
	char t_cam_nam[256];

	fprintf(stderr, "Number of Cameras active: %d\n", cameras_used);
	std::vector < std::string > tmp_camera_pipe_names;

	for (int i = 0; i < cameras_used; i++)
	{
		int ch = pipe_client_get_next_available_channel();
		camera_pipe_channels[i] = ch;

		fprintf(stderr, "Camera merge --- > ch: %d to cam id: %d\n", ch, i);

		sprintf(t_cam_nam, "%s", cam_info_vec[i].name);

		if (std::find(tmp_camera_pipe_names.begin(),
				tmp_camera_pipe_names.end(), t_cam_nam)
				== tmp_camera_pipe_names.end())
		{
			// pipe_client_set_disconnect_cb(FEAT_OVERLAY_CH, _imu_disconnect_cb, NULL);
			pipe_client_set_camera_helper_cb(ch, _cam_helper_cb,
					&cam_info_vec[i].mode);
			flags = CLIENT_FLAG_EN_CAMERA_HELPER;
			int ret = pipe_client_open(ch, t_cam_nam, PROCESS_NAME, flags,
					1280 * 800 * 15);
			if (ret)
			{
				fprintf(stderr, "failed to open %s\n", cam_info_vec[i].name);
				return -1;
			}
			else
			{
				fprintf(stderr, "Opening camera pipe: %s\n",
						cam_info_vec[i].name);
			}

			fprintf(stderr, "tmp_camera_pipe_names.push_back(): %s\n",
					t_cam_nam);

			tmp_camera_pipe_names.push_back(t_cam_nam);
		}
		else
		{
			fprintf(stderr,
					"Note: found camera pipe callback already exists, likely a stereo camera setup\n");
		}

	}

	fprintf(stderr, "imu pipe name: %s\n", imu_name);

	connect_mavlink_service();

	if (en_ext_feature_tracker)
	{
		connect_vft_service();
	}	
	return 0;
}

int main(int argc, char *argv[])
{
	// Parse the command line options and terminate if the parser says we should terminate
	if (_parse_opts(argc, argv))
	{
		printf("exiting server\n");
		return -1;
	}

	// Load the config files
	printf("Loading our own config file\n");
	if (config_file_read() < 0)
	{
		printf("config_file_read() < 0\n");
		_quit(-1);
	}
	config_file_print();

	// read camera multicam setup and configs
	if (cam_config_file_read(is_color, is_single, set_tracker_mode) < 0)
	{
		fprintf(stderr, "ERROR cam_config_file_read\n");
		_quit(-1);
	}
	cam_config_file_print();

	// load external info
	printf("Loading external config file\n");
	if (read_external_configs_from_file() < 0)
	{
		printf ("read_external_configs_from_file() < 0\n");
		_quit(-1);
	}
	
	if (en_config_only)
	{
		_quit(-1);
	}
	// coordindate system correction OVINS only needed
	flu_ned_correction_mat(1, 1) = -1;
	flu_ned_correction_mat(2, 2) = -1;
	
	// Create the VIO Manager -- Core OpenVINS state
	vio_manager_options = generate_open_vins_manager_options();

	if ((world_correction.at<double>(0, 0) * world_correction.at<double>(1, 1))
			> 0)
		gravity_vector_direction = 1;
	else
		gravity_vector_direction = -1;

	vio_manager_options.use_klt = !en_ext_feature_tracker;		

	vio_manager = std::unique_ptr < ov_msckf::VioManager
			> (new ov_msckf::VioManager(vio_manager_options));

	/* make sure another instance isn't running
	 * if return value is -3 then a background process is running with
	 * higher privaledges and we couldn't kill it, in which case we should
	 * not continue or there may be hardware conflicts. If it returned -4
	 * then there was an invalid argument that needs to be fixed.
	 */
	if (kill_existing_process(PROCESS_NAME, 2.0) < -2)
	{
		std::cerr << "ERROR: could not kill existing process" << std::endl;
		_quit(-1);
	}

	// start signal handler so we can exit cleanly
	if (enable_signal_handler() == -1)
	{
		std::cerr << "ERROR: failed to start signal handler" << std::endl;
		_quit(-1);
	}

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	make_pid_file(PROCESS_NAME);

	////////////////////////////////////////////////////////////////////////////////
	// set this critical process to use FIFO scheduler with high priority
	////////////////////////////////////////////////////////////////////////////////

	struct sched_param param;
	memset(&param, 0, sizeof(sched_param));
	param.sched_priority = 98;
	// we go with _RR as I want equal time on IMU and camera threads.
	fprintf(stderr, "setting scheduler\n");
	int ret = sched_setscheduler(0, SCHED_RR, &param);
	if (ret == -1)
	{
		fprintf(stderr, "WARNING Failed to set priority, errno = %d\n", errno);
		fprintf(stderr, "This seems to be a problem with ADB, the scheduler\n");
		fprintf(stderr,
				"should work properly when this is a background process\n");
	}
	// check
	ret = sched_getscheduler(0);
	if (ret != SCHED_RR)
	{
		fprintf(stderr, "WARNING: failed to set scheduler\n");
	}
	else
	{
		// even thought this is a success, print to stderr to that it shows up
		// in the correct order. stdout logs in journalctl are usually out of
		// sync with stderr
		fprintf(stderr, "INFO: set FIFO priority successfully!\n");
	}
	// The threads created by libmodal_pipe after this should inherit this
	// priority, TODO validate this

	// Create the server pipes
	printf("create_server_pipes\n");
	if (create_server_pipes() < 0)
	{
		printf("create_server_pipes < 0\n");
		_quit(0);
	}
	
	/////////////////////////////////////////////////////////////////////////////////////////////
	// temp disable
	// open vins will occasionally stall out on init, and the health monitor
	// will then cause vvpx4 to switch out of position control
	// comment the below thread creation if this behavior occurs
	/////////////////////////////////////////////////////////////////////////////////////////////

	// until they connect, inidcate that they are disconnected
	global_error_codes |= ERROR_CODE_CAM_MISSING;
	global_error_codes |= ERROR_CODE_IMU_MISSING;

	perf_limit = 1;

	// Connect to the client pipes and start getting data
	if (connect_client_pipes() < 0)
	{
		printf("connect_client_pipes < 0\n");
		_quit(0);
	}
	usleep(100000); // 100ms
	// Set the main running flag to 1 to indicate that we are running
	main_running = 1;

	zeroTimeOut = boost::posix_time::microsec_clock::local_time();

	// run until start/stop module catches a signal and changes main_running to 0
	pthread_attr_t tattr;
	pthread_attr_init(&tattr);
	pthread_create(&health_thread, &tattr, _health_thread_func, NULL);

	pthread_attr_t tattr_overlay;
	pthread_attr_init(&tattr_overlay);
	pthread_create(&overlay_thread, &tattr_overlay, _overlay_thread_func, NULL);

	//////////////////////////
	// connect to CPU monitor for throttling cpu to prevent thermal runaway
	pipe_client_set_simple_helper_cb(CPU_CH, _simple_cpu_cb, NULL);
	pipe_client_set_disconnect_cb(CPU_CH, _disconnect_cpu_cb, NULL);
	// for this test we will use the simple helper
	int flags = EN_PIPE_CLIENT_SIMPLE_HELPER;
	// init connection to server. In auto-reconnect mode this will "succeed"
	// even if the server is offline, but it will connect later on automatically
	ret = pipe_client_open(CPU_CH, CPU_PIPE_DIR, PROCESS_NAME, flags, CPU_STATS_RECOMMENDED_READ_BUF_SIZE);
	// check for success
	if (ret)
	{
		fprintf(stderr, "ERROR opening CPU channel:\n");
		pipe_print_error(ret);
		return -1;
	}
	//////////////////////////

	while (main_running)
	{
		usleep(1e6);
	}

	pthread_join(health_thread, NULL);
	pthread_join(overlay_thread, NULL);

	printf("Quiting VIO server\n");
	// Shutdown Nicely
	_quit(0);

	return 0;
}

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay,
		float az, float *phi, float *theta, float *psi)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if (twoKi > 0.0f)
		{
			integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;      // apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else
		{
			integralFBx = 0.0f;     // prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));           // pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	*theta = asin(-2 * q1 * q3 + 2 * q0 * q2);       // pitch
	*phi = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1)
			+ M_PI; // roll
	*psi = atan2(2 * (q1 * q2 + q0 * q3),
			q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);      //yaw

			//      fprintf(stderr, "r: %6.2f p: %6.2f y: %6.2f\n",
			//                      *phi*180.0/M_PI, *theta*180.0/M_PI,*psi*180.0/M_PI);
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*) &y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*) &i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

static void dead_reckon_position(double dt, float ax, float ay, float az,
		float gx, float gy, float gz)
{

	float phi, theta, psi;

	static bool started = false;

	MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az, &phi, &theta, &psi);

	if (!started)
	{
		float dt_gx = abs(phi - _last_gyro[0]);
		float dt_gy = abs(theta - _last_gyro[1]);
		float dt_gz = abs(psi - _last_gyro[2]);

		_last_gyro[0] = phi;
		_last_gyro[1] = theta;
		_last_gyro[2] = psi;
//              printf("%f %f %f\n", dt_gx, dt_gy, dt_gz);

		if (dt_gx <= 0.00001 && dt_gy <= 0.00001 && dt_gz <= 0.00001)
		{
			printf("IMU settled and ready\n");
			started = true;
			_accel_bias[0] = ax;
			_accel_bias[1] = ay;
			_accel_bias[2] = az;
			_gyro_bias[0] = phi;
			_gyro_bias[1] = theta;
			_gyro_bias[2] = psi;
		}
	}
	else
	{
		phi -= _gyro_bias[0];
		theta -= _gyro_bias[1];
		psi -= _gyro_bias[2];

		float cosPhi = (double) cos(phi);
		float sinPhi = (double) sin(phi);
		float cosThe = (double) cos(theta);
		float sinThe = (double) sin(theta);
		float cosPsi = (double) cos(psi);
		float sinPsi = (double) sin(psi);

		float dcm[3][3];
		dcm[0][0] = cosThe * cosPsi;
		dcm[0][1] = -cosPhi * sinPsi + sinPhi * sinThe * cosPsi;
		dcm[0][2] = sinPhi * sinPsi + cosPhi * sinThe * cosPsi;

		dcm[1][0] = cosThe * sinPsi;
		dcm[1][1] = cosPhi * cosPsi + sinPhi * sinThe * sinPsi;
		dcm[1][2] = -sinPhi * cosPsi + cosPhi * sinThe * sinPsi;

		dcm[2][0] = -sinThe;
		dcm[2][1] = sinPhi * cosThe;
		dcm[2][2] = cosPhi * cosThe;

		float accel_vec[3] =
		{ ax - _accel_bias[0], ay - _accel_bias[1], (az - _accel_bias[2])
				+ 9.80665f };
		float acc_mag = sqrt(
				accel_vec[0] * accel_vec[0] + accel_vec[1] * accel_vec[1]
						+ accel_vec[2] * accel_vec[2]);

		//printf("Staionary: %f %f %f - %f\n", accel_vec[0], accel_vec[1], accel_vec[2], acc_mag);
//            bool stationary = acc_mag < 0.25;
		bool stationary = 0;

		float local_frame_accel[3] =
		{ 0, 0, 0 };

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				local_frame_accel[j] += dcm[j][i] * accel_vec[i];
			}
		}

		if (stationary)
		{
			_vel_imu[0] = 0.;
			_vel_imu[1] = 0.;
			_vel_imu[2] = 0.;
		}
		else
		{
			_vel_imu[0] = _last_vel[0] + local_frame_accel[0] * dt;
			_vel_imu[1] = _last_vel[1] + local_frame_accel[1] * dt;
			_vel_imu[2] = _last_vel[2] + local_frame_accel[2] * dt;
		}

		_last_vel[0] = _vel_imu[0];
		_last_vel[1] = _vel_imu[1];
		_last_vel[2] = _vel_imu[2];

		_odometry_imu[0] = _last_odometry_imu[0] + _vel_imu[0] * dt;
		_odometry_imu[1] = _last_odometry_imu[1] + _vel_imu[1] * dt;
		_odometry_imu[2] = _last_odometry_imu[2] + _vel_imu[2] * dt;
		_last_odometry_imu[0] = _odometry_imu[0];
		_last_odometry_imu[1] = _odometry_imu[1];
		_last_odometry_imu[2] = _odometry_imu[2];

		static int ctn = 0;
		if (ctn++ % 10 == 0)
			fprintf(stderr, "*, %6.2f\t%6.2f\t%6.2f\n", _vel_imu[0],
					_vel_imu[1], _vel_imu[2]);

//              fprintf(stderr, "*, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f\n",
//                                                                      _vel_imu[0],
//                                                                              _vel_imu[1],
//                                                                              _vel_imu[2],
//                                                                              accel_vec[0], accel_vec[1], accel_vec[2],
//                                                                              gx, gy, gz,
//                                                                              (double)phi*180/M_PI,
//                                                                              (double)theta*180/M_PI,
//                                                                              (double)psi*180/M_PI);
	}
}
