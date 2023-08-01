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
#define FEAT_OVERLAY_CH 2
#define BARO_CH 3

#define CAMERA_CH_START_OFFSET 1
#define IMU_PIPE_MIN_PIPE_SIZE (2 * 640 * 640)  // give ourselves huge buffers
#define CAM_PIPE_SIZE (10 * 1280 * 800)         // give ourselves huge buffers
#define PROCESS_NAME "open-vins-server"
#define FEATURE_NAME "tracked_feats"
#define FEATURE_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR FEATURE_NAME "/"
#define FEATURE_OVERLAY_NAME "feat_overlay"
#define FEATURE_OVERLAY_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR FEATURE_OVERLAY_NAME "/"

// after 300ms with no response, the health monitor thread assumes mvVISLAM
// has locked up while processing a frame and starts sending messages indicating
// a stall has occured with a failed state
#define STALL_TIMEOUT_NS 300000000

// auto restart if the system fails to init after 10 seconds
#define INIT_FAILURE_TIMEOUT_NS 10000000000

// do not check for blowups until 1 second after VIO claims to have initialized
#define BLOWUP_DETECT_TIMEOUT_NS 1000000000

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
static pthread_t health_thread;
static pthread_t overlay_thread;

static double last_feature_time;
static double last_imu_time;
static int64_t time_avg;
static int avg  = 0;
static int perf_limit  = 1;
static int start_idx = 0;
static int cameras_used = 0;
static int camera_pipe_channels[10];
static int every_other = 0;
static double T_uncertainty = 0;
static double R_uncertainty = 0;
static int gravity_vector_direction = -1;

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
#define DRAW_BONUS_ROWS_TOP 64
#define DRAW_BONUS_ROWS_BOT 64
static cv::Mat draw_frame;
static camera_image_metadata_t draw_meta;

static cv::Mat cached_overlay;
static camera_image_metadata_t cached_meta;
std::mutex overlay_mutex;

static cv::Mat world_correction;
std::deque<ov_core::CameraData> camera_queue;
std::mutex camera_queue_mtx;

// set any error codes here for publishing in the data structure
static uint32_t global_error_codes = 0;

std::vector<camera_info> cam_info_vec;
static char imu_name[CHAR_BUF_SIZE] = "imu";
static char baro_name[CHAR_BUF_SIZE] = "mavlink_baro";

static size_t num_cams = 0;
static int max_width = 0;
static int max_height = 0;

#define VFT_CMD_START   "start"
#define VFT_CMD_RESTART "restart"
#define VFT_CMD_PAUSE   "pause"

// function prototypes
static void _publish(double vio_dt);
static void _publish_default(double pose_timestamp);
static bool show_extra_points_on_overlay = true;

static int _hard_reset(bool is_locked);
static int connect_client_pipes(void);

static int8_t verbosity_level
{ static_cast<uint8_t>(ov_core::Printer::PrintLevel::SILENT) };

static ext_vio_data_t d;  // complete "extended" vio MPA packet
static vio_data_t s;      // simplified vio packet


std::string log_path = "";

static RingBuffer *img_ringbuf = 	new RingBuffer(20);

std::atomic<bool> thread_update_running;
std::atomic<bool> image_update_running;
boost::posix_time::ptime pT1, pT2, cT1, cT2, zeroTimeOut;
static double ref_zero_alt = -9999.0;
static double baro_alt = 0.;

// printed if some invalid argument was given
static void _print_usage(void)
{
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

static bool _parse_opts(int argc, char *argv[])
{
	static struct option long_options[] =
	{
	{ "config", no_argument, 0, 'c' },
	{ "debug", no_argument, 0, 'd' },
	{ "help", no_argument, 0, 'h' },
	{ "timing-imu", no_argument, 0, 'i' },
	{ "log_path", required_argument, 0, 'l' },
	{ "position", no_argument, 0, 'p' },
	{ "timing-cam", no_argument, 0, 't' },
	{ "verbosity", required_argument, 0, 'v' },
	{ 0, 0, 0, 0 } };

	// set default before we do anything else
//	ov_core::Printer::setPrintLevel(ov_core::Printer::PrintLevel::ALL);
	ov_core::Printer::setPrintLevel(ov_core::Printer::PrintLevel::SILENT);

	while (1)
	{
		int option_index = 0;
		int c = getopt_long(argc, argv, "cdhil:ptv:", long_options,
				&option_index);

		// Detect the end of the options.
		if (c == -1)
		{
			break;
		}

		switch (c)
		{
		case 0:
			// for long args without short equivalent that just set a flag nothing left to do so just break.
			if (long_options[option_index].flag != 0)
				break;
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

// pose data is published from the same thread that does the camera processing
// and pose estimation. That freezes, sometimes for over a second, during blowups
// so this thread exists to keep data coming out during that situation, warning
// consumers that there is an issue.
// this does NOT monitor for blowup criteria, that's done in the camera thread
// as soon as a new pose is calculated. This thread is to warn when that
// camera thread freezes.
static void* _health_thread_func(__attribute__((unused)) void *ctx)
{
	while (main_running)
	{
		usleep(30000);  // run about the same speed as the camera

	}

#ifdef NEED_TO_FIX_PIPE_RESETS
	while (main_running)
	{
		usleep(30000);  // run about the same speed as the camera

		int64_t current_time = _apps_time_monotonic_ns();
		int64_t delay_ns = current_time - last_real_pose_timestamp_ns;

		if (init_failure_detector_reset_flag)
		{
			if (time_of_last_reset != 0)
			{
				uint64_t time_since_reset = current_time - time_of_last_reset;
				if (time_since_reset > INIT_FAILURE_TIMEOUT_NS)
				{
					fprintf(stderr,
							"WARNING failed to init in time, trying again\n");
					_hard_reset(false);
					continue;
				}
			}
			else
				continue;
		}

		// If last packet is recent enough, nothing to worry about.
		// Otherwise, send out failure packets, this inlcudes global error codes
		// indicating if we are waiting for cam or IMU data
		if (delay_ns < STALL_TIMEOUT_NS && last_real_pose_timestamp_ns != 0)
			continue;

		if (!is_initialized)
			continue;

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
		pipe_server_write(EXTENDED_CH, (char*) &d, sizeof(ext_vio_data_t));
		pipe_server_write(SIMPLE_CH, (char*) &s, sizeof(vio_data_t));

		// turn off dropped cam frame code now we have informed everyone.
		global_error_codes &= ~ERROR_CODE_DROPPED_CAM;
	}
#endif

	return NULL;
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


static int _hard_reset(bool is_locked)
{
	return 0;
}

static int _hard_reset_tbd(bool is_locked)
{
	// lock the mutex before calling any ov api calls
	if (!is_locked)
		vio_manager_mutex.lock();
	imu_lock_mutex.lock();

	// stop it if it's running
	if (is_initialized)
	{
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

	T_uncertainty = 0;
	R_uncertainty = 0;

	// now start again
	vio_manager.reset(new ov_msckf::VioManager(vio_manager_options));

	if (vio_manager == NULL)
	{
		fprintf(stderr, "Error creating vio_manager object\n");
		_quit(-1);
	}

	if (!is_locked)
		vio_manager_mutex.unlock();
	imu_lock_mutex.unlock();

	return 0;
}

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
		printf("Client requested hard reset\n");
		_hard_reset(false);  // close and restart the object
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

static void _new_feat_data_default_handler(__attribute__((unused)) int ch,
		char *data, int bytes, __attribute__((unused)) void *context)
{
	//  TBD -- NOT USED currently
	return;
}

// helper callback for cams we are using in the system
static void _cam_helper_cb(__attribute__((unused)) int ch,
		camera_image_metadata_t meta, char *frame, void *context)
{

	is_cam_connected = true;
	// camera working, reset errors
	global_error_codes &= ~ERROR_CODE_CAM_MISSING;

	if (!is_imu_connected)
	{
		s.state = VIO_STATE_INITIALIZING;
		memcpy(&d.v, &s, sizeof(vio_data_t));
		is_initialized = false;
		// send to both pipes
		pipe_server_write(EXTENDED_CH, (char*) &d, sizeof(ext_vio_data_t));
		pipe_server_write(SIMPLE_CH, (char*) &s, sizeof(vio_data_t));
		return;
	}

	if (image_update_running)
		return;

//	if (image_update_running)
//	{
//		pipe_server_write(EXTENDED_CH, (char*) &d, sizeof(ext_vio_data_t));
//		pipe_server_write(SIMPLE_CH, (char*) &s, sizeof(vio_data_t));
////		printf("Cam block\n");
//		return;
//	}
//	int64_t cam_timestamp_ns = meta.timestamp_ns;
//	cam_timestamp_ns += meta.exposure_ns / 2;
//
//	// TODO perhaps just flush the whole buffer instead of only old frames?
//	// this keeps the fifo from overflowing
//	if(cam_timestamp_ns < (_apps_time_monotonic_ns()-1000000000)){
//		global_error_codes |= ERROR_CODE_DROPPED_CAM;
//		for (int i=0; i<cameras_used; i++)
//		{
//			pipe_client_flush(camera_pipe_channels[i]);
//		}
//		fprintf(stderr, "ERROR detected frame older than 1s, flushing cam pipe\n");
//		return;
//	}

	// try to lock to bigger cores if we can
#ifdef BUILD_QRB5165
    _check_and_set_affinity();
#endif

	camera_mode *cm = (camera_mode*) context;

	img_ringbuf_packet *curr_message = new img_ringbuf_packet;

	curr_message->metadata = meta;

	if (meta.format == IMAGE_FORMAT_RAW8)
	{
		memcpy(curr_message->image_pixels, (uint8_t*) frame, meta.size_bytes);
	}
	else if (meta.format == IMAGE_FORMAT_STEREO_RAW8)
	{
		if (*cm == STEREO)
			memcpy(curr_message->image_pixels, (uint8_t*) frame,
					meta.size_bytes);
		else if (*cm == STEREO_LEFT_ONLY)
		{
			memcpy(curr_message->image_pixels, (uint8_t*) frame,
					meta.size_bytes / 2);
			curr_message->metadata.format = IMAGE_FORMAT_RAW8;
			curr_message->metadata.size_bytes /= 2;
		}
		else if (*cm == STEREO_RIGHT_ONLY)
		{
			memcpy(curr_message->image_pixels,
					(uint8_t*) frame + meta.size_bytes / 2,
					meta.size_bytes / 2);
			curr_message->metadata.format = IMAGE_FORMAT_RAW8;
			curr_message->metadata.size_bytes /= 2;
		}
	}
	else if (meta.format == IMAGE_FORMAT_NV12)
	{
		memcpy(curr_message->image_pixels, (uint8_t*) frame,
				meta.width * meta.height);
		curr_message->metadata.format = IMAGE_FORMAT_RAW8;
		curr_message->metadata.size_bytes = meta.width * meta.height;
	}
	else if (meta.format == IMAGE_FORMAT_STEREO_NV12
			|| meta.format == IMAGE_FORMAT_STEREO_NV21)
	{
		if (*cm == STEREO)
		{
			memcpy(curr_message->image_pixels, (uint8_t*) frame,
					meta.width * meta.height);
			memcpy(curr_message->image_pixels + (meta.width * meta.height),
					(uint8_t*) frame + (meta.width * meta.height * 3 / 2),
					meta.width * meta.height);
			curr_message->metadata.format = IMAGE_FORMAT_STEREO_RAW8;
			curr_message->metadata.size_bytes = meta.width * meta.height * 2;
		}
		else if (*cm == STEREO_LEFT_ONLY)
		{
			memcpy(curr_message->image_pixels, (uint8_t*) frame,
					meta.width * meta.height);
			curr_message->metadata.format = IMAGE_FORMAT_RAW8;
			curr_message->metadata.size_bytes = meta.width * meta.height;
		}
		else if (*cm == STEREO_RIGHT_ONLY)
		{
			memcpy(curr_message->image_pixels,
					(uint8_t*) frame + meta.size_bytes / 2,
					meta.width * meta.height);
			curr_message->metadata.format = IMAGE_FORMAT_RAW8;
			curr_message->metadata.size_bytes = meta.width * meta.height;
		}
	}

	img_ringbuf->insert_data(curr_message);

	cv::Mat internal_img(meta.height, meta.width, CV_8UC1);
	std::memcpy(internal_img.data, curr_message->image_pixels, meta.size_bytes);
	ov_core::CameraData message;
	message.timestamp = curr_message->metadata.timestamp_ns * 1e-09;
	message.sensor_ids.push_back(0);
	message.images.push_back(internal_img.clone());
	message.masks.push_back(
			cv::Mat::zeros(internal_img.rows, internal_img.cols, CV_8UC1));
	std::lock_guard<std::mutex> lck(camera_queue_mtx);
	camera_queue.push_back(message);

	// TODO: how to sort between multiple cameras?
	//std::sort(camera_queue.begin(), camera_queue.end());

	delete curr_message;
}


// imu callback registered to the imu server
static void _new_imu_data_default_handler(__attribute__((unused)) int ch,
		char *data, int bytes, __attribute__((unused)) void *context)
{
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

//	if (!is_init)
//	{
//		if (is_init)
//		{
//			for (int i=0; i<cameras_used; i++)
//			{
//				pipe_client_flush(camera_pipe_channels[i]);
//			}
//			pipe_client_flush(IMU_CH);
//			camera_queue.clear();
//
//		}
//	}

	// flag that imu data is active, remove the error, skip data if camera is disconnected
	is_imu_connected = 1;
	global_error_codes &= ~ERROR_CODE_IMU_MISSING;

	if (!is_cam_connected)
	{
		s.state = VIO_STATE_INITIALIZING;
		memcpy(&d.v, &s, sizeof(vio_data_t));
		is_initialized = false;
		// send to both pipes
		pipe_server_write(EXTENDED_CH, (char*) &d, sizeof(ext_vio_data_t));
		pipe_server_write(SIMPLE_CH, (char*) &s, sizeof(vio_data_t));

		return;
	}

	is_init = vio_manager->initialized();

	// time this in debug mode
	int64_t   process_time;

	static bool changed_motion_state = false;
	// TODO current IMU is setup to batch send imu values. This has a conflict with OV's internal processing system
	// by pausing caluclation while the publishing loop runs with new timestamps.
	// So on average use every other packet.
	if (!vio_manager->is_moving())
	{
    	perf_limit  = 4;
	}
	else
	{
		perf_limit = 1;
		if (!changed_motion_state)
		{
			printf("[WARN] Motion detected, going to full processing\n");
			changed_motion_state = true;
		}

	}

	for (int i = 0; i < n_packets; i+=perf_limit)
	{
		// check if we somehow got an out-of-order imu sample and reject it
		if ((int64_t) data_array[i].timestamp_ns <= last_imu_timestamp_ns)
		{
			double dt = (last_imu_timestamp_ns - data_array[i].timestamp_ns)
					* 1e-09;
			fprintf(stderr, "WARNING out-of-order imu %fms before previous\n",
					dt);
			continue;
		}
		else
		{
			// Create the data struct that we will use for ingesting data into the vio manager
			ov_core::ImuData vio_manager_data;
			vio_manager_data.timestamp = data_array[i].timestamp_ns * 1e-09; // (seconds)

			Eigen::Matrix<double, 3, 1> t_wm;
			Eigen::Matrix<double, 3, 1> t_am;
			Eigen::Matrix<double, 3, 1> j_am;

			t_wm(0, 0) = data_array[i].gyro_rad[0];
			t_wm(1, 0) = data_array[i].gyro_rad[1];
			t_wm(2, 0) = data_array[i].gyro_rad[2];

			t_am(0, 0) = data_array[i].accl_ms2[0];
			t_am(1, 0) = data_array[i].accl_ms2[1];
			t_am(2, 0) = data_array[i].accl_ms2[2];

			j_am(0, 0) = data_array[i].accl_ms2[0];
			j_am(1, 0) = data_array[i].accl_ms2[1];
			j_am(2, 0) = fabs(data_array[i].accl_ms2[2])-9.81;

			// NED to FLU systems as per VINS.
			static Eigen::Matrix3d correction_mat = Eigen::Matrix3d::Identity();
			correction_mat(1,1) = -1;
			correction_mat(2,2) = -1;
//		    static Eigen::Matrix3d correction_mat((double*)world_correction.data);

			t_wm = correction_mat * t_wm;
		    t_am = correction_mat * t_am;


		    // FLU
		    vio_manager_data.wm(0, 0) = t_wm(0, 0); // roll
			vio_manager_data.wm(1, 0) = t_wm(1, 0);  // pitch
			vio_manager_data.wm(2, 0) = t_wm(2, 0);  //yaw
			vio_manager_data.am(0, 0) = t_am(0, 0);  // X axis
			vio_manager_data.am(1, 0) = t_am(1, 0);  // Y axis
			vio_manager_data.am(2, 0) = t_am(2, 0);  // Z axis

			vio_manager->feed_measurement_imu(vio_manager_data);
			last_imu_timestamp_ns = data_array[i].timestamp_ns;
			last_imu_time = vio_manager_data.timestamp;

			if (!vio_manager->is_moving() &&
					((boost::posix_time::microsec_clock::local_time() -  zeroTimeOut).total_microseconds() * 1e-6) > 5)
			{
				if ( j_am.norm() > 0.25 && t_wm.norm() > 0.0025)
				{
						image_update_running = false;
				}
				else
				{
					image_update_running = true;
					return;
				}
			}

			  if (thread_update_running)
				return;

			  thread_update_running = true;

			double timestamp_imu_inC = vio_manager_data.timestamp
					- vio_manager->get_state()->_calib_dt_CAMtoIMU->value()(0);

			std::thread thread([&] {

					std::lock_guard<std::mutex> lck(camera_queue_mtx);
		//			image_update_running = true;

					while (!camera_queue.empty()
							&& camera_queue.at(0).timestamp < timestamp_imu_inC)
					{
						vio_manager->feed_measurement_camera(camera_queue.at(0));

						_publish_default(last_imu_time);

						camera_queue.pop_front();
					}
					thread_update_running = false;
			});
			thread.join();

//			image_update_running = false;
/////////////////////////////////////////////////////////////////
//
//			  if (thread_update_running)
//			    return;
//			  thread_update_running = true;
//			  std::thread thread([&] {
//			    // Lock on the queue (prevents new images from appending)
//			    std::lock_guard<std::mutex> lck(camera_queue_mtx);
//
//			      // Loop through our queue and see if we are able to process any of our camera measurements
//			      // We are able to process if we have at least one IMU measurement greater than the camera time
//			      double timestamp_imu_inC = vio_manager_data.timestamp
//			    		  	  	  	  	  	  	  	  	  	  	  	  	  - vio_manager->get_state()->_calib_dt_CAMtoIMU->value()(0);
//
//			      while (!camera_queue.empty() && camera_queue.at(0).timestamp < timestamp_imu_inC) {
//						vio_manager->feed_measurement_camera(camera_queue.at(0));
//
//						_publish_default(last_imu_time);
//
//						camera_queue.pop_front();
//			      }
//			    thread_update_running = false;
//			  });
//
//			  thread.detach();

			  // If we are single threaded, then run single threaded
			  // Otherwise detach this thread so it runs in the background!
//			  if (!_app->get_params().use_multi_threading_subs) {
//			    thread.join();
//			  } else {
//			    thread.detach();
//			  }




// TODO: multi camera
//
//        if (thread_update_running)
//            return;
//        thread_update_running = true;
//        std::thread thread([&] {
//            // Lock on the queue (prevents new images from appending)
//            std::lock_guard<std::mutex> lck(camera_queue_mtx);
//
//    		double timestamp_imu_inC = vio_manager_data.timestamp - vio_manager->get_state()->_calib_dt_CAMtoIMU->value()(0);
//    		while (!camera_queue.empty() && camera_queue.at(0).timestamp < timestamp_imu_inC) {
//    			auto rT0_1 = boost::posix_time::microsec_clock::local_time();
//    			double update_dt = 100.0 * (timestamp_imu_inC - camera_queue.at(0).timestamp);
//    			vio_manager->feed_measurement_camera(camera_queue.at(0));
//				_publish_default(timestamp_imu_inC);
//    			camera_queue.pop_front();
//    			auto rT0_2 = boost::posix_time::microsec_clock::local_time();
//    			double time_total = (rT0_2 - rT0_1).total_microseconds() * 1e-6;
//    			//printf("[TIME]: %.4f seconds total (%.1f hz, %.2f ms behind)\n"  time_total, 1.0 / time_total, update_dt);
//    		}
//
//    		thread_update_running = false;
//        });
//
//        thread.join();
//
//		static long last_cam_time = 0;
//		long now_time = _apps_time_monotonic_ns() ;
//		if (last_cam_time> 0)
//		{
//			double cam_dt = now_time - last_cam_time;
//			printf("imu frame time: %f %f %d\n",  cam_dt, 1/(cam_dt*1e-9), i);
//		}
//		last_cam_time = now_time;
		}
	}

	return;
}


// return 0 if all is well, otherwise return the reason for blowup
static int _check_for_blowup(std::shared_ptr<ov_msckf::State> current_state,
		Eigen::Matrix<double, 12, 12> cov_plus, int good_features)
{
	int64_t current_ts = current_state->_timestamp * 1e9;
	static int64_t last_time_with_good_cov = 0;
	static int64_t last_time_with_enough_features = 0;

	// reset timers to current time after reset so we don't trip this during init
	if (hard_reset_blowup_flag)
	{
		last_time_with_enough_features = current_ts;
		last_time_with_good_cov = current_ts;
		hard_reset_blowup_flag = 0;
	}

	// Now go through our 4 blowup criteria
	// max velocity check  current_state->_imu->vel().cast<float>()
	float vel = sqrtf(
			(current_state->_imu->vel_fej()(0)
					* current_state->_imu->vel_fej()(0))
					+ (current_state->_imu->vel_fej()(1)
							* current_state->_imu->vel_fej()(1))
					+ (current_state->_imu->vel_fej()(2)
							* current_state->_imu->vel_fej()(2)));
	if (vel > auto_reset_max_velocity)
	{
		fprintf(stderr,
				"WARNING auto-resetting due to exceeding max velocity of %4.1fm/s\n",
				(double) auto_reset_max_velocity);
		return ERROR_CODE_VEL_INST_CERT;
	}

	// get max velocity covariance
	double cov = cov_plus(6, 6);
	if (cov_plus(7, 7) > cov)
		cov = cov_plus(7, 7);
	if (cov_plus(8, 8) > cov)
		cov = cov_plus(8, 8);

	// min feature timeout check
	if (good_features > auto_reset_min_features)
	{
		last_time_with_enough_features = current_ts;
	}
	else
	{
		float tmp = (float) (current_ts - last_time_with_enough_features)
				/ 1000000000.0f;
		if (tmp > auto_reset_min_feature_timeout_s)
		{
			fprintf(stderr,
					"WARNING auto-resetting due to low feature count\n");
			fprintf(stderr, "feats: %d, limit: %d\n", good_features,
					auto_reset_min_features);
			return ERROR_CODE_LOW_FEATURES;
		}
	}

	// max v cov timeout check
	if (cov < auto_reset_max_v_cov)
	{
		last_time_with_good_cov = current_ts;
	}
	else
	{
		float tmp = (current_ts - last_time_with_good_cov) / 1000000000.0f;
		if (tmp > auto_reset_max_v_cov_timeout_s)
		{
			fprintf(stderr,
					"WARNING auto-resetting due to high vel covariance\n");
			fprintf(stderr, "vel cov: %6.5f, limit: %6.5f\n", cov,
					auto_reset_max_v_cov);
			return ERROR_CODE_VEL_WINDOW_CERT;
		}
	}

	// check for instant vel covariance limit
	if (cov > auto_reset_max_v_cov_instant)
	{
		fprintf(stderr,
				"WARNING auto-resetting due to vel covariance instant limit\n");
		fprintf(stderr, "vel cov: %6.5f, limit: %6.5f\n", cov,
				auto_reset_max_v_cov_instant);
		return ERROR_CODE_VEL_INST_CERT;
	}

	// all is good (for now)
	return 0;
}

static void _publish_default(double pose_timestamp)
{

	static int skip_cnt = 0;
	static int target_id = -1;

	std::shared_ptr<ov_msckf::State> current_state = vio_manager->get_state(); // contains a few extra pieces we need
	Eigen::Matrix<double, 12, 12> cov_plus =
			Eigen::Matrix<double, 12, 12>::Zero();  // covariance!!!!

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

	// record that we just got a successful pose and point cloud
	last_real_pose_timestamp_ns = static_cast<int64_t>(pose_timestamp * 1e9);

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

#ifdef TRUE_MONTE

	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covariance_posori);
	Eigen::MatrixXd transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
	static std::mt19937 gen{ std::random_device{}() };
	static std::normal_distribution<> dist;
	Eigen::VectorXd uncertain = transform * Eigen::VectorXd{ 6 }.unaryExpr([&](auto x) { return dist(gen); });
	Eigen::Vector3d  T_sigma = uncertain.segment(0,2);
	Eigen::Vector3d  R_sigma = uncertain.segment(3,5);
	T_uncertainty = (T_uncertainty * 0.6) + (0.4 * (sqrt((T_sigma.array() - T_sigma.mean()).square().sum() / (T_sigma.size() - 1))));
	R_uncertainty = (R_uncertainty * 0.6) + (0.4* (sqrt((R_sigma.array() - R_sigma.mean()).square().sum() / (R_sigma.size() - 1))));
	printf("Uncertainty in the robot's pose: xyz: %f R:%f\n", T_uncertainty, R_uncertainty);
	printf("Get the uncertainty in the robot's pose: (%dx%d) %f\n", uncertain.rows(), uncertain.cols(), std_dev);
	printf("(%d): ", (int)cov_varis.size());
	for (int i=0; i<cov_varis.size(); i++)
	{
		printf("%f ", cov_varis(i));
	}
	printf("\n");

#endif
	// correction matrix
	static Eigen::Matrix3d flu_ned_correction_mat = Eigen::Matrix3d::Identity();
	flu_ned_correction_mat(1, 1) = -1;
	flu_ned_correction_mat(2, 2) = -1;

	// get features
	// this function will give us back as much info as available for features in various stages of the overall state
	std::vector<output_feature> curr_pixel_locs =
			vio_manager->get_pixel_loc_features();

	for (size_t d = 0; d < curr_pixel_locs.size(); d++)
	{
		if (curr_pixel_locs[d].pix_loc[0] > 0.0f
				&& curr_pixel_locs[d].pix_loc[1] > 0.0f)
		{
			if (curr_pixel_locs[d].point_quality == OV_HIGH)
			{
				// USED points
				n_good_points++;
				// if we aren't over our max feature count and we have a 3d estimate, correct it to our real world
				if (n_good_points <= VIO_MAX_REPORTED_FEATURES)
				{
					double d_tsf[3] =
					{ curr_pixel_locs[d].tsf[0], curr_pixel_locs[d].tsf[1],
							curr_pixel_locs[d].tsf[2] };

					Eigen::Matrix<double, 3, 1> curr_feat_holder(d_tsf);
#ifdef MAYNEED
					curr_feat_holder = world_correction_mat * curr_feat_holder;
#endif

					// replaces with corrected values
					Eigen::MatrixXf::Map(curr_pixel_locs[d].tsf, 3, 1) =
							curr_feat_holder.cast<float>();
				}
			}
			else if (curr_pixel_locs[d].point_quality == OV_MEDIUM)
			{
				// UNUSED points
				n_oos_points++;
			}
		}
	}

	// check if its initialized or not
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

	if (current_state->error_flag == VIO_STATE_FAILED)
	{
		fprintf(stderr,
				"WARNING auto-resetting, EKF starved of good features for too long\n");
		_hard_reset(true);
		s.state = VIO_STATE_FAILED;
	}

	// we finished initializing, no longer check for init timeout
	if (s.state == VIO_STATE_OK)
	{
		init_failure_detector_reset_flag = 0;
	}

	// if we just went from good to failed, treat this like a reset for the
	// init failure detector so it can timeout the same as VIO tries to re-init
	// itself after it's own internal reset
	if (last_state != VIO_STATE_FAILED && s.state == VIO_STATE_FAILED)
	{
		init_failure_detector_reset_flag = 1;
		time_of_last_reset = static_cast<int64_t>(current_state->_timestamp
				* 1e9);
	}

	// record time when vio claimed to have initialized and only check for
	// for blowups some time after this
	if (last_state != VIO_STATE_OK && s.state == VIO_STATE_OK)
	{
		//blowup_detector_flag = 1;
		time_of_first_okay = static_cast<int64_t>(current_state->_timestamp
				* 1e9);
	}
	last_state = s.state;

	// while VIO state is OK, do our own additional blowup checks if enough
	// time has passed since the init
	int64_t time_since_first_okay = _apps_time_monotonic_ns()
			- time_of_first_okay;
	if (blowup_detector_flag && time_since_first_okay > BLOWUP_DETECT_TIMEOUT_NS)
	{
		int code = _check_for_blowup(current_state, cov_plus, n_good_points);
		if (code)
		{
			_hard_reset(true);
			s.state = VIO_STATE_FAILED;
			s.error_code |= code;
		}
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

	Eigen::Matrix<double, 3, 1> imu_wrt_wio_holder = current_state->_imu->pos();
	if (gravity_vector_direction == 1)
	{
		imu_wrt_wio_holder = flu_ned_correction_mat * imu_wrt_wio_holder;
	}

	//    std::cout << "POS:\n" << imu_wrt_wio_holder << std::endl;

	Eigen::MatrixXf::Map(s.T_imu_wrt_vio, 3, 1) =
			imu_wrt_wio_holder.cast<float>();
	Eigen::Matrix<double, 3, 1> vel_imu_wrt_vio_holder =
			current_state->_imu->vel();
	if (gravity_vector_direction == 1)
	{
		vel_imu_wrt_vio_holder = flu_ned_correction_mat
				* vel_imu_wrt_vio_holder;
	}
	Eigen::MatrixXf::Map(s.vel_imu_wrt_vio, 3, 1) = vel_imu_wrt_vio_holder.cast<
			float>();

	Eigen::Matrix3d final_out = current_state->_imu->Rot_fej();
	if (gravity_vector_direction == -1)
	{
		final_out = flu_ned_correction_mat * final_out;
	}
	else
	{
		final_out = flu_ned_correction_mat.transpose() * final_out
				* flu_ned_correction_mat;
	}
	Eigen::MatrixXf::Map(reinterpret_cast<float*>(s.R_imu_to_vio), 3, 3) =
			final_out.cast<float>();

	// camera position here is a bit funky, since open vins outputs imu to cam and we want cam to imu
	Eigen::Matrix3d cam_out = ov_core::quat_2_Rot(
			current_state->_calib_IMUtoCAM[0]->quat()).transpose();
	if (gravity_vector_direction == -1)
		cam_out = flu_ned_correction_mat * cam_out;
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

	// since open vins does the gravity alignment internally, gravity vec is always 0,0,1 and cov is 0'd out BUT
	// voxl flips it to actual
	float grav_vec[3] =
	{ 0, 0, (float) gravity_vector_direction };
	memcpy(s.gravity_vector, grav_vec, sizeof(float) * 3);

	// limit the number of features to what fits in our pipe packet
	d.n_total_features = (int) curr_pixel_locs.size();
	if (d.n_total_features > VIO_MAX_REPORTED_FEATURES)
	{
		d.n_total_features = VIO_MAX_REPORTED_FEATURES;
	}

	memcpy(d.features, curr_pixel_locs.data(),
			d.n_total_features * sizeof(vio_feature_t));

	s.quality = calc_quality(s.state, s.velocity_covariance,
			vel_imu_wrt_vio_holder.norm(), max_width, max_height,
			d.n_total_features, cam_info_vec.size(), d.features);

	// fill in simplified struct inside the extended packet
	memcpy(&d.v, &s, sizeof(vio_data_t));

	pipe_server_write(EXTENDED_CH, (char*) &d, sizeof(ext_vio_data_t));
	usleep(2);
	pipe_server_write(SIMPLE_CH, (char*) &s, sizeof(vio_data_t));
	usleep(2);

	// for debug only
	if (en_debug)
	{
		printf("state: ");
		pipe_print_vio_state(s.state);
		printf(" err: ");
		pipe_print_vio_error(s.error_code);
		printf("\n");
	}
	if (en_debug_pos)
	{
		printf("%6.3f %6.3f %6.3f ", (double) s.T_imu_wrt_vio[0],
				(double) s.T_imu_wrt_vio[1], (double) s.T_imu_wrt_vio[2]);
		printf("\n");
	}

	// turn off dropped frame code now we have informed everyone.
	global_error_codes &= ~ERROR_CODE_DROPPED_CAM;

	// if someone has subscribed to the overlay, draw it
	if (pipe_server_get_num_clients(OVERLAY_CH) > 0)
	{
		if (every_other++ % 3 == 0)  // 10fps
		{
			cv::Mat overlay_cp;

			img_ringbuf_packet *curr_imgs = new img_ringbuf_packet;

			feat_ts_mutex.lock();
			// TODO May not exists, then what
			double target_time = pose_timestamp*1e9;
			//int ret = img_ringbuf->get_data_at_time((int64_t)target_time, curr_imgs);
			int ret = img_ringbuf->get_data_at_position(0, curr_imgs);

			if (ret < 0)
			{
				feat_ts_mutex.unlock();
				fprintf(stderr, "FAILED TO FETCH IMG RINGBUF at time %f %ld %ld %ld %ld %ld\n",
						target_time,
						img_ringbuf->get_timestamp_at_position(0),
						img_ringbuf->get_timestamp_at_position(1),
						img_ringbuf->get_timestamp_at_position(2),
						img_ringbuf->get_timestamp_at_position(3),
						img_ringbuf->get_timestamp_at_position(4));
				return;
			}

			// now, construct some cv::Mats with our images (we know them to be greyscale)
			std::vector<cv::Mat> img_set;
			if (curr_imgs->metadata.format == IMAGE_FORMAT_STEREO_RAW8)
			{
				cv::Mat img(curr_imgs->metadata.height,
						curr_imgs->metadata.width,
						CV_8UC1, curr_imgs->image_pixels);
				cv::Mat img2(curr_imgs->metadata.height,
						curr_imgs->metadata.width,
						CV_8UC1,
						curr_imgs->image_pixels
								+ (curr_imgs->metadata.width
										* curr_imgs->metadata.height));
				img_set.push_back(img);
				img_set.push_back(img2);
			}
			else
			{
				cv::Mat img(curr_imgs->metadata.height,
						curr_imgs->metadata.width,
						CV_8UC1, curr_imgs->image_pixels);
				img_set.push_back(img);
			}

			feat_ts_mutex.unlock();

			for (size_t i = 0; i < curr_pixel_locs.size(); i++)
			{
				// re-identified slam landmark
				if (curr_pixel_locs[i].point_quality == OV_RE_HIGH)
				{
					cv::drawMarker(img_set[curr_pixel_locs[i].cam_id],
							cv::Point2f(curr_pixel_locs[i].pix_loc[0],
									curr_pixel_locs[i].pix_loc[1]),
							cv::Scalar(255), cv::MARKER_SQUARE, 8, 2);
				}
				// slam landmark
				else if (curr_pixel_locs[i].point_quality == OV_HIGH)
				{
					cv::drawMarker(img_set[curr_pixel_locs[i].cam_id],
							cv::Point2f(curr_pixel_locs[i].pix_loc[0],
									curr_pixel_locs[i].pix_loc[1]),
							cv::Scalar(255), cv::MARKER_SQUARE, 8, 2);
				}
				// tracked feature
				else if (curr_pixel_locs[i].point_quality == OV_MEDIUM)
				{
					cv::drawMarker(img_set[curr_pixel_locs[i].cam_id],
							cv::Point2f(curr_pixel_locs[i].pix_loc[0],
									curr_pixel_locs[i].pix_loc[1]),
							cv::Scalar(145), cv::MARKER_SQUARE, 8, 2);
				}
				else if (show_extra_points_on_overlay)
				{
					cv::drawMarker(img_set[curr_pixel_locs[i].cam_id],
							cv::Point2f(curr_pixel_locs[i].pix_loc[0],
									curr_pixel_locs[i].pix_loc[1]),
							cv::Scalar(0), cv::MARKER_SQUARE, 8, 2);
				}
			}

			cv::resize(img_set[0], img_set[0], cv::Size(640, 480));

			if (img_set.size() == 1)
				overlay_cp = img_set[0];
			else
			{
				cv::resize(img_set[1], img_set[1], cv::Size(640, 480));
				cv::hconcat(img_set[0], img_set[1], overlay_cp);
			}

			static float font_size = 0.8;
			static float border_scale = 1;
			if (overlay_cp.cols <= 640)
			{
				font_size = 0.55;
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
			sprintf(str, "CEP: %3.3f R_err: %3.2f   XYZ: %6.2lf %6.2lf %6.2lf",
					T_uncertainty, R_uncertainty * 180 / M_PI,
					(double) s.T_imu_wrt_vio[0], (double) s.T_imu_wrt_vio[1],
					(double) s.T_imu_wrt_vio[2]);
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
				sprintf(oos_pts_string, "#pts: %2d  (%2d)", n_good_points,
						n_oos_points);
			else
				oos_pts_string[0] = 0;

			sprintf(str, "ex(ms): %6.1f Gain: %5d Q: %3d %s ",
					draw_meta.exposure_ns / 1000000.0, draw_meta.gain,
					s.quality, oos_pts_string);

			font_size = 0.6;
			cv::putText(
					overlay_cp, //target image
					str, //text
					cv::Point((overlay_cp.cols - text_size.width) / 2,
							overlay_cp.rows - text_size.height / 2 + 2), //top-left position
					cv::FONT_HERSHEY_COMPLEX, font_size,
					cv::Scalar(255, 255, 255), //font color
					font_size, cv::LINE_AA);

			// draw out to pipe
			pipe_server_write_camera_frame(OVERLAY_CH, draw_meta,
					(char*) overlay_cp.data);
			usleep(2);

			delete curr_imgs;
		}
	}

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
	vio_manager_options.init_options.init_max_disparity = 10000;
	vio_manager_options.init_options.init_dyn_use = false;

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

	vio_manager_options.downsample_cameras = false; // TBD
	vio_manager_options.num_opencv_threads = 6;
	vio_manager_options.num_pts = num_features_to_track;
	vio_manager_options.fast_threshold = fast_threshold;
	vio_manager_options.min_px_dist = min_pix_dist;
	vio_manager_options.histogram_method = histogram_method;
	vio_manager_options.knn_ratio = knn_ratio;
	vio_manager_options.track_frequency = track_frequency;

	/// CAMERA INTRINSICS + EXTRINSICS ///

	std::lock_guard<std::mutex> lg(vio_manager_mutex);

	std::shared_ptr<ov_core::CamBase> cam_calib_intrinsic;
	for (size_t i = 0; i < cam_info_vec.size(); i++)
	{
		// ov uses camequi model to represent fisheye cameras, and the radtan model for standard lenses
		if (cam_info_vec[i].is_fisheye)
		{
			std::cout << "OpenVINS using fisheye camera" << std::endl;

			cam_calib_intrinsic = std::make_shared<ov_core::CamEqui>(
					cam_info_vec[i].cam_calib_intrinsic(8, 0),
					cam_info_vec[i].cam_calib_intrinsic(9, 0));
		}
		else
		{
			cam_calib_intrinsic = std::make_shared<ov_core::CamRadtan>(
					cam_info_vec[i].cam_calib_intrinsic(8, 0),
					cam_info_vec[i].cam_calib_intrinsic(9, 0));
		}
		// The camera intrinsics
		cam_calib_intrinsic->set_value(cam_info_vec[i].cam_calib_intrinsic);
		vio_manager_options.camera_intrinsics[i] = cam_calib_intrinsic;
		vio_manager_options.camera_extrinsics[i] = cam_info_vec[i].cam_wrt_imu;

		std::cout << "OpenVINS FINAL Cam extrinsics to IMU: "
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

	return vio_manager_options;
}

static void _feat_disconnect_cb(__attribute__((unused)) int ch,
		__attribute__((unused)) void *context)
{
	fprintf(stderr,
			"WARNING: disconnected from feature server, resetting VIO\n");

	std::lock_guard<std::mutex> lg(vio_manager_mutex);
	global_error_codes |= ERROR_CODE_CAM_MISSING;
	is_cam_connected = 0;
	ov_msckf::VioManagerOptions vio_manager_options =
			generate_open_vins_manager_options();
	// HARD RESET
	_hard_reset(false);
	return;
}

static void _imu_disconnect_cb(__attribute__((unused)) int ch,
		__attribute__((unused)) void *context)
{
//	fprintf(stderr,
//			"****** WARNING: disconnected from imu server, resetting VIO\n");
//	global_error_codes |= ERROR_CODE_IMU_MISSING;
//	last_imu_timestamp_ns = 0;
//	is_imu_connected = 0;
//	std::lock_guard<std::mutex> lg(vio_manager_mutex);
//	ov_msckf::VioManagerOptions vio_manager_options =
//			generate_open_vins_manager_options();
//	// HARD RESET
//	_hard_reset(false);
	return;
}


static void _baro_disconnect_cb(__attribute__((unused)) int ch,
		__attribute__((unused)) void *context)
{
	return;
}

static void _new_baro_data_default_handler(__attribute__((unused)) int ch,
		char *data, int bytes, __attribute__((unused)) void *context)
{
		mavlink_message_t* baro_msg = (mavlink_message_t*) data;
	    // basic sanity checks
	    if(bytes<0){
	        fprintf(stderr, "ERROR validating BARO data received through pipe: number of bytes = %d\n", bytes);
	        return;
	    }
	    if(data==NULL){
	        fprintf(stderr, "ERROR validating BARO data received through pipe: got NULL data pointer\n");
	        return;
	    }

		uint32_t baro_time_ms = mavlink_msg_scaled_pressure_get_time_boot_ms(baro_msg);
		float pressure = mavlink_msg_scaled_pressure_get_press_abs(baro_msg);

		if (ref_zero_alt == -9999)
			ref_zero_alt = (pressure * 0.750062);

		baro_alt = (pressure * 0.750062) - ref_zero_alt;

		//printf("Got Baro data %d %f meters\n", baro_time_ms, baro_alt);
}

static int create_server_pipes(void)
{
	int flags = SERVER_FLAG_EN_CONTROL_PIPE;

	// init extended pipe
	pipe_info_t info1 =
	{
	OV_VIO_EXTENDED_NAME,       // name
			OV_VIO_EXTENDED_LOCATION,   // location
			"ext_vio_data_t",           // type
			PROCESS_NAME,               // server_name
			CAM_PIPE_SIZE,  // size_bytes
			0                           // server_pid
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
	pipe_info_t info2 =
	{
	OV_VIO_SIMPLE_NAME,         // name
			OV_VIO_SIMPLE_LOCATION,     // location
			"vio_data_t",               // type
			PROCESS_NAME,               // server_name
			VIO_RECOMMENDED_PIPE_SIZE*2,  // size_bytes
			0                           // server_pid
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
	pipe_info_t info3 =
	{
	OV_VIO_OVERLAY_NAME,        // name
			OV_VIO_OVERLAY_LOCATION,    // location
			"camera_image_metadata_t",  // type
			PROCESS_NAME,               // server_name
			CAM_PIPE_SIZE,              // size_bytes
			0                           // server_pid
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

	return 0;
}

static int read_external_configs_from_file(void)
{

	fprintf(stderr,
			"=====> Using internal KLT Feature Tracking and File base camera configuration: %s\n", imu_name);

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

	free(cam_json);

	return 0;
}

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
	printf("Number of Cameras active: %d\n", cameras_used);
	for (int i = 0; i < cameras_used; i++)
	{
		int ch = pipe_client_get_next_available_channel();
		camera_pipe_channels[i] = ch;
		// pipe_client_set_disconnect_cb(FEAT_OVERLAY_CH, _imu_disconnect_cb, NULL);
		pipe_client_set_camera_helper_cb(ch, _cam_helper_cb,
				&cam_info_vec[i].mode);
		flags = CLIENT_FLAG_EN_CAMERA_HELPER;
		int ret = pipe_client_open(ch, cam_info_vec[i].name, PROCESS_NAME,
				flags, 1280 * 800 * 15);
		if (ret)
		{
			fprintf(stderr, "failed to open %s\n", cam_info_vec[i].name);
			return -1;
		}
	}

	printf("imu pipe name: %s\n", imu_name);


	// connect to baro
	pipe_client_set_disconnect_cb(BARO_CH, _baro_disconnect_cb, NULL);
	pipe_client_set_simple_helper_cb(BARO_CH, _new_baro_data_default_handler,
			NULL);
	flags = CLIENT_FLAG_EN_SIMPLE_HELPER;
	if (pipe_client_open(BARO_CH, baro_name, PROCESS_NAME, flags,
			IMU_RECOMMENDED_READ_BUF_SIZE) != 0)
	{
		fprintf(stderr, "failed to open Baro\n");
		return -1;
	}

	return 0;
}


int main(int argc, char *argv[])
{
	// Parse the command line options and terminate if the parser says we should terminate
	if (_parse_opts(argc, argv))
	{
		return -1;
	}

	// Load the config files
	printf("Loading our own config file\n");
	if (config_file_read() < 0)
		_quit(-1);
	config_file_print();

	// read camera multicam setup and configs
	if (cam_config_file_read() < 0)
	{
		fprintf(stderr, "ERROR %d\n", cam_config_file_read());
		_quit(-1);
	}
	cam_config_file_print();

	// load external info
	printf("Loading external config file\n");
	if (read_external_configs_from_file() < 0)
		_quit(-1);

	// Create the VIO Manager -- Core OpenVINS state
	vio_manager_options = generate_open_vins_manager_options();

	 if ((world_correction.at<double>(0,0) * world_correction.at<double>(1,1)) > 0)
		  gravity_vector_direction = 1;
	 else
		  gravity_vector_direction = -1;

	vio_manager = std::unique_ptr<ov_msckf::VioManager>(
			new ov_msckf::VioManager(vio_manager_options));

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
		param.sched_priority = 95;
		fprintf(stderr, "setting scheduler\n");
		int ret = sched_setscheduler(0, SCHED_FIFO, &param);
		if(ret==-1){
			fprintf(stderr, "WARNING Failed to set priority, errno = %d\n", errno);
			fprintf(stderr, "This seems to be a problem with ADB, the scheduler\n");
			fprintf(stderr, "should work properly when this is a background process\n");
		}
		// check
		ret = sched_getscheduler(0);
		if(ret!=SCHED_FIFO){
			fprintf(stderr, "WARNING: failed to set scheduler\n");
		}
		else{
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
		_quit(0);

	/////////////////////////////////////////////////////////////////////////////////////////////
	// temp disable
	// open vins will occasionally stall out on init, and the health monitor
	// will then cause vvpx4 to switch out of position control
	// comment the below thread creation if this behavior occurs
	/////////////////////////////////////////////////////////////////////////////////////////////

	pthread_attr_t tattr;
	pthread_attr_init(&tattr);
	pthread_create(&health_thread, &tattr, _health_thread_func, NULL);


	// until they connect, inidcate that they are disconnected
	global_error_codes |= ERROR_CODE_CAM_MISSING;
	global_error_codes |= ERROR_CODE_IMU_MISSING;

	perf_limit  = 1;

	// Connect to the client pipes and start getting data
	if (connect_client_pipes() < 0)
		_quit(0);

	// Set the main running flag to 1 to indicate that we are running
	main_running = 1;

	zeroTimeOut = boost::posix_time::microsec_clock::local_time() ;
	// run until start/stop module catches a signal and changes main_running to 0
	while (main_running)
	{
		usleep(5000000);
	}

	pthread_join(health_thread, NULL);

	// Shutdown Nicely
	_quit(0);

	printf("Quiting VIO server\n");
	return 0;
}
