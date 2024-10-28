#ifndef SHARED_VARS_H
#define SHARED_VARS_H

#include <core/VioManager.h>
#include <state/Propagator.h>
#include <state/State.h>
#include <state/StateHelper.h>
#include <modal_pipe.h>
#include <atomic>

#include "img_ringbuffer.h"
#include "config_file.h"


// server channels
#define EXTENDED_CH 0
#define SIMPLE_CH 1
#define OVERLAY_CH 2
#define OVSTATUS_CH 3

// client channels and config
#define IMU_CH 0
#define FEATURE_CH 8
#define FEAT_OVERLAY_CH 9
#define BARO_CH 10
#define CPU_CH 11

#define PROCESS_NAME "voxl-open-vins-server"
#define FEATURE_NAME "tracked_feats"
#define FEATURE_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR FEATURE_NAME "/"
#define FEATURE_OVERLAY_NAME "feat_overlay"
#define FEATURE_OVERLAY_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR FEATURE_OVERLAY_NAME "/"

extern int en_debug;
extern int en_debug_pos;
extern int en_debug_timing_cam;
extern int offline;
extern bool bypass_reset_checks;
extern bool is_thermal;
extern bool pause_qmin;
extern bool zero_horizon;
extern uint16_t resume_processing;
extern bool overlay_client_connected;

extern double last_imu_time;
extern int64_t last_cam_time;

extern std::unique_ptr<ov_msckf::VioManager> vio_manager;

extern volatile float alt_z;
extern volatile bool changed_motion_state;
extern volatile bool imu_moved;
extern volatile int throttle_state;

extern volatile bool is_armed;
extern volatile bool is_resetting;
extern volatile bool use_takeoff_cam;
extern volatile bool has_idle_images;
extern volatile bool ext_blind_take_off_force;
extern volatile bool check_for_stable_vins;

extern volatile int64_t last_imu_timestamp_ns;
extern volatile int64_t last_feat_timestamp_ns;

extern volatile int64_t last_real_pose_timestamp_ns;
extern volatile int64_t last_sent_timestamp_ns;
extern double last_good_feat_ts;
extern double current_reset_max_velocity;

extern volatile int is_imu_connected;
extern volatile int is_cam_connected;
extern volatile int is_init;
extern volatile int gravity_axis;
extern volatile int perf_limit;
extern int cameras_used;
extern int camera_pipe_channels[10];
extern int gravity_vector_direction;

extern volatile int init_failure_detector_reset_flag;

extern Eigen::Matrix3d flu_ned_correction_mat;
extern Eigen::Matrix<double, 4, 1> default_level_horizon; 

extern bool pause_cam_states[6];


extern std::mutex publish_lock_mutex;
extern std::mutex cam_lock_mutex;

extern volatile int is_initialized;
extern volatile int idler_limit;
extern int64_t last_time_alignment_ns;
extern int32_t last_frame_frame_id;
extern int64_t last_frame_timestamp_ns;

extern cv::Mat idle_image1;
extern cv::Mat idle_image2;
extern cv::Mat idle_image3;

extern uint32_t global_error_codes;

extern int current_width;
extern int current_height;
extern uint8_t update_slots;

extern void _publish_default(double pose_timestamp);

extern ext_vio_data_t d;  // complete "extended" vio MPA packet
extern vio_data_t s;      // simplified vio packet

extern uint8_t slot_image_pixels[MAX_IMAGE_SIZE];  
extern RingBuffer *img_ringbuf;

extern std::deque<ov_core::CameraData> camera_queue;
extern std::mutex camera_queue_mtx;

extern cv::Mat world_correction;
extern Eigen::Matrix3d world_correction_eigen;
extern Eigen::Matrix<double, 4, 1> world_correction_q;

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

extern display_bucket_t display_snapshot;


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

extern ov_status_t ov_status;


extern double baro_alt;
extern double d_baro;

extern std::atomic<bool> thread_update_running;
extern std::atomic<bool> image_update_running;



#endif