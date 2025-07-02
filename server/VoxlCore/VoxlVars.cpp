/**
 * @file VoxlVars.cpp
 * @brief Global variable definitions for VOXL OpenVINS server
 * @author Zauberflote
 * @date 2025
 * @version 1.0
 *
 * This file contains the definitions of all global variables declared in VoxlVars.h.
 * It provides the actual storage for system state, configuration parameters,
 * and operational variables used throughout the VIO system.
 */

#include "VoxlVars.h"
#include "VoxlCommon.h"
#include <atomic>
#include <memory>
#include <mutex>
#include <vector>
#include <string>

// ============================================================================
// CORE VIO MANAGER
// ============================================================================

/** @brief Main VIO manager instance */
std::unique_ptr<ov_msckf::VioManager> vio_manager;

// ============================================================================
// ATOMIC STATE VARIABLES
// ============================================================================

/** @brief Main process running flag */
volatile int main_running = 1;

/** @brief Current VIO state (initializing, tracking, etc.) */
std::atomic<uint8_t> vio_state(0);

/** @brief VIO error codes */
std::atomic<uint32_t> vio_error_codes(0);

/** @brief Flag indicating if system is currently resetting */
std::atomic<bool> is_resetting(false);

/** @brief Flag indicating if system is armed */
std::atomic<bool> is_armed(false);

/** @brief Flag indicating if camera is connected */
std::atomic<bool> is_cam_connected(false);

/** @brief Flag indicating if IMU is connected */
std::atomic<bool> is_imu_connected(false);

// ============================================================================
// SERVER CONFIGURATION VARIABLES
// ============================================================================

/** @brief Enable automatic reset functionality */
int en_auto_reset = 0;

/** @brief Maximum velocity threshold for auto reset */
float auto_reset_max_velocity = 0.0f;

/** @brief Maximum velocity covariance for instant reset */
float auto_reset_max_v_cov_instant = 0.0f;

/** @brief Maximum velocity covariance for timeout reset */
float auto_reset_max_v_cov = 0.0f;

/** @brief Timeout duration for velocity covariance reset (seconds) */
float auto_reset_max_v_cov_timeout_s = 0.0f;

/** @brief Minimum number of features for auto reset */
int auto_reset_min_features = 0;

/** @brief Minimum feature timeout for auto reset (seconds) */
float auto_reset_min_feature_timeout_s = 0.0f;

/** @brief Auto fallback timeout (seconds) */
float auto_fallback_timeout_s = 0.0f;

/** @brief Minimum velocity for auto fallback */
float auto_fallback_min_v = 0.0f;

/** @brief Enable continuous yaw checks */
bool en_cont_yaw_checks = false;

/** @brief Fast yaw threshold */
float fast_yaw_thresh = 0.0f;

/** @brief Fast yaw timeout (seconds) */
float fast_yaw_timeout_s = 0.0f;

/** @brief Base folder for yaml configuration files */
char folder_base[CHAR_BUF_SIZE] = "/etc/modalai/VoxlConfig/starling2";  

/** @brief Enable debug output */
int en_debug = 0;

// ============================================================================
// SENSOR CONFIGURATION VARIABLES
// ============================================================================

/** @brief IMU device name */
char imu_name[64];

/** @brief Vector of camera configuration information */
std::vector<cam_info> cam_info_vec;

// ============================================================================
// IMU-SPECIFIC VARIABLES
// ============================================================================

/** @brief Timestamp of last IMU data (nanoseconds) */
volatile int64_t last_imu_timestamp_ns = 0;

// ============================================================================
// CAMERA-SPECIFIC VARIABLES
// ============================================================================

/** @brief Takeoff camera identifier */
int takeoff_cam = 0;

/** @brief Enable takeoff camera functionality */
int en_takeoff_cam = 0;

/** @brief Vector of takeoff camera identifiers */
std::vector<int> takeoff_cams;

/** @brief Timestamp of last camera data (nanoseconds) */
volatile int64_t last_cam_time = 0;

/** @brief Number of cameras currently in use */
int cameras_used = 0;

// ============================================================================
// PIPE COMMUNICATION VARIABLES
// ============================================================================

/** @brief Camera pipe channels array */
int camera_pipe_channels[MAX_CAM_CNT] = {0};

// ============================================================================
// IMAGE PROCESSING VARIABLES
// ============================================================================

/** @brief Image ring buffer for processing */
voxl::img_ringbuf_packet *img_ringbuf = nullptr;
