/**
 * @file VoxlVars.h
 * @brief Global variable declarations and constants for VOXL OpenVINS server
 * @author Zauberflote
 * @date 2025
 * @version 1.0
 *
 * This header file declares all global variables, constants, and configuration
 * parameters used throughout the VOXL OpenVINS server. It provides centralized
 * management of system state, configuration, and operational parameters.
 *
 * The file contains:
 * - Pipe channel definitions and constants
 * - Global state variables for VIO operation
 * - Configuration parameters for auto-reset and error handling
 * - Sensor-specific variables and synchronization primitives
 * - Image processing and camera management variables
 */

#ifndef VOXL_VARS_H
#define VOXL_VARS_H

#pragma once

// Standard includes
#include <memory>
#include <atomic>
#include <string>
#include <vector>
#include <mutex>
#include <cstdint>
#include <condition_variable>

// Third-party includes
#include <core/VioManager.h>
#include <core/VioManagerOptions.h>
#include <modal_pipe.h>
#include <modal_json.h>
#include <voxl_common_config.h>

// Local includes
#include "VoxlCommon.h"

// ============================================================================
// DATA STRUCTURES
// ============================================================================
namespace voxl
{
    /**
     * @struct img_ringbuf_packet
     * @brief Structure to hold image data packet for the ring buffer
     *
     * This structure encapsulates all the data needed for a single image frame,
     * including camera ID, metadata, and the actual pixel data.
     */
    typedef struct img_ringbuf_packet
    {
        int camid;                            ///< Camera identifier
        camera_image_metadata_t metadata;     ///< Image metadata (timestamp, format, etc.)
        uint8_t image_pixels[MAX_IMAGE_SIZE]; ///< Raw image pixel data
    } img_ringbuf_packet;
}

// ============================================================================
// SERVER CHANNEL DEFINITIONS
// ============================================================================

/**
 * @brief MPA server channel for extended VIO data output
 *
 * Channel used for publishing comprehensive VIO data including
 * full state information, covariance matrices, and detailed tracking data.
 */
#define EXTENDED_CH 0

/**
 * @brief MPA server channel for simple VIO data output
 *
 * Channel used for publishing simplified VIO data with essential
 * pose and velocity information for basic applications.
 */
#define SIMPLE_CH 1

/**
 * @brief MPA server channel for overlay data output
 *
 * Channel used for publishing visual overlay data for debugging
 * and visualization purposes.
 */
#define OVERLAY_CH 2

/**
 * @brief MPA server channel for status information
 *
 * Channel used for publishing system status, health information,
 * and operational state data.
 */
#define OVSTATUS_CH 3

// ============================================================================
// CLIENT CHANNEL DEFINITIONS
// ============================================================================

/**
 * @brief MPA client channel for IMU data input
 *
 * Channel used for receiving inertial measurement unit data
 * from the IMU service.
 */
#define IMU_CH 0

/**
 * @brief MPA client channel for feature overlay data
 *
 * Channel used for receiving feature tracking overlay data
 * for visualization and debugging.
 */
#define FEAT_OVERLAY_CH 9

/**
 * @brief MPA client channel for barometer data input
 *
 * Channel used for receiving barometric pressure data
 * for altitude estimation.
 */
#define BARO_CH 10

/**
 * @brief MPA client channel for CPU monitoring data
 *
 * Channel used for receiving CPU usage and performance
 * monitoring data.
 */
#define CPU_CH 11

// ============================================================================
// PIPE NAMES AND LOCATIONS
// ============================================================================

/**
 * @brief Control commands available for VIO system
 *
 * Comma-separated list of control commands that can be sent
 * to the VIO system for operational control.
 */
#define OV_VIO_CONTROL_COMMANDS (RESET_VIO_SOFT "," RESET_VIO_HARD)

/**
 * @brief Name for extended VIO data pipe
 *
 * Pipe name for publishing comprehensive VIO data including
 * full state information and covariance matrices.
 */
#define OV_VIO_EXTENDED_NAME "ov_extended"

/**
 * @brief Location for extended VIO data pipe
 *
 * Full path location for the extended VIO data pipe.
 */
#define OV_VIO_EXTENDED_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR OV_VIO_EXTENDED_NAME "/"

/**
 * @brief Name for simple VIO data pipe
 *
 * Pipe name for publishing simplified VIO data with essential
 * pose and velocity information.
 */
#define OV_VIO_SIMPLE_NAME "ov"

/**
 * @brief Location for simple VIO data pipe
 *
 * Full path location for the simple VIO data pipe.
 */
#define OV_VIO_SIMPLE_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR OV_VIO_SIMPLE_NAME "/"

/**
 * @brief Name for overlay data pipe
 *
 * Pipe name for publishing visual overlay data for debugging
 * and visualization.
 */
#define OV_VIO_OVERLAY_NAME "ov_overlay"

/**
 * @brief Location for overlay data pipe
 *
 * Full path location for the overlay data pipe.
 */
#define OV_VIO_OVERLAY_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR OV_VIO_OVERLAY_NAME "/"

/**
 * @brief Name for status information pipe
 *
 * Pipe name for publishing system status and health information.
 */
#define OV_STATUS_NAME "ov_status"

/**
 * @brief Location for status information pipe
 *
 * Full path location for the status information pipe.
 */
#define OV_STATUS_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR OV_STATUS_NAME "/"

// ============================================================================
// SYSTEM CONSTANTS
// ============================================================================

/**
 * @brief Process name for the VOXL OpenVINS server
 *
 * Used for process identification, logging, and system integration.
 */
#define PROCESS_NAME "voxl-open-vins-server"

/**
 * @brief Maximum number of cameras supported by VOXL2
 *
 * Hardware limitation for the number of cameras that can be
 * simultaneously processed by the VOXL2 platform.
 */
#define MAX_CAM_CNT 6

/**
 * @brief Standard character buffer size
 *
 * Default buffer size for string operations and name storage.
 */
#define CHAR_BUF_SIZE 128

// ============================================================================
// CORE VIO MANAGER
// ============================================================================


/** @brief VIO manager options 
 * 
 * Options for configuring the VIO manager instance.
 * This includes settings for state initialization, estimator options,
 * and other operational parameters.
*/
extern ov_msckf::VioManagerOptions vio_manager_options;

/**
 * @brief Global VIO manager instance
 *
 * Main VIO manager that handles the complete visual-inertial
 * odometry pipeline. This is the central component that processes
 * camera and IMU data to estimate pose and velocity.
 */
extern std::unique_ptr<ov_msckf::VioManager> vio_manager;

// ============================================================================
// ATOMIC STATE VARIABLES
// ============================================================================

/**
 * @brief Main server running state
 *
 * Volatile flag indicating whether the main server loop is running.
 * Used for graceful shutdown and state management.
 */
extern volatile int main_running;

/**
 * @brief Current VIO system state
 *
 * Atomic variable representing the current operational state
 * of the VIO system (initializing, running, error, etc.).
 */
extern std::atomic<uint8_t> vio_state;

/**
 * @brief VIO error codes
 *
 * Atomic variable containing error codes and flags for
 * various VIO system errors and warnings.
 * Uses uint32_t to accommodate all error codes (up to bit 21).
 */
extern std::atomic<uint32_t> vio_error_codes;

/** @brief Should reset floag
 * 
 * Flag indicating that system should reset
 */
extern std::atomic<bool> reset_requested;

/**
 * @brief VIO reset state flag
 *
 * Atomic flag indicating whether the VIO system is currently
 * undergoing a reset operation.
 */
extern std::atomic<bool> is_resetting;

/** 
 * @brief Number of callbacks inside the system
 * 
 * Atomic counter for tracking the number of in-flight
 * callbacks or operations currently being processed that need
 * to be accounted for during reset.
 */
extern std::atomic<uint32_t> active_callbacks;      

/**
 * @brief Mutex for reset
 *
 * Synchronisation mutex used *only* for the reset hand-off
 */
extern std::mutex reset_mtx;

/** 
 * @brief Reset conditional variable 
 * 
 * Condition variable used to synchronize reset operations,
 * the reset thread will wait on active_callbacks to reach zero before proceeding.
 * */
extern std::condition_variable_any reset_cv;

/**
 * @brief System armed state
 *
 * Atomic flag indicating whether the system is armed and ready
 * for operation (typically used in drone applications).
 */
extern std::atomic<bool> is_armed;

/**
 * @brief Camera connection state
 *
 * Atomic flag indicating whether camera services are connected
 * and providing data.
 */
extern std::atomic<bool> is_cam_connected;

/**
 * @brief IMU connection state
 *
 * Atomic flag indicating whether the IMU service is connected
 * and providing data.
 */
extern std::atomic<bool> is_imu_connected;

// ============================================================================
// AUTO-RESET CONFIGURATION VARIABLES
// ============================================================================

/**
 * @brief Enable automatic VIO reset
 *
 * Flag to enable automatic reset of the VIO system when
 * certain error conditions are detected.
 */
extern int en_auto_reset;

/**
 * @brief Maximum velocity threshold for auto-reset
 *
 * Velocity threshold above which the system will trigger
 * an automatic reset if other conditions are met.
 */
extern float auto_reset_max_velocity;

/**
 * @brief Maximum instantaneous velocity covariance for auto-reset
 *
 * Threshold for the instantaneous velocity covariance that
 * triggers an automatic reset.
 */
extern float auto_reset_max_v_cov_instant;

/**
 * @brief Maximum velocity covariance for auto-reset
 *
 * Threshold for the average velocity covariance that
 * triggers an automatic reset.
 */
extern float auto_reset_max_v_cov;

/**
 * @brief Timeout for velocity covariance auto-reset
 *
 * Duration in seconds that the velocity covariance must exceed
 * the threshold before triggering an auto-reset.
 */
extern float auto_reset_max_v_cov_timeout_s;

/**
 * @brief Minimum number of features for auto-reset
 *
 * Minimum number of tracked features required to avoid
 * triggering an automatic reset.
 */
extern int auto_reset_min_features;

/**
 * @brief Timeout for minimum features auto-reset
 *
 * Duration in seconds that the feature count must be below
 * the minimum before triggering an auto-reset.
 */
extern float auto_reset_min_feature_timeout_s;

/**
 * @brief Timeout for auto-fallback mode
 *
 * Duration in seconds before the system falls back to
 * a simpler tracking mode.
 */
extern float auto_fallback_timeout_s;

/**
 * @brief Minimum velocity for auto-fallback
 *
 * Minimum velocity threshold required to trigger
 * auto-fallback mode.
 */
extern float auto_fallback_min_v;

/**
 * @brief Enable continuous yaw checks
 *
 * Flag to enable continuous monitoring of yaw angle
 * for stability and accuracy validation.
 */
extern bool en_cont_yaw_checks;

/**
 * @brief Fast yaw threshold
 *
 * Threshold for detecting rapid yaw changes that
 * may indicate tracking issues.
 */
extern float fast_yaw_thresh;

/**
 * @brief Fast yaw timeout
 *
 * Duration in seconds for fast yaw change detection
 * before triggering corrective actions.
 */
extern float fast_yaw_timeout_s;

/**
 * @brief Using stereo camera configuration
 *
 * Flag to indicate whether the system is configured
 * to use stereo cameras for depth estimation.
 */

extern int using_stereo;

/**
 * @brief Base folder for yaml configuration files
 *
 * Base folder for storing YAML configuration files
 * for the VINS.
 */
extern char folder_base[CHAR_BUF_SIZE];
/**
 * @brief Enable debug output
 *
 * Flag to enable detailed debug output and logging
 * for development and troubleshooting.
 */
extern int en_debug;

// ============================================================================
// SENSOR CONFIGURATION VARIABLES
// ============================================================================

/**
 * @brief IMU service name
 *
 * Name of the IMU service to connect to for
 * inertial measurement data.
 */
extern char imu_name[64];

/**
 * @brief Camera information vector
 *
 * Vector containing configuration and calibration information
 * for all cameras in the system.
 */
extern std::vector<cam_info> cam_info_vec;

// ============================================================================
// IMU-SPECIFIC VARIABLES
// ============================================================================

/**
 * @brief Last IMU timestamp in nanoseconds
 *
 * Timestamp of the most recent IMU measurement.
 * Used for synchronization and timing validation.
 */
extern volatile int64_t last_imu_timestamp_ns;

/**
 * @brief Mutex for IMU data access synchronization
 *
 * Mutex used to synchronize access to IMU data
 * between multiple threads.
 */
extern std::mutex imu_lock_mutex;

// ============================================================================
// CAMERA-SPECIFIC VARIABLES
// ============================================================================

/**
 * @brief Takeoff camera identifier
 *
 * Identifier for the camera used during takeoff
 * and initial flight phases.
 */
extern int takeoff_cam;

/**
 * @brief Enable takeoff camera functionality
 *
 * Flag to enable special handling for the takeoff camera,
 * including different processing parameters.
 */
extern int en_takeoff_cam;

/**
 * @brief Vector of takeoff camera identifiers
 *
 * Vector containing identifiers for all cameras
 * that should be used during takeoff.
 */
extern std::vector<int> takeoff_cams;

/**
 * @brief Last camera timestamp
 *
 * Timestamp of the most recent camera measurement.
 * Used for synchronization and timing validation.
 */
extern volatile int64_t last_cam_time;

/**
 * @brief Number of cameras currently in use
 *
 * Count of cameras that are currently active
 * and providing data to the VIO system.
 */
extern int cameras_used;

/**
 * @brief Altitude z
 * 
 * Atomic variable for the altitude z
 */
extern std::atomic<float> alt_z;

/** @brief Takeoff altitude threshold */
extern float takeoff_alt_threshold;

/** @brief Occlude stereo left */
extern bool occlude_stereo_left;

/** @brief Occlude stereo right */
extern bool occlude_stereo_right;

// ============================================================================
// PIPE CHANNEL VARIABLES
// ============================================================================

/**
 * @brief Camera pipe channel array
 *
 * Array containing the pipe channel numbers for each camera.
 * Used for managing camera data connections.
 */
extern int camera_pipe_channels[MAX_CAM_CNT];

// ============================================================================
// IMAGE PROCESSING VARIABLES
// ============================================================================

/**
 * @brief Image ring buffer pointer
 *
 * Pointer to the image ring buffer used for efficient
 * image data management and processing.
 */
extern voxl::img_ringbuf_packet *img_ringbuf;


// ============================================================================
// SETUP EXPERIMENTAL VARIABLES
// ============================================================================

extern bool sync_config; ///< Flag to indicate if configuration synchronization is enabled

#endif // VOXL_VARS_H
