/**
 * @file VoxlCommon.h
 * @brief Common definitions and utilities for the VOXL OpenVINS server
 * @author Joao Leonardo Silva Cotta (@zauberflote1)
 * @date 2025
 * @version 1.0
 *
 * This header file contains common definitions, data structures, and utility functions
 * used throughout the VOXL OpenVINS server. It provides the foundation for camera
 * management, IMU handling, and general system utilities.
 *
 * The file includes:
 * - Camera mode enumerations and conversion functions
 * - Image data structures for tracking
 * - Camera information structures
 * - Utility functions for timing and memory management
 * - Mathematical helper functions
 */

#ifndef VOXL_COMMON_H
#define VOXL_COMMON_H
#pragma once

// Standard includes
#include <stdint.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <functional>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <modal_pipe.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <iomanip>
#include <iostream>
#include <fstream>

// ============================================================================
// CONSTANTS AND MACROS
// ============================================================================

/**
 * @brief Arbitrary string buffer size for names and identifiers
 *
 * Used for camera names, tracking names, and other string identifiers
 * throughout the system.
 */
#define CM_CHAR_BUF_SIZE 64

/**
 * @brief Convert degrees to radians
 *
 * Macro for converting angle measurements from degrees to radians.
 */
#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0)
#endif

/**
 * @brief Convert radians to degrees
 *
 * Macro for converting angle measurements from radians to degrees.
 */
#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0 / M_PI)
#endif

/**
 * @brief Maximum size for image data buffer
 *
 * Calculated for 1280x800 resolution with 3 channels and 2 bytes per pixel
 */
constexpr size_t MAX_IMAGE_SIZE = 1280 * 800 * 3 * 2;

// ============================================================================
// ENUMERATIONS
// ============================================================================

/**
 * @enum camera_mode
 * @brief Camera operation modes for the VIO system
 *
 * This enumeration defines the different camera modes supported by the system.
 * The mode determines how multiple cameras are used in the visual-inertial
 * odometry pipeline.
 */
typedef enum camera_mode
{
    UNKNOWN = -1,         ///< Unknown or invalid camera mode
    MONO = 0,             ///< Single camera mode
    STEREO = 1,           ///< Stereo camera mode (both cameras active)
    STEREO_LEFT_ONLY = 2, ///< Stereo setup with only left camera active
    STEREO_RIGHT_ONLY = 3 ///< Stereo setup with only right camera active
} camera_mode;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Convert camera mode enumeration to string representation
 *
 * This function converts a camera_mode enumeration value to its corresponding
 * string representation for logging and debugging purposes.
 *
 * @param cm Camera mode enumeration value
 * @return String representation of the camera mode
 */
static std::string camera_mode_as_string(camera_mode cm)
{
    if (cm == MONO)
    {
        return "MONO";
    }
    else if (cm == STEREO)
    {
        return "STEREO";
    }
    else if (cm == STEREO_LEFT_ONLY)
    {
        return "STEREO_LEFT_ONLY";
    }
    else if (cm == STEREO_RIGHT_ONLY)
    {
        return "STEREO_RIGHT_ONLY";
    }
    else
        return "UNKNOWN";
}

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * @struct cam_info
 * @brief Camera information and calibration data
 *
 * This structure contains all the configuration and calibration information
 * for a single camera in the VIO system, including intrinsic parameters,
 * extrinsic parameters, and operational settings.
 */
typedef struct cam_info
{
    char name[128];              ///< Camera name identifier
    char tracking_name[128];     ///< Name used for tracking operations
    camera_mode mode;            ///< Camera operation mode
    bool is_occluded_on_takeoff; ///< Flag indicating if camera is occluded on takeoff
    size_t cam_id;               ///< Unique camera identifier
} cam_info;

// ============================================================================
// TIMING AND SYSTEM UTILITIES
// ============================================================================

/**
 * @brief Get monotonic time in nanoseconds
 *
 * This function provides a high-precision monotonic timestamp for timing
 * measurements and synchronization. It uses the CLOCK_MONOTONIC clock
 * which is not affected by system time changes.
 *
 * Used across main projects for consistent timing measurements.
 *
 * @return Monotonic time in nanoseconds, or -1 on error
 */
static int64_t _apps_time_monotonic_ns()
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts))
    {
        fprintf(stderr, "ERROR calling clock_gettime\n");
        return -1;
    }
    return (int64_t)ts.tv_sec * 1000000000 + (int64_t)ts.tv_nsec;
}

#endif // VOXL_COMMON_H
