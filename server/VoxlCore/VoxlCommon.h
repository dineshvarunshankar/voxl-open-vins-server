/**
 * @file VoxlCommon.h
 * @brief Common definitions and utilities for the VOXL OpenVINS server
 * @author Zauberflote
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

/**
 * @brief Convert string representation to camera mode enumeration
 *
 * This function converts a string representation of a camera mode to its
 * corresponding enumeration value. Used for parsing configuration files
 * and command-line arguments.
 *
 * @param str_cm String representation of camera mode
 * @return Camera mode enumeration value
 */
static camera_mode string_camera_mode_to_enum(const char *str_cm)
{
    if (!strncmp(str_cm, "MONO", sizeof("MONO")))
    {
        return MONO;
    }
    if (!strncmp(str_cm, "STEREO", sizeof("STEREO")))
    {
        return STEREO;
    }
    if (!strncmp(str_cm, "STEREO_LEFT_ONLY", sizeof("STEREO_LEFT_ONLY")))
    {
        return STEREO_LEFT_ONLY;
    }
    if (!strncmp(str_cm, "STEREO_RIGHT_ONLY", sizeof("STEREO_RIGHT_ONLY")))
    {
        return STEREO_RIGHT_ONLY;
    }
    else
        return UNKNOWN;
}

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * @struct image_data
 * @brief Base packet structure for image data fed to trackers
 *
 * This structure contains all the necessary information for processing
 * images in the VIO system, including timestamps, tracker identifiers,
 * image data, and masking information.
 *
 * The structure is used as the primary data container for all image
 * processing operations in the tracking pipeline.
 */
typedef struct image_data
{
    int64_t timestamp_ns;            ///< Timestamp of image in nanoseconds
    std::vector<size_t> tracker_ids; ///< Vector of tracker IDs per camera, matching order of images + masks
    std::vector<cv::Mat> images;     ///< Vector of images to track across, in order matching ids vector
    std::vector<cv::Mat> masks;      ///< Vector of masks denoting regions of non-interest, in order matching ids vector
                                     ///< Mask regions with val == 255 will be ignored in tracking process
} image_data;

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
    char name[128];                                  ///< Camera name identifier
    char tracking_name[128];                         ///< Name used for tracking operations
    char preview_name[128];                          ///< Name used for preview/display
    camera_mode mode;                                ///< Camera operation mode
    Eigen::Matrix<double, 7, 1> cam_wrt_imu;         ///< Camera pose relative to IMU (quaternion + position)
    Eigen::Matrix<double, 8, 1> cam_calib_intrinsic; ///< Camera intrinsic calibration parameters
    int width;                                       ///< Image width in pixels
    int height;                                      ///< Image height in pixels
    bool is_fisheye;                                 ///< Flag indicating if camera uses fisheye lens
    size_t cam_id;                                   ///< Unique camera identifier
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

/**
 * @brief Mathematical sign function
 *
 * Returns the sign of a double value: 1.0 for positive, -1.0 for negative,
 * and 0.0 for zero.
 *
 * @param x Input value
 * @return Sign of the input value
 */
static double sign(double x)
{
    return x > 0 ? 1.0 : (x < 0 ? -1.0 : 0.0);
}

// ============================================================================
// MEMORY MANAGEMENT UTILITIES
// ============================================================================

/**
 * @brief Extract memory usage value from /proc/self/status
 *
 * This function reads the /proc/self/status file to extract specific
 * memory usage statistics for the current process.
 *
 * @param fieldName Name of the field to extract (e.g., "VmSize:", "VmRSS:")
 * @return Memory value in bytes, or 0 if not found
 */
static long getValueFromStatus(const std::string &fieldName)
{
    std::ifstream status("/proc/self/status");
    std::string line;
    while (std::getline(status, line))
    {
        if (line.find(fieldName) == 0)
        {
            long value;
            char unit[4]; // Increased size to accommodate "kB\0"
            sscanf(line.c_str(), "%*s %ld %s", &value, unit);
            // Convert to bytes based on unit
            if (strcmp(unit, "kB") == 0)
                value *= 1024;
            return value;
        }
    }
    return 0;
}

/**
 * @brief Print comprehensive memory usage statistics
 *
 * This function displays detailed memory usage information for the current
 * process, including virtual memory size, resident set size, data segment
 * size, and stack size. Useful for debugging and performance monitoring.
 *
 * @param label Label to identify the memory usage report
 */
static void printMemoryUsage(const std::string &label)
{
    const double mb = 1024.0 * 1024.0;

    long vmSize = getValueFromStatus("VmSize:");
    long vmRSS = getValueFromStatus("VmRSS:");
    long vmData = getValueFromStatus("VmData:");
    long vmStk = getValueFromStatus("VmStk:");

    std::cout << "\n=== " << label << " ===\n"
              << std::fixed << std::setprecision(2)
              << "Virtual Memory Size: " << vmSize / mb << " MB\n"
              << "Resident Set Size: " << vmRSS / mb << " MB\n"
              << "Data Segment Size: " << vmData / mb << " MB\n"
              << "Stack Size: " << vmStk / mb << " MB\n"
              << std::endl;
}

#endif // VOXL_COMMON_H
