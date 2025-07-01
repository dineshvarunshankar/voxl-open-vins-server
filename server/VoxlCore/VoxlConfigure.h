/**
 * @file VoxlConfigure.h
 * @brief Configuration management for VOXL OpenVINS server
 * @author Zauberflote
 * @date 2025
 * @version 1.0
 * 
 * This header defines the configuration management system for the VOXL OpenVINS
 * server. It provides functions for reading server configuration files and
 * synchronizing camera configurations with the system.
 * 
 * The configuration system handles:
 * - Server configuration file parsing and validation
 * - Camera configuration synchronization with system services
 * - Lens intrinsics and distortion model management
 * - Extrinsic calibration support (planned)
 */

#ifndef VOXL_CONFIGURE_H
#define VOXL_CONFIGURE_H
#pragma once

// Standard includes
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <string>

// Third-party includes
#include <modal_json.h>
#include <voxl_common_config.h>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

// Local includes
#include "VoxlVars.h"

// ============================================================================
// CONSTANTS AND MACROS
// ============================================================================

/**
 * @brief Default configuration file path
 * 
 * Path to the main configuration file for the VOXL OpenVINS server.
 * This file contains all server-specific configuration parameters.
 */
#define CONFIG_FILE "/etc/modalai/voxl-open-vins-server.conf"

/**
 * @brief Configuration file header comment
 * 
 * Standard header comment that is written to configuration files
 * to provide context and usage information.
 */
#ifndef CONFIG_FILE_HEADER
#define CONFIG_FILE_HEADER \
    "\
/**\n\
 * This file contains configuration that's specific to voxl-open-vins-server.\n\
 * \n\
 * *NOTE*: all time variables are measured in seconds\n\
 */\n"
#endif

// ============================================================================
// NAMESPACE DECLARATIONS
// ============================================================================

/**
 * @namespace voxl
 * @brief Main namespace for VOXL OpenVINS server components
 * 
 * This namespace contains all the core functionality for the VOXL OpenVINS
 * server, including configuration management, camera handling, and IMU
 * processing.
 */
namespace voxl {

// ============================================================================
// CONFIGURATION FUNCTIONS
// ============================================================================

/**
 * @brief Synchronize camera configuration with system services
 * 
 * This function reads camera configuration from system services and
 * synchronizes the lens intrinsics and distortion model parameters
 * with the VIO system. It ensures that the camera calibration data
 * used by the VIO system matches the current system configuration.
 * 
 * The function performs the following operations:
 * - Reads camera configuration from system services
 * - Updates lens intrinsics parameters
 * - Updates distortion model parameters
 * - Validates configuration consistency
 * 
 * TODO: Add support for extrinsic calibration parameters
 * 
 * @return 0 on success, non-zero on failure
 * @see read_server_config()
 */
int sync_cam_config(void);

/**
 * @brief Read and parse server configuration file
 * 
 * This function reads the main server configuration file and parses
 * all the parameters needed for VIO operation. It handles JSON format
 * configuration files and validates the parameters.
 * 
 * The function reads configuration for:
 * - VIO algorithm parameters

 * 
 * @return 0 on success, non-zero on failure
 * @see sync_cam_config()
 */
int read_server_config(void);

} // namespace voxl

#endif // VOXL_CONFIGURE_H