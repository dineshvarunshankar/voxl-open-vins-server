/**
 * @file ImuMinimal.h
 * @brief IMU interface and data handling for VOXL OpenVINS
 * @author Zauberflote
 * @date 2025
 * @version 1.0
 * 
 * This header defines the IMU interface and callback functions for handling
 * inertial measurement unit data in the VOXL OpenVINS system. It provides
 * the connection to the IMU service and data processing capabilities.
 */

#ifndef IMU_MODULE_H
#define IMU_MODULE_H

// Standard includes
#include <mutex>
#include <functional>
#include <optional>
#include <array>

// Third-party includes
#include <modal_pipe.h>

// Local includes
#include "VoxlVars.h"
#include "VoxlCommon.h"
#include "CameraQueueFusion.h"
#include "VoxlHK.h"

// ============================================================================
// EXTERNAL DECLARATIONS
// ============================================================================

/** @brief Mutex for IMU data access synchronization */
extern std::mutex imu_lock_mutex;

// ============================================================================
// CALLBACK FUNCTIONS
// ============================================================================

/**
 * @brief Callback for IMU disconnect events
 * 
 * This function is called when the IMU service disconnects. It handles
 * the cleanup and state management required when IMU data becomes unavailable.
 * 
 * @param ch Channel number (unused)
 * @param context Context pointer (unused)
 */
void _imu_disconnect_cb(__attribute__((unused)) int ch,
                        __attribute__((unused)) void *context);

/**
 * @brief Handler for incoming IMU data
 * 
 * This callback processes incoming IMU data, updates the system state,
 * and triggers appropriate processing based on motion state. It serves
 * as the primary entry point for all IMU data processing in the system.
 * 
 * The function:
 * - Extracts accelerometer and gyroscope measurements
 * - Updates system timestamps and state
 * - Triggers VIO processing when appropriate
 * - Manages data synchronization with camera data
 * 
 * @param ch Channel number (unused)
 * @param data Pointer to IMU data buffer
 * @param bytes Size of data buffer in bytes
 * @param context Context pointer (unused)
 */
void _imu_data_handler_cb(__attribute__((unused)) int ch, 
                         char *data, int bytes, 
                         __attribute__((unused)) void *context);

// ============================================================================
// SERVICE MANAGEMENT
// ============================================================================

/**
 * @brief Creates IMU pipe client and associated callbacks
 *
 * This function sets up the disconnect and data handler callbacks,
 * and opens the client pipe connection to the IMU service. It initializes
 * the complete IMU data pipeline for the VIO system.
 * 
 * The function performs the following operations:
 * - Sets up disconnect callback for graceful handling of service disconnection
 * - Sets up data handler callback for processing incoming IMU measurements
 * - Opens the client pipe connection to the IMU service
 * - Configures the pipe for optimal data flow
 * 
 * @return 0 on success, non-zero on failure
 */
int connect_imu_service(void);

#endif // IMU_MODULE_H
