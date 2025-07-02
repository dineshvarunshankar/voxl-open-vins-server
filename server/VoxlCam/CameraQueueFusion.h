/**
 * @file CameraQueueFusion.h
 * @brief Camera queue fusion system for VOXL OpenVINS
 * @author Zauberflote
 * @date 2025
 * @version 1.0
 *
 * This header defines the camera queue fusion system that synchronizes
 * and combines data from multiple cameras. It provides temporal alignment
 * and batch processing capabilities for multi-camera VIO systems.
 */

#ifndef CAMERA_QUEUE_FUSION_H
#define CAMERA_QUEUE_FUSION_H

// Standard includes
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <iostream>
#include <condition_variable>
#include <deque>
#include <thread>

// Third-party includes
#include <opencv2/opencv.hpp>

// Local includes
#include "VoxlVars.h"
#include "VoxlCommon.h"
#include "CameraBase.h"
#include "MonoCameraMinimal.h"

// ============================================================================
// CONSTANTS
// ============================================================================

/**
 * @brief Maximum number of cameras supported
 *
 * Limited by 32-bit mask implementation for camera ready tracking
 */
constexpr size_t MAX_CAMERA_COUNT = 6;

// ============================================================================
// CAMERA QUEUE FUSION CLASS
// ============================================================================

/**
 * @class CameraQueueFusion
 * @brief Camera queue fusion system for multi-camera synchronization
 *
 * This class implements a sophisticated system for synchronizing and
 * fusing data from multiple cameras. It uses a mask-based approach to
 * track camera readiness and provides temporal alignment of camera data.
 *
 * Features:
 * - Multi-camera synchronization using bit masks
 * - Temporal alignment of camera frames
 * - Thread-safe queue management
 * - Event-driven processing with condition variables
 * - Batch processing capabilities
 *
 * The system works by:
 * 1. Tracking which cameras have new data available
 * 2. Waiting for all expected cameras to be ready
 * 3. Fusing the synchronized data into batches
 * 4. Providing sorted output for VIO processing
 */
class CameraQueueFusion
{
public:
    /**
     * @brief Get singleton instance
     * @return Reference to the singleton CameraQueueFusion instance
     */
    static CameraQueueFusion &getInstance();

    /**
     * @brief Start the fusion system
     *
     * Initializes the fusion system with the specified number of cameras
     * and starts the background fusion thread.
     *
     * @param num_cams Number of cameras to synchronize
     */
    void start(size_t num_cams);

    /**
     * @brief Mark a camera as ready with new data
     *
     * This method is called when a camera has new data available.
     * It updates the camera ready mask and may trigger fusion processing.
     *
     * @param cam_id Camera identifier (0-based)
     */
    void markCameraReady(size_t cam_id);

    /**
     * @brief Get sorted batch of camera data
     *
     * Retrieves a batch of synchronized camera data that is sorted
     * by timestamp and filtered by the specified cutoff time.
     *
     * @param timestamp_cutoff Timestamp cutoff for data inclusion
     * @param out Output vector to store the sorted camera data
     * @return true if data was retrieved, false if no data available
     */
    bool getSortedBatch(double timestamp_cutoff, std::vector<ov_core::CameraData> &out);

private:
    /**
     * @brief Main fusion loop
     *
     * Background thread function that continuously processes camera data
     * and performs temporal synchronization and fusion.
     */
    void fusionLoop();

    // ============================================================================
    // PRIVATE MEMBER VARIABLES
    // ============================================================================

    /** @brief Bit mask indicating which cameras are ready */
    std::atomic<uint32_t> camera_ready_mask_{0};

    /** @brief Expected mask when all cameras should be ready */
    uint32_t expected_mask_ = 0;

    /** @brief Number of cameras in the system */
    size_t num_cams_ = 0;

    /** @brief Mutex for condition variable synchronization */
    std::mutex cv_mtx_;

    /** @brief Condition variable for event-driven wake up */
    std::condition_variable cv_;

    /** @brief Mutex for fusion data access */
    std::mutex fusion_mutex_;

    /** @brief Queue of fused camera frames */
    std::deque<ov_core::CameraData> fused_frames_;

    /** @brief Flag indicating if fusion system is running */
    std::atomic<bool> running_{false};

    /** @brief Background fusion thread */
    std::thread fusion_thread_;
};

#endif // CAMERA_QUEUE_FUSION_H