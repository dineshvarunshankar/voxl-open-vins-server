/**
 * @file CameraBase.h
 * @brief Base class for camera implementations in VOXL OpenVINS
 * @author Zauberflote
 * @date 2025
 * @version 1.0
 *
 * This header defines the abstract base class for all camera implementations
 * in the VOXL OpenVINS system. It provides the interface for camera handling,
 * pipe communication, image processing, and VIO integration.
 */

#ifndef CAMERABASE_H
#define CAMERABASE_H

#pragma once

// Standard includes
#include <memory>
#include <iostream>
#include <pthread.h>
#include <string>
#include <mutex>
#include <vector>
#include <functional>
#include <optional>

// Third-party includes
#include <opencv2/opencv.hpp>
#include <modal_pipe.h>
#include <boost/lockfree/spsc_queue.hpp>
#include <core/VioManager.h>
#include <core/VioManagerOptions.h>

// Local includes
#include "VoxlVars.h"

namespace voxl
{

    /**
     * @class CameraBase
     * @brief Base class for all camera implementations
     *
     * This abstract class defines the interface for camera handlers in the system.
     * It provides the foundation for different camera types (mono, stereo, etc.)
     * and handles pipe communication, image processing, and VIO integration.
     *
     * The class manages:
     * - Pipe client connections for image data
     * - Image processing and feature extraction
     * - Integration with the VIO manager
     * - Thread-safe operations
     * - OpenCL context for GPU acceleration
     */
    class CameraBase
    {
    public:
        /**
         * @brief Constructor
         * @param camera_info Camera configuration information
         */
        explicit CameraBase(const cam_info &camera_info);

        /**
         * @brief Virtual destructor
         */
        virtual ~CameraBase() = default;

        /**
         * @brief Initialize the camera pipe connection
         *
         * This method establishes the connection to the camera pipe service
         * and sets up the necessary callbacks for image data reception.
         *
         * @return True if successful, false otherwise
         */
        virtual bool connect();

        /**
         * @brief Disconnect and clean up resources
         *
         * This method properly closes the pipe connection and cleans up
         * any allocated resources.
         */
        virtual void disconnect();

        /**
         * @brief Get the camera information
         * @return Camera information structure
         */
        const cam_info &get_camera_info() const { return camera_info_; }

        /**
         * @brief Pop camera data from the queue
         *
         * Retrieves the next available camera data from the internal queue.
         * This method is used to get processed image data for VIO processing.
         *
         * The method uses a lock-free SPSC (Single Producer, Single Consumer)
         * queue for efficient thread-safe data transfer between the camera
         * callback thread and the VIO processing thread.
         *
         * @param out Reference to store the popped data
         * @return true if data was popped, false if queue is empty
         */
        bool popCameraData(ov_core::CameraData &out) { return camera_queue.pop(out); }

        /**
         * @brief Get the camera pipe channel
         * @return Pipe channel number
         */
        int get_channel() const { return channel_; }

        /**
         * @brief Get the camera identifier
         * @return Camera identifier
         */
        size_t get_id() const { return camera_info_.cam_id; }

    protected:
        /**
         * @brief Process incoming image data
         *
         * This method is called by the pipe callback when new image data arrives.
         * Derived classes must implement this method to handle their specific
         * image processing requirements.
         *
         * @param meta Image metadata containing timestamp and other information
         * @param frame Pointer to image data buffer
         */
        virtual void process_image(const camera_image_metadata_t &meta, voxl::ImageType img_type, void *frame) = 0;

        /**
         * @brief Common callback function for pipe client
         *
         * This function receives raw image data from the pipe and dispatches
         * it to the appropriate processing method. It serves as the entry
         * point for all camera data processing.
         *
         * @param ch Channel number
         * @param meta Image metadata
         * @param frame Pointer to image data
         * @param context Context pointer (the CameraBase instance)
         */
        static void camera_callback(int ch, camera_image_metadata_t meta, char *frame, void *context);

        static void camera_device_buffer_callback(int ch, mpa_ion_buf_t *data, void *context);

        // ============================================================================
        // MEMBER VARIABLES
        // ============================================================================

        /** @brief Camera configuration information */
        cam_info camera_info_;

        /** @brief Channel used for pipe communication */
        int channel_{-1};

        /** @brief Indicates if camera is connected to pipe */
        bool is_connected_{false};

        /** @brief Mutex for thread-safe operations */
        std::mutex mutex_;

        /** @brief OpenCL context used if GPU is enabled */
        cl_context ctx_{nullptr};

        /** @brief OpenCL command queue used if GPU is enabled */
        cl_command_queue q_{nullptr};

        /**
         * @brief Lock-free SPSC queue for camera data
         *
         * Used for efficient communication between camera thread and VIO thread.
         * Capacity of 64 provides approximately 2.4 seconds of buffering at 30fps.
         */
        boost::lockfree::spsc_queue<ov_core::CameraData, boost::lockfree::capacity<64>> camera_queue;

        /** @brief Instance-local buffer for image processing */
        img_ringbuf_packet curr_message_;

        /** @brief Per-instance reusable mask for feature tracking */
        cv::Mat use_mask_;
    };

} // namespace voxl

#endif // CAMERABASE_H