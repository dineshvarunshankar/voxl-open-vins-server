/**
 * @file MonoCameraMinimal.cpp
 * @brief Monocular camera implementation for VOXL OpenVINS
 * @author Zauberflote
 * @date 2025
 * @version 1.0
 *
 * This file implements the MonoCamera class, which specializes the CameraBase
 * for monocular camera configurations. It handles single camera image processing
 * and integration with the VIO system.
 *
 * The implementation provides:
 * - Single camera image processing for RAW8 format
 * - ION buffer processing for efficient memory management
 * - Queue-based data management for VIO integration
 * - System state awareness and validation
 * - Camera fusion system integration
 * - Thread-safe data handling
 */

#include "MonoCameraMinimal.h"


namespace voxl
{

    /**
     * @brief Constructor for MonoCamera
     *
     * Initializes a monocular camera instance by calling the base class
     * constructor with the provided camera configuration information.
     *
     * @param camera_info Camera configuration and calibration information
     */
    MonoCamera::MonoCamera(const cam_info &camera_info)
        : CameraBase(camera_info)
    {
    }

    /**
     * @brief Process incoming image data
     *
     * Overrides the base class method to handle monocular camera-specific
     * image processing. This method is called by the pipe callback when
     * new image data arrives.
     *
     * The processing includes:
     * - Updating camera connection status and timestamps
     * - Checking system readiness before processing
     * - Routing to appropriate format-specific processing
     * - Updating current image dimensions
     *
     * Currently supports RAW8 format with fast-path processing.
     *
     * @param meta Image metadata containing timestamp and format information
     * @param frame Pointer to image data buffer
     */
    void MonoCamera::process_image(const camera_image_metadata_t &meta, char *frame)
    {

        // Update flags quickly
        is_cam_connected = true;
        last_cam_time = _apps_time_monotonic_ns();

        // Early return if system not ready
        if (!is_system_ready())
            return;
            
        // if we are resetting, just return
        if (is_resetting.load(std::memory_order_relaxed)) return;

        // indicate that we are processing IMU data
        active_callbacks.fetch_add(1, std::memory_order_acquire);
        if (is_resetting.load(std::memory_order_relaxed))
        {
            active_callbacks.fetch_sub(1, std::memory_order_release);
            return;
        }

        // Update dimensions
        current_height = meta.height;
        current_width = meta.width;

        // Process only supported formats with fast path
        if (meta.format == IMAGE_FORMAT_RAW8)
        {
            process_raw8(meta, frame);
        }
        else
        {
            // Rare case, can be slower
            fprintf(stderr, "Unsupported image format: %d\n", meta.format);
            vio_error_codes |= ERROR_CODE_CAM_BAD_FORMAT;
        }

        // check if in flight processing count reaches zero
        if (active_callbacks.fetch_sub(1, std::memory_order_release) == 1)
        {
            // If we are resetting, notify the reset condition variable
            // This will wake up the reset thread to continue processing
            std::lock_guard<std::mutex> lk(reset_mtx);
            reset_cv.notify_one();
        }
    }

    /**
     * @brief Process RAW8 image format
     *
     * Handles the processing of RAW8 format images, which is a common
     * format for monochrome cameras used in VIO systems.
     *
     * The processing includes:
     * - Setting up image ring buffer packet with metadata
     * - Copying image data to internal buffer
     * - Creating OpenCV Mat view of the image data
     * - Setting up mask for feature tracking regions
     * - Creating CameraData message for VIO processing
     * - Pushing data to camera queue
     * - Notifying fusion system of data availability
     *
     * @param meta Image metadata containing timestamp and dimensions
     * @param frame Pointer to image data
     */
    void MonoCamera::process_raw8(const camera_image_metadata_t &meta, char *frame)
    {
        // TODO: FIX INTERCHANCHABLE META AND CURR_MESSAGE USAGE, KEEP IT CONSISTENT, RESPECT HALACHAH
        //  Use static buffer to avoid allocations in the hot path
        curr_message_.camid = get_channel();
        curr_message_.metadata = meta;
        memcpy(curr_message_.image_pixels, reinterpret_cast<uint8_t *>(frame), meta.size_bytes);

        // OpenCV view (no copy)
        cv::Mat image(current_height, current_width, CV_8UC1, curr_message_.image_pixels);

        // Check if dimensions changed and update mask efficiently
        const bool dimensions_changed = (use_mask_.rows != current_height || use_mask_.cols != current_width);
        if (dimensions_changed) {
            mask_dimensions_changed_ = true;
        }

        // Determine if mask should be active based on occlusion and altitude
        const bool should_mask = camera_info_.is_occluded_on_takeoff && 
                                std::abs(alt_z.load(std::memory_order_relaxed)) < takeoff_alt_threshold;
        
        // Update mask only when necessary
        update_mask_if_needed(should_mask);

        ov_core::CameraData message;
        message.timestamp = (meta.timestamp_ns) * 1e-09; // TODO: check  if we should consider adding exposure time/2  --> NAIVELY ADDING BRING CHAOS (Multi-cam) DO IT AT YOUR OWN RISK
        message.sensor_ids.push_back(get_id());

        // clone might be optional --> depends on consumer thread ownership guarantees TODO: CHECK THIS LATER ON
        message.images.emplace_back(image.clone());
        message.masks.emplace_back(use_mask_);

        if (!camera_queue.push(message))
        {
            if (true)
            {
                // TODO: DROP OLDEST FRAME, ADD NEW FRAME --> RIGHT NOW WE JUST DROP THE NEW FRAME, NOT KOSHER
                std::cerr << "Camera queue full — dropping frame from cam " << get_channel() << std::endl;
                vio_error_codes |= ERROR_CODE_DROPPED_CAM;
            }
        }
        else
        {
            // Notify fusion system that camera data is ready
            CameraQueueFusion::getInstance().markCameraReady(get_id());
        }
    }

    void MonoCamera::update_mask_if_needed(bool should_mask)
    {
        // Check if we need to update the mask
        const bool needs_update = !current_mask_state_.has_value() || 
                                 current_mask_state_.value() != should_mask ||
                                 mask_dimensions_changed_;
        
        if (!needs_update) {
            return;
        }

        // Update mask state
        current_mask_state_ = should_mask;
        mask_dimensions_changed_ = false;

        // Create or update mask efficiently
        if (should_mask) {
            // Use cv::Scalar constructor for better performance
            use_mask_ = cv::Mat(current_height, current_width, CV_8UC1, cv::Scalar(255));
        } else {
            // Use cv::Scalar constructor for better performance  
            use_mask_ = cv::Mat(current_height, current_width, CV_8UC1, cv::Scalar(0));
        }
    }

    /**
     * @brief Check if system is in reset state
     *
     * Determines whether the VIO system is currently in a reset state,
     * which affects how image processing should be handled.
     *
     * @return true if system is resetting, false otherwise
     */
    bool MonoCamera::is_system_resetting() const
    {
        return is_resetting;
    }

    /**
     * @brief Check if system is ready to process images
     *
     * Determines whether the VIO system is ready to accept and process
     * new image data. The system is considered ready when both the IMU
     * is connected and the main process is running.
     *
     * @return true if system is ready, false otherwise
     */
    bool MonoCamera::is_system_ready() const
    {
        return is_imu_connected && main_running;
    }

} // namespace voxl