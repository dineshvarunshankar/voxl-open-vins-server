/**
 * @file MonoCameraMinimal.cpp
 * @brief Monocular camera implementation for VOXL OpenVINS
 * @author Joao Leonardo Silva Cotta (@zauberflote1)
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
#include <CL/cl.h>
#include <CL/cl_ext.h>

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

    static void release_cl_mem(void *mem)
    {
        if (mem)
        {
            cl_int err = clReleaseMemObject((cl_mem)mem);
            if (err != CL_SUCCESS)
            {
                fprintf(stderr, "Failed to release cl_mem object %p, err=%d\n", mem, err);
            }
        }
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
    void MonoCamera::process_image(const camera_image_metadata_t &meta, voxl::ImageType img_type, void *frame)
    {

        // Update flags quickly
        is_cam_connected = true;
        last_cam_time = _apps_time_monotonic_ns();

        // Early return if system not ready
        if (!is_system_ready())
        {
            if (img_type == voxl::ImageType::CL_MEM)
                release_cl_mem(frame);
            return;
        }

        // if we are resetting, just return
        if (is_resetting.load(std::memory_order_relaxed))
        {
            skip_jerk_detection = true;
            if (img_type == voxl::ImageType::CL_MEM)
                release_cl_mem(frame);
            return;
        }

        // indicate that we are processing IMU data
        active_callbacks.fetch_add(1, std::memory_order_acquire);
        if (is_resetting.load(std::memory_order_relaxed))
        {

            skip_jerk_detection = true;
            if (img_type == voxl::ImageType::CL_MEM)
                release_cl_mem(frame);

            if (active_callbacks.fetch_sub(1, std::memory_order_release) == 1)
            {
                std::lock_guard<std::mutex> lk(reset_mtx);
                reset_cv.notify_one();
            }
            return;
        }
        // CRITICAL FIX: DISABLE frame throttling during initialization completely
        // User reported issues with initialization - process ALL frames until VIO is stable

        // Check if VIO is fully initialized AND past grace period
        // Multi-camera (unsynced) rigs must NOT decimate per-camera: the alternating drops are
        // asymmetric across streams and break the epoch cadence the estimator anchors on
        const bool vio_ready_for_throttling = vio_manager &&
                                              vio_manager->initialized() &&
                                              vio_manager_options.state_options.num_cameras <= 1 &&
                                              vio_state.load(std::memory_order_acquire) == VIO_STATE_OK;

        // NEVER throttle during: INITIALIZING, FAILED, or first few seconds of OK state
        if (!skip_jerk_detection && vio_ready_for_throttling)
        {
            // Only throttle when VIO is stable and running
            const bool is_static = !(non_static.load(std::memory_order_acquire));
            const bool acc_no_jerk = !(has_acc_jerk.load(std::memory_order_acquire));

            if (en_debug)
            {
                printf("is_static: %d, acc_no_jerk: %d, drop_frames: %d\n",
                       is_static, acc_no_jerk, drop_frames);
            }

            if (is_static || acc_no_jerk)
            {
                if (!drop_frames)
                {
                    drop_frames = true;
                    if (en_debug)
                        printf("normal frame\n");
                }
                else
                {
                    // Drop this frame - platform is static
                    if (img_type == voxl::ImageType::CL_MEM)
                        release_cl_mem(frame);
                    drop_frames = false;
                    if (en_debug)
                        printf("dropping frame\n");
                    if (active_callbacks.fetch_sub(1, std::memory_order_release) == 1)
                    {
                        std::lock_guard<std::mutex> lk(reset_mtx);
                        reset_cv.notify_one();
                    }
                    return;
                }
            }
        }
        else if (!vio_ready_for_throttling && en_debug)
        {
            // During initialization or unstable state, process ALL frames
            printf("[INIT] VIO not stable - processing ALL frames (no throttling)\n");
        }
        // Update dimensions
        current_height = meta.height;
        current_width = meta.width;

        if (img_type == voxl::ImageType::CV_MAT)
        {
            // Process only supported formats with fast path
            if (meta.format == IMAGE_FORMAT_RAW8)
            {
                process_raw8(meta, (char *)frame);
            }
            else
            {
                // Rare case, can be slower
                fprintf(stderr, "Unsupported image format: %d\n", meta.format);
                vio_error_codes |= ERROR_CODE_CAM_BAD_FORMAT;
            }
        }
        if (img_type == voxl::ImageType::CL_MEM)
        {
            // Process only supported formats
            if (meta.format == IMAGE_FORMAT_RAW8)
            {
                process_device_buf_raw8(meta, (cl_mem)frame);
            }
            else
            {
                // Rare case, can be slower
                fprintf(stderr, "Unsupported image format: %d\n", meta.format);
                vio_error_codes |= ERROR_CODE_CAM_BAD_FORMAT;
            }
        }
        skip_jerk_detection = false;
        // check if in flight processing count reaches zero and if a reset is requested
        if (active_callbacks.fetch_sub(1, std::memory_order_release) == 1)
        {
            // Notify the reset thread to continue processing
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
        if (dimensions_changed)
        {
            mask_dimensions_changed_ = true;
        }

        // Determine if mask should be active based on occlusion and altitude
        const bool altitude_below_threshold = std::abs(alt_z.load(std::memory_order_relaxed)) < takeoff_alt_threshold;
        const bool should_mask = camera_info_.is_occluded_on_takeoff && altitude_below_threshold && !occlusion_threshold_passed_;

        if (!altitude_below_threshold && camera_info_.is_occluded_on_takeoff) {
            occlusion_threshold_passed_ = true;
        }

        // Update mask only when necessary
        update_mask_if_needed(should_mask);

        ov_core::CameraData message;
        message.timestamp = (meta.timestamp_ns) * 1e-09; // TODO: check  if we should consider adding exposure time/2  --> NAIVELY ADDING BRING CHAOS (Multi-cam) DO IT AT YOUR OWN RISK
        message.sensor_ids.push_back(get_id());

        // clone might be optional --> depends on consumer thread ownership guarantees TODO: CHECK THIS LATER ON
        message.images.emplace_back(image.clone());
        message.masks.emplace_back(use_mask_);

        modal_flow::ImageView iv{{meta.width, meta.height, modal_flow::PixelFormat::R8, meta.stride}, message.images[0].data, modal_flow::ExternalType::None, 0};
        modal_flow::Frame img_frame({get_id(), 0, iv});
        message.img_frames.push_back(img_frame);

        // Push straight into the estimator's lock-free per-camera ring (this pipe thread is the
        // single producer for this stream); the IMU feed releases frames in global time order.
        // Overflow/late drops are disposed + counted via the processed(false) callback.
        if (vio_manager && frame_transform.is_initialized)
        {
            vio_manager->feed_measurement_camera(message);
        }
    }

    void MonoCamera::process_device_buf_raw8(const camera_image_metadata_t &meta, cl_mem frame)
    {
        curr_message_.camid = get_channel();
        curr_message_.metadata = meta;

        // Check if dimensions changed and update mask efficiently
        const bool dimensions_changed = (use_mask_.rows != current_height || use_mask_.cols != current_width);
        if (dimensions_changed)
        {
            mask_dimensions_changed_ = true;
        }

        // Determine if mask should be active based on occlusion and altitude
        const bool altitude_below_threshold = std::abs(alt_z.load(std::memory_order_relaxed)) < takeoff_alt_threshold;
        const bool should_mask = camera_info_.is_occluded_on_takeoff && altitude_below_threshold && !occlusion_threshold_passed_;

        if (!altitude_below_threshold && camera_info_.is_occluded_on_takeoff) {
            occlusion_threshold_passed_ = true;
        }

        // Update mask only when necessary
        update_mask_if_needed(should_mask);

        ov_core::CameraData message;
        message.timestamp = (meta.timestamp_ns) * 1e-09; // TODO: check  if we should consider adding exposure time/2  --> NAIVELY ADDING BRING CHAOS (Multi-cam) DO IT AT YOUR OWN RISK
        message.sensor_ids.push_back(get_id());

        // modal_flow::ImageView ivA{{camA.width, camA.height, modal_flow::PixelFormat::R8, img_prev.step}, img_prev.data, modal_flow::ExternalType::None, 0};

        modal_flow::ImageView iv{{meta.width, meta.height, modal_flow::PixelFormat::R8, meta.stride}, nullptr, modal_flow::ExternalType::ClMem, static_cast<uint64_t>(reinterpret_cast<std::uintptr_t>(frame))};
        modal_flow::Frame img_frame({get_id(), 0, iv});

        message.cl_images.emplace_back(frame);

        message.img_frames.push_back(img_frame);

        message.images.emplace_back(cv::Mat::zeros(meta.height, meta.width, CV_8UC1));

        message.masks.emplace_back(use_mask_);

        // Push straight into the estimator's lock-free per-camera ring; drops (overflow/late) are
        // disposed via the processed(false) callback, which releases the cl_mem handle exactly once
        if (vio_manager && frame_transform.is_initialized)
        {
            vio_manager->feed_measurement_camera(message);
        }
        else
        {
            // No estimator yet: release the device buffer ourselves so it cannot leak
            release_cl_mem(frame);
        }
    }

    void MonoCamera::update_mask_if_needed(bool should_mask)
    {
        // Check if we need to update the mask
        const bool needs_update = !current_mask_state_.has_value() ||
                                  current_mask_state_.value() != should_mask ||
                                  mask_dimensions_changed_;

        if (!needs_update)
        {
            return;
        }

        // Update mask state
        current_mask_state_ = should_mask;
        mask_dimensions_changed_ = false;

        // Create or update mask efficiently
        if (should_mask)
        {
            // Use cv::Scalar constructor for better performance
            use_mask_ = cv::Mat(current_height, current_width, CV_8UC1, cv::Scalar(255));
        }
        else
        {
            // Use cv::Scalar constructor for better performance
            use_mask_ = cv::Mat(current_height, current_width, CV_8UC1, cv::Scalar(0));
        }
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