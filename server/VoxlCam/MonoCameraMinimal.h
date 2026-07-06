/**
 * @file MonoCameraMinimal.h
 * @brief Monocular camera implementation for VOXL OpenVINS
 * @author Joao Leonardo Silva Cotta (@zauberflote1)
 * @date 2025
 * @version 1.0
 *
 * This header defines the MonoCamera class, which specializes the CameraBase
 * for monocular camera configurations. It handles single camera image processing
 * and integration with the VIO system.
 */

#ifndef MONOCAMERAMINIMAL_H
#define MONOCAMERAMINIMAL_H
#pragma once
#include "CameraBase.h"
#include "VoxlVars.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <deque>
#include <optional>
#include <cstdint>

namespace voxl
{

    /**
     * @class MonoCamera
     * @brief Handles monocular camera input
     *
     * This class specializes CameraBase for monocular camera configurations.
     * It processes images from a single camera and feeds them to the VIO system.
     *
     * The MonoCamera class provides:
     * - Single camera image processing
     * - RAW8 format support
     * - ION buffer processing
     * - Queue-based data management
     * - System state awareness
     *
     * Key features:
     * - Inherits from CameraBase for common functionality
     * - Implements specific image processing for monocular setups
     * - Integrates with the camera queue fusion system
     * - Handles various image formats and buffer types
     */
    class MonoCamera : public CameraBase
    {
    public:
        /**
         * @brief Constructor
         * @param camera_info Camera configuration information
         */
        explicit MonoCamera(const cam_info &camera_info);

        /**
         * @brief Destructor
         */
        ~MonoCamera() override = default;

    protected:
        /**
         * @brief Process incoming image data
         *
         * Overrides the base class method to handle monocular camera-specific
         * image processing. This method is called by the pipe callback when
         * new image data arrives.
         *
         * @param meta Image metadata containing timestamp and format information
         * @param frame Pointer to image data buffer
         */
        void process_image(const camera_image_metadata_t &meta, voxl::ImageType type, void *frame) override;

    private:
        /**
         * @brief Process RAW8 image format
         *
         * Handles the processing of RAW8 format images, which is a common
         * format for monochrome cameras used in VIO systems.
         *
         * @param meta Image metadata
         * @param frame Pointer to image data
         */
        void process_raw8(const camera_image_metadata_t &meta, char *frame);

        /**
         * @brief Process RAW8 image format from device
         *
         * Handles the RAW8 image format, accounting for the fact that the
         * buffer pointer references memory on the device
         *
         * @param meta Image metdata
         * @param frame Pointer to cl_mem image buffer
         */
        void process_device_buf_raw8(const camera_image_metadata_t &meta, cl_mem frame);

        /**
         * @brief Update mask based on current system state
         *
         * Efficiently updates the mask only when necessary, avoiding
         * unnecessary allocations and copies.
         *
         * @param should_mask Whether the mask should be active
         */
        void update_mask_if_needed(bool should_mask);

        /**
         * @brief Check if system is ready to process images
         *
         * Determines whether the VIO system is ready to accept and process
         * new image data.
         *
         * @return True if system is ready, false otherwise
         */
        bool is_system_ready() const;

        /** @brief Current image height in pixels */
        int current_height;

        /** @brief Current image width in pixels */
        int current_width;

        /** @brief Current mask state to avoid unnecessary updates */
        std::optional<bool> current_mask_state_;

        /** @brief Flag indicating if mask dimensions have changed */
        bool mask_dimensions_changed_{false};

        /** @brief Flag indicating occlusion threshold has already been passed once */
        bool occlusion_threshold_passed_{false};
    };

} // namespace voxl
#endif // MONOCAMERAMINIMAL_H