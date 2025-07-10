/**
 * @file StereoCameraMinimal.h
 * @brief Stereo camera implementation for VOXL OpenVINS
 * @author Zauberflote
 * @date 2025
 * @version 1.0
 *
 * This header defines the StereoCamera class, which specializes the CameraBase
 * for stereo camera configurations. It handles single camera image processing
 * and integration with the VIO system.
 */

#ifndef STEREOCAMERAMINIMAL_H
#define STEREOCAMERAMINIMAL_H
#pragma once
#include "CameraBase.h"
#include "CameraQueueFusion.h"
#include "VoxlVars.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <deque>

namespace voxl
{

    /**
     * @class StereoCamera
     * @brief Handles stereo camera input
     *
     * This class specializes CameraBase for stereo camera configurations.
     * It processes images from a single camera and feeds them to the VIO system.
     *
     * The StereoCamera class provides:
     * - Single camera image processing
     * - RAW8 format support
     * - ION buffer processing
     * - Queue-based data management
     * - System state awareness
     *
     * Key features:
     * - Inherits from CameraBase for common functionality
     * - Implements specific image processing for stereo setups
     * - Integrates with the camera queue fusion system
     * - Handles various image formats and buffer types
     */
    class StereoCamera : public CameraBase
    {
    public:
        /**
         * @brief Constructor
         * @param camera_info Camera configuration information
         */
        explicit StereoCamera(const cam_info &camera_info);

        /**
         * @brief Destructor
         */
        ~StereoCamera() override = default;

    protected:
        /**
         * @brief Process incoming image data
         *
         * Overrides the base class method to handle stereo camera-specific
         * image processing. This method is called by the pipe callback when
         * new image data arrives.
         *
         * @param meta Image metadata containing timestamp and format information
         * @param frame Pointer to image data buffer
         */
        void process_image(const camera_image_metadata_t &meta, char *frame) override;

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
         * @brief Check if system is in reset state
         *
         * Determines whether the VIO system is currently in a reset state,
         * which affects how image processing should be handled.
         *
         * @return True if system is resetting, false otherwise
         */
        bool is_system_resetting() const;

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
    };

} // namespace voxl
#endif // STEREOCAMERAMINIMAL_H