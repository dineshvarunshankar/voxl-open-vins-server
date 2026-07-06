/**
 * @file CameraManager.h
 * @brief Camera management system for VOXL OpenVINS
 * @author Joao Leonardo Silva Cotta (@zauberflote1)
 * @date 2025
 * @version 1.0
 *
 * This header defines the CameraManager class, which provides centralized
 * management of all camera instances in the VOXL OpenVINS system. It handles
 * camera creation, initialization, and lifecycle management.
 */

#pragma once

// Standard includes
#include <memory>
#include <vector>
#include <unordered_map>
#include <string>
#include <mutex>
#include <iostream>
#include <thread>
#include <unistd.h>

// Local includes
#include "CameraBase.h"
#include "MonoCameraMinimal.h"
#include "StereoCameraMinimal.h"

namespace voxl
{

    // ============================================================================
    // CAMERA MANAGER CLASS
    // ============================================================================

    /**
     * @class CameraManager
     * @brief Manages all camera instances in the system
     *
     * This class is responsible for creating, initializing, and managing
     * all camera instances. It provides a centralized point for camera
     * configuration and access using the singleton pattern.
     *
     * The CameraManager provides:
     * - Centralized camera lifecycle management
     * - Thread-safe camera access
     * - Automatic camera type detection and creation
     * - Resource cleanup and shutdown
     * - Camera configuration validation
     *
     * Key features:
     * - Singleton pattern for global access
     * - Template-based camera creation for extensibility
     * - Thread-safe operations with mutex protection
     * - Automatic resource management
     * - Support for multiple camera types
     */
    class CameraManager
    {
    public:
        /**
         * @brief Get the singleton instance of the CameraManager
         *
         * Returns the single instance of the CameraManager, creating it
         * if it doesn't exist (lazy initialization).
         *
         * @return Reference to the CameraManager instance
         */
        static CameraManager &getInstance();

        /**
         * @brief Initialize the camera manager with camera configurations
         *
         * Sets up the camera manager with the provided camera configurations.
         * This method creates and initializes all cameras based on their
         * configuration information.
         *
         * @param camera_configs Vector of camera configurations
         * @return True if successful, false otherwise
         */
        bool initialize(const std::vector<cam_info> &camera_configs);

        /**
         * @brief Shut down all cameras and clean up resources
         *
         * Performs a clean shutdown of all camera instances, disconnecting
         * them from their data sources and freeing allocated resources.
         */
        void shutdown();

        /**
         * @brief Get all cameras
         *
         * Returns a vector containing all managed camera instances.
         *
         * @return Vector of camera shared pointers
         */
        std::vector<std::shared_ptr<CameraBase>> getAllCameras();

    private:
        /**
         * @brief Private constructor (singleton pattern)
         */
        CameraManager() = default;

        /**
         * @brief Private destructor (singleton pattern)
         */
        ~CameraManager();

        /**
         * @brief Deleted copy constructor (singleton pattern)
         */
        CameraManager(const CameraManager &) = delete;

        /**
         * @brief Deleted assignment operator (singleton pattern)
         */
        CameraManager &operator=(const CameraManager &) = delete;

        /**
         * @brief Create and connect a specific type of camera
         *
         * Template method that creates and connects a camera of the specified
         * type. This allows for extensibility to support different camera
         * implementations while maintaining a consistent interface.
         *
         * @tparam CameraType The type of camera to create (MonoCamera, StereoCamera, etc.)
         * @param config Camera configuration
         * @return True if camera was successfully created and connected, false otherwise
         */
        template <typename CameraType>
        bool createAndConnectCamera(const cam_info &config);

        // ============================================================================
        // PRIVATE MEMBER VARIABLES
        // ============================================================================

        /** @brief Collection of camera instances by ID */
        std::unordered_map<size_t, std::shared_ptr<CameraBase>> cameras_;

        /**
         * @brief Mutex for thread-safe operations
         *
         * Mutable to allow locking in const methods
         */
        mutable std::mutex mutex_;

        /** @brief Flag indicating if the manager has been initialized */
        bool initialized_{false};
    };

} // namespace voxl