/**
 * @file CameraManager.cpp
 * @brief Camera management system implementation for VOXL OpenVINS
 * @author Joao Leonardo Silva Cotta (@zauberflote1)
 * @date 2025
 * @version 1.0
 *
 * This file implements the CameraManager class, which provides centralized
 * management of all camera instances in the VOXL OpenVINS system. It handles
 * camera creation, initialization, and lifecycle management.
 *
 * The implementation provides:
 * - Singleton pattern for global camera management
 * - Template-based camera creation for extensibility
 * - Thread-safe operations with mutex protection
 * - Automatic camera type detection and creation
 * - Resource cleanup and shutdown management
 * - Camera fusion system integration
 */

#include "CameraManager.h"

namespace voxl
{

    /**
     * @brief Get the singleton instance of the CameraManager
     *
     * Returns the single instance of the CameraManager, creating it
     * if it doesn't exist (lazy initialization). This ensures that
     * only one camera manager exists throughout the application lifecycle.
     *
     * @return Reference to the CameraManager instance
     */
    CameraManager &CameraManager::getInstance()
    {
        static CameraManager instance;
        return instance;
    }

    /**
     * @brief Destructor for CameraManager
     *
     * Performs cleanup by calling the shutdown method to ensure proper
     * resource deallocation and camera disconnection.
     */
    CameraManager::~CameraManager()
    {
        shutdown();
    }

    /**
     * @brief Create and connect a specific type of camera
     *
     * Template method that creates and connects a camera of the specified
     * type. This allows for extensibility to support different camera
     * implementations while maintaining a consistent interface.
     *
     * The method performs the following operations:
     * - Creates a new camera instance of the specified type
     * - Stores the camera in the internal map by camera ID
     * - Attempts to connect the camera to its data source
     * - Logs success or failure information
     *
     * @tparam CameraType The type of camera to create (MonoCamera, StereoCamera, etc.)
     * @param config Camera configuration information
     * @return true if camera was successfully created and connected, false otherwise
     */
    template <typename CameraType>
    bool CameraManager::createAndConnectCamera(const cam_info &config)
    {
        auto camera = std::make_shared<CameraType>(config);
        cameras_[config.cam_id] = camera;

        if (true)
        {
            std::cout << "Created camera: " << config.name
                      << " in " << camera_mode_as_string(config.mode) << " mode" << std::endl;
        }

        if (!camera->connect())
        {
            std::cerr << "Failed to connect camera: " << config.name << std::endl;
            vio_error_codes |= ERROR_CODE_CAM_MISSING;
            return false;
        }

        return true;
    }

    /**
     * @brief Initialize the camera manager with camera configurations
     *
     * Sets up the camera manager with the provided camera configurations.
     * This method creates and initializes all cameras based on their
     * configuration information.
     *
     * The initialization process includes:
     * - Validating that the manager is not already initialized
     * - Checking that camera configurations are provided
     * - Filtering cameras by supported modes (currently only MONO)
     * - Creating and connecting each camera
     * - Starting the camera fusion system
     * - Setting the initialized flag
     *
     * If any camera fails to initialize, the method performs a clean
     * shutdown and returns false.
     *
     * @param camera_configs Vector of camera configurations
     * @return true if successful, false otherwise
     */
    bool CameraManager::initialize(const std::vector<cam_info> &camera_configs)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (initialized_)
        {
            std::cerr << "CameraManager already initialized" << std::endl;
            return true;
        }

        if (camera_configs.empty())
        {
            std::cerr << "No camera configurations provided" << std::endl;
            vio_error_codes |= ERROR_CODE_CAM_MISSING;
            return false;
        }

        std::vector<cam_info> cameras;
        for (const auto &config : camera_configs)
        {
            if (config.mode == MONO)
            {
                if (!createAndConnectCamera<MonoCamera>(config))
                {
                    shutdown();
                    return false;
                }
            }
            else if (config.mode == STEREO)
            {
                if (!createAndConnectCamera<StereoCamera>(config))
                {
                    shutdown();
                    return false;
                }
            }
            else
            {
                std::cerr << "Unsupported or disabled camera mode: " << camera_mode_as_string(config.mode)
                          << " for camera: " << config.name << std::endl;
            }
        }

        if (cameras_.empty())
        {
            std::cerr << "No valid mono cameras found." << std::endl;
            vio_error_codes |= ERROR_CODE_LOW_FEATURES;
            return false;
        }

        if (true)
        {
            std::cout << "Successfully initialized " << cameras_.size() << " cameras" << std::endl;
        }

        // No fusion thread: cameras push straight into the estimator's lock-free per-camera rings
        // and the IMU feed releases frames in global timestamp order (ov_msckf AsyncCameraBuffer)

        initialized_ = true;
        return true;
    }

    /**
     * @brief Shut down all cameras and clean up resources
     *
     * Performs a clean shutdown of all camera instances, disconnecting
     * them from their data sources and freeing allocated resources.
     *
     * The shutdown process includes:
     * - Checking if the manager is already shut down
     * - Disconnecting each camera in a separate thread to avoid blocking
     * - Waiting for disconnect operations to complete with timeout
     * - Clearing the camera collection
     * - Resetting the initialized flag
     *
     * The method uses detached threads for camera disconnection to
     * prevent blocking the main thread during shutdown.
     */
    void CameraManager::shutdown()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!initialized_)
            return;

        std::cout << "Shutting down CameraManager..." << std::endl;

        try
        {
            for (auto &[id, camera] : cameras_)
            {
                std::cout << "Disconnecting Camera " << id << std::endl;
                bool success = false;

                std::thread disconnect_thread([&camera, &success]()
                                              {
                try {
                    camera->disconnect();
                    success = true;
                } catch (const std::exception& e) {
                    std::cerr << "Error disconnecting camera: " << e.what() << std::endl;
                } });

                disconnect_thread.detach();

                for (int i = 0; i < 10 && !success; i++)
                {
                    usleep(10000); // 10ms
                }

                if (!success)
                {
                    std::cerr << "Camera disconnect timeout" << std::endl;
                }
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Shutdown exception: " << e.what() << std::endl;
        }

        cameras_.clear();
        initialized_ = false;
        std::cout << "CameraManager shutdown complete" << std::endl;
    }

    /**
     * @brief Get all cameras
     *
     * Returns a vector containing all managed camera instances.
     * The method is thread-safe and returns a copy of the camera
     * collection to prevent external modification.
     *
     * @return Vector of camera shared pointers
     */
    std::vector<std::shared_ptr<CameraBase>> CameraManager::getAllCameras()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<std::shared_ptr<CameraBase>> result;
        for (const auto &[id, cam] : cameras_)
        {
            result.push_back(cam);
        }
        return result;
    }


} // namespace voxl
