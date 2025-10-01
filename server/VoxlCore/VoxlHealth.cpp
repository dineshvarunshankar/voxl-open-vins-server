/**
 * @file VoxlHealth.cpp
 * @brief Health check implementation for VOXL OpenVINS
 * @author Zauberflote
 * @date 2025
 * @version 1.0
 *
 * This file implements the health check system for the VOXL OpenVINS server.
 */

#include "VoxlHK.h"
using namespace voxl;
// ============================================================================
// HEALTH CHECK IMPLEMENTATION
// ============================================================================

/**
 * @brief Constructor for HealthCheck
 *
 * Initializes the health check system with default values.
 * The health check is not started until start() is called.
 */
HealthCheck::HealthCheck()
{
    // Initialize with current system state
    last_error_codes_ = vio_error_codes.load();
    last_vio_state_ = vio_state.load();
    last_imu_connected_ = is_imu_connected.load();
    last_cam_connected_ = is_cam_connected.load();
    last_health_check_ns_ = _apps_time_monotonic_ns();
}

/**
 * @brief Destructor for HealthCheck
 *
 * Ensures proper cleanup by calling stop() if the health check is still running.
 */
HealthCheck::~HealthCheck()
{
    stop();
}

/**
 * @brief Start the health check system
 *
 * Initializes and starts the health monitoring thread that runs at 30Hz.
 * The thread continuously monitors system health and error conditions.
 */
void HealthCheck::start()
{
    std::lock_guard<std::mutex> lock(health_mutex_);

    if (running_.load())
    {
        std::cerr << "HealthCheck already running" << std::endl;
        return;
    }

    running_.store(true);
    health_thread_ = std::thread(&HealthCheck::healthCheckLoop, this);
    health_thread_.detach();

    std::cout << "HealthCheck started - monitoring at 30Hz" << std::endl;
}

/**
 * @brief Stop the health check system
 *
 * Stops the health monitoring thread and performs cleanup.
 */
void HealthCheck::stop()
{
    std::lock_guard<std::mutex> lock(health_mutex_);

    if (!running_.load())
    {
        return;
    }

    running_.store(false);

    // Give the thread a moment to finish
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "HealthCheck stopped" << std::endl;
}

/**
 * @brief Main health check loop
 *
 * Runs at 30Hz and performs comprehensive health monitoring including:
 * - Error code analysis and logging
 * - System state validation
 * - Performance monitoring
 * - Auto-reset condition checking
 */
void HealthCheck::healthCheckLoop()
{
    const int64_t health_check_period_ns = 33333333; // 30Hz = ~33.33ms

    while (running_.load() && main_running)
    {
        auto start_time = _apps_time_monotonic_ns();
        // Update connectivity status first
        checkSystemConnectivity();

        // Publish blank VIO data packets when sensors are missing
        if (!is_imu_connected.load() || !is_cam_connected.load())
        {
            if (!is_imu_connected.load())
                std::cerr << "[HEALTH] ERROR: IMU disconnected; publishing blank VIO data" << std::endl;
            if (!is_cam_connected.load())
                std::cerr << "[HEALTH] ERROR: Camera disconnected; publishing blank VIO data" << std::endl;
            std::cout << "[HEALTH] Publishing blank VIO packet due to missing sensors" << std::endl;
            Publisher::getInstance().publishBlank();
        }
        else
        {
            analyzeErrorCodes();
            monitorSystemPerformance();
            checkAutoResetConditions();
            checkVINSResetRequest();
        }

        // Update counters
        health_check_count_++;
        last_health_check_ns_ = start_time;

        // Sleep to maintain 30Hz rate
        int64_t elapsed_ns = _apps_time_monotonic_ns() - start_time;
        int64_t sleep_ns = std::max<int64_t>(0, health_check_period_ns - elapsed_ns);

        if (sleep_ns > 0)
        {
            std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_ns));
        }
    }
}

/**
 * @brief Analyze and log error codes
 *
 * Examines the current error codes and logs detailed information
 * about any active errors or warnings.
 */
void HealthCheck::analyzeErrorCodes()
{
    uint32_t current_error_codes = vio_error_codes.load();

    // Check for new errors
    uint32_t new_errors = current_error_codes & ~last_error_codes_;
    uint32_t cleared_errors = last_error_codes_ & ~current_error_codes;

    if (new_errors != 0)
    {
        std::cerr << "[HEALTH] New errors detected: 0x" << std::hex << (int)new_errors << std::dec << std::endl;
        printf("[DEBUG] Current error codes: 0x%x, New errors: 0x%x\n", (int)current_error_codes, (int)new_errors);

        // Log specific error details
        if (new_errors & ERROR_CODE_COVARIANCE)
        {
            std::cerr << "[HEALTH] ERROR: Covariance matrix not positive definite" << std::endl;
        }
        if (new_errors & ERROR_CODE_IMU_OOB)
        {
            std::cerr << "[HEALTH] ERROR: IMU exceeded range (out of bounds)" << std::endl;
        }
        if (new_errors & ERROR_CODE_IMU_BW)
        {
            std::cerr << "[HEALTH] ERROR: IMU bandwidth too low" << std::endl;
        }
        if (new_errors & ERROR_CODE_NOT_STATIONARY)
        {
            std::cerr << "[HEALTH] ERROR: System not stationary at initialization" << std::endl;
        }
        if (new_errors & ERROR_CODE_NO_FEATURES)
        {
            std::cerr << "[HEALTH] ERROR: No features for extended period" << std::endl;
        }
        if (new_errors & ERROR_CODE_CONSTRAINT)
        {
            std::cerr << "[HEALTH] ERROR: Insufficient constraints from features" << std::endl;
        }
        if (new_errors & ERROR_CODE_FEATURE_ADD)
        {
            std::cerr << "[HEALTH] ERROR: Failed to add new features" << std::endl;
        }
        if (new_errors & ERROR_CODE_VEL_INST_CERT)
        {
            std::cerr << "[HEALTH] ERROR: Exceeded instant velocity uncertainty" << std::endl;
        }
        if (new_errors & ERROR_CODE_VEL_WINDOW_CERT)
        {
            std::cerr << "[HEALTH] ERROR: Exceeded velocity uncertainty" << std::endl;
        }
        if (new_errors & ERROR_CODE_DROPPED_IMU)
        {
            std::cerr << "[HEALTH] WARNING: Dropped IMU samples" << std::endl;
        }
        if (new_errors & ERROR_CODE_BAD_CAM_CAL)
        {
            std::cerr << "[HEALTH] ERROR: Intrinsic camera calibration questionable" << std::endl;
        }
        if (new_errors & ERROR_CODE_LOW_FEATURES)
        {
            std::cerr << "[HEALTH] ERROR: Insufficient good features to initialize" << std::endl;
        }
        if (new_errors & ERROR_CODE_DROPPED_CAM)
        {
            std::cerr << "[HEALTH] WARNING: Dropped camera frame" << std::endl;
        }
        if (new_errors & ERROR_CODE_DROPPED_GPS_VEL)
        {
            std::cerr << "[HEALTH] WARNING: Dropped GPS velocity sample" << std::endl;
        }
        if (new_errors & ERROR_CODE_BAD_TIMESTAMP)
        {
            std::cerr << "[HEALTH] ERROR: Sensor measurements with bad timestamps" << std::endl;
            printf("[DEBUG] Health check detected ERROR_CODE_BAD_TIMESTAMP\n");
        }
        if (new_errors & ERROR_CODE_IMU_MISSING)
        {
            std::cerr << "[HEALTH] ERROR: Missing IMU data" << std::endl;
        }
        if (new_errors & ERROR_CODE_CAM_MISSING)
        {
            std::cerr << "[HEALTH] ERROR: Missing camera frames" << std::endl;
        }
        if (new_errors & ERROR_CODE_CAM_BAD_RES)
        {
            std::cerr << "[HEALTH] ERROR: Camera resolution unsupported" << std::endl;
        }
        if (new_errors & ERROR_CODE_CAM_BAD_FORMAT)
        {
            std::cerr << "[HEALTH] ERROR: Camera format unsupported" << std::endl;
        }
        if (new_errors & ERROR_CODE_UNKNOWN)
        {
            std::cerr << "[HEALTH] ERROR: Unknown error" << std::endl;
        }
        if (new_errors & ERROR_CODE_STALLED)
        {
            std::cerr << "[HEALTH] ERROR: Frame processing stalled" << std::endl;
        }
    }

    if (cleared_errors != 0)
    {
        std::cout << "[HEALTH] Errors cleared: 0x" << std::hex << (int)cleared_errors << std::dec << std::endl;
    }

    last_error_codes_ = current_error_codes;
}

/**
 * @brief Check system connectivity
 *
 * Monitors the connection status of cameras and IMU, logging
 * any disconnection events or connectivity issues.
 */
void HealthCheck::checkSystemConnectivity()
{
    // THIS IS A REDO OF THE PAST SYSTEM CONNECTIVITY CHECK INSIDE monitorSystemPerformance --> THIS IS A BETTER APPROACH
    //  Detect stale sensor data
    const int64_t sensor_timeout_ns = 5000000000; // 5 second timeout --> MAYBE MAKE THIS SMALLER IF NEEDED BE
    int64_t now_ns = _apps_time_monotonic_ns();
    // If no new IMU data within timeout, mark IMU as disconnected
    if (last_imu_timestamp_ns != 0 && now_ns - last_imu_timestamp_ns > sensor_timeout_ns)
    {
        if (is_imu_connected.load())
        {
            std::cerr << "[HEALTH] IMU likely disconnected --> stale data (no data for "
                      << (now_ns - last_imu_timestamp_ns) / 1000000 << "ms)" << std::endl;
        }
        is_imu_connected.store(false);
    }
    // If no new camera data within timeout, mark camera as disconnected
    if (last_cam_time != 0 && now_ns - last_cam_time > sensor_timeout_ns)
    {
        if (is_cam_connected.load())
        {
            std::cerr << "[HEALTH] Camera likely disconnected --> stale data (no data for "
                      << (now_ns - last_cam_time) / 1000000 << "ms)" << std::endl;
        }
        is_cam_connected.store(false);
    }

    bool current_imu_connected = is_imu_connected.load();
    bool current_cam_connected = is_cam_connected.load();

    // Check IMU connection changes
    if (current_imu_connected != last_imu_connected_)
    {
        if (current_imu_connected)
        {
            std::cout << "[HEALTH] IMU connected" << std::endl;
            // Clear IMU-related errors when connection is restored --> CURRENTLY CLEANING ALL ERRORS
            clearErrorCodes(0, true);
            // Request reset upon IMU reconnection
            reset_requested.store(true);
            std::cout << "[HEALTH] Reset requested due to IMU reconnection" << std::endl;
        }
        else
        {
            std::cerr << "[HEALTH] ERROR: IMU disconnected" << std::endl;
            vio_error_codes |= ERROR_CODE_IMU_MISSING;
        }
        last_imu_connected_ = current_imu_connected;
    }

    // Check camera connection changes
    if (current_cam_connected != last_cam_connected_)
    {
        if (current_cam_connected)
        {
            std::cout << "[HEALTH] Camera connected" << std::endl;
            // Clear camera-related errors when connection is restored
            // CAN PROBABLY CLEAR ALL ERRORS HERE...
            clearErrorCodes(ERROR_CODE_CAM_MISSING | ERROR_CODE_DROPPED_CAM);
     
            // Don't trigger a reset on the very first connection
            if (first_camera_connection_seen_) {
                // Request reset upon camera reconnection
                reset_requested.store(true);
                std::cout << "[HEALTH] Reset requested due to camera reconnection" << std::endl;
            } else {
                first_camera_connection_seen_ = true; // no reset on first connection
            }
        }
        else
        {
            std::cerr << "[HEALTH] ERROR: Camera disconnected" << std::endl;
            vio_error_codes |= ERROR_CODE_CAM_MISSING;
        }
        last_cam_connected_ = current_cam_connected;
    }

    // Check VIO state changes
    uint8_t current_vio_state = vio_state.load();
    if (current_vio_state != last_vio_state_)
    {
        std::cout << "[HEALTH] VIO state changed: " << (int)last_vio_state_ << " -> " << (int)current_vio_state << std::endl;
        last_vio_state_ = current_vio_state;
    }
}

/**
 * @brief Monitor system performance
 *
 * Tracks system performance metrics including processing rates,
 * memory usage, and timing statistics.
 */
void HealthCheck::monitorSystemPerformance()
{
    static int64_t last_performance_log_ns = 0;
    int64_t current_time_ns = _apps_time_monotonic_ns();

    // Log performance metrics every 5 seconds
    if (current_time_ns - last_performance_log_ns > 5000000000)
    { // 5 seconds
        std::cout << "[HEALTH] Performance - Health checks: " << health_check_count_
                  << ", IMU timestamp: " << last_imu_timestamp_ns
                  << ", Camera timestamp: " << last_cam_time << std::endl;

        last_performance_log_ns = current_time_ns;
        health_check_count_ = 0; // Reset counter
    }
}

/**
 * @brief Check auto-reset conditions
 *
 * Evaluates whether auto-reset conditions are met based on
 * current system state and error conditions.
 */
void HealthCheck::checkAutoResetConditions()
{
    if (!en_auto_reset)
    {
        return; // Auto-reset disabled
    }

    // Suppress auto-reset for a grace period after a hard reset to allow sensors to come back online
    int64_t now = _apps_time_monotonic_ns();
    if (now - time_of_last_reset < INIT_FAILURE_TIMEOUT_NS)
    {
        return;
    }

    // Also skip auto-reset logic while the VIO manager is still initializing. OpenVINS may
    // require several seconds of IMU/vision data and heavy optimization before the
    // "initialized()" flag is set; triggering another reset in that window leads to a loop.
    if (!vio_manager || !vio_manager->initialized())
    {
        return;
    }

    uint32_t current_error_codes = vio_error_codes.load();

    if (current_error_codes != 0)
    {
        std::cerr << "[HEALTH] AUTO-RESET RECOMMENDED: Error code(s) detected: 0x" << std::hex << (int)current_error_codes << std::dec << std::endl;
        clearErrorCodes(0, true);
        // Set reset flag (this would trigger reset in main loop)
        reset_requested.store(true);
    }
}

/**
 * @brief Check for VINS reset request
 *
 * This function checks if a reset has been requested and handles the reset process.
 * It ensures that only one reset operation can be in progress at a time.
 */
void HealthCheck::checkVINSResetRequest()
{
    // atomically check if a reset has been requested, if not, return
    if (!reset_requested.exchange(false, std::memory_order_acq_rel))
        return;

    // check time since last reset
    int64_t current_time = _apps_time_monotonic_ns();
    uint64_t time_since_reset = current_time - time_of_last_reset;
    if (time_since_reset <= INIT_FAILURE_TIMEOUT_NS)
    {
        std::cout << "[HEALTH] Reset requested but last reset was too recent ("
                  << (time_since_reset / 1000000) << "ms ago), ignoring request" << std::endl;
        return;
    }

    // If reset is requested, check if we are already resetting
    if (is_resetting.exchange(true, std::memory_order_acq_rel))
    {
        std::cout << "[HEALTH] Reset already in progress, ignoring request\n";
        return;
    }

    if (en_debug)
        std::cout << "[HEALTH] Reset requested, preparing to reset VIO system" << std::endl;

    int rc = 0;
    try
    {
        rc = doHardReset();
        reset_num_counter.fetch_add(1, std::memory_order_acq_rel);
    }
    catch (const std::exception &e)
    {
        fprintf(stderr, "[ERROR] Exception during reset: %s\n", e.what());
        // Check if it's a permission error
        if (strstr(e.what(), "Operation not permitted") != nullptr)
        {
            fprintf(stderr, "[ERROR] Permission denied during reset - this may be due to insufficient privileges\n");
        }
        rc = -1;
    }

    if (rc == 0)
    {
        std::cout << "[HEALTH] VIO system reset successfully" << std::endl;

        // Clear last sensor timestamps; they will be filled when fresh data arrives
        last_imu_timestamp_ns = 0;
        last_cam_time = 0;
    }
    else
    {
        std::cerr << "[HEALTH] VIO system reset failed with code: " << rc << std::endl;
        // Clear reset flags even on failure to prevent getting stuck --> PRIME MOVE HERE
        reset_requested.store(false, std::memory_order_release);
    }

    time_of_last_reset = _apps_time_monotonic_ns();

    is_resetting.store(false, std::memory_order_release);
    reset_cv.notify_all();
    return;
}

int HealthCheck::doHardReset()
{
    // wait until all callbacks have finished processing
    {
        std::unique_lock<std::mutex> lk(reset_mtx);
        // Add timeout to prevent infinite blocking --> FOR NOW, 5 SECONDS
        bool wait_result = reset_cv.wait_for(lk, std::chrono::seconds(5),
            [this]
            {
                auto cur = active_callbacks.load(std::memory_order_acquire);
                return cur == 0;
            });

        if (!wait_result)
        {
            fprintf(stderr, "[ERROR] Timeout waiting for callbacks to finish during reset. active_callbacks=%d\n",
                    active_callbacks.load(std::memory_order_acquire));
            return -1;
        }
    }
    Publisher::getInstance().set_first_packet(true);
    clearErrorCodes(0, true);
    printf("[HEALTH] Hard reset in progress\n");

    // ensure we have a valid and initialized VIO manager; if not, create one directly
    if (!vio_manager || !vio_manager->initialized())
    {
        if (en_debug)
            std::cout << "[HEALTH] VIO manager was uninitialized, creating a fresh instance" << std::endl;

        try
        {
            vio_manager = std::make_unique<ov_msckf::VioManager>(vio_manager_options);
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "[ERROR] Failed to create VIO manager during reset: %s\n", e.what());
            return -1;
        }

        return 0; // fresh manager created, nothing else to reset
    }

    // Create references for old and new VIO manager
    std::unique_ptr<ov_msckf::VioManager> old_vio_manager;
    std::unique_ptr<ov_msckf::VioManager> new_vio_manager;

    try
    {
        new_vio_manager = std::make_unique<ov_msckf::VioManager>(vio_manager_options);

        if (!new_vio_manager)
        {
            fprintf(stderr, "[ERROR] Failed to create new VIO manager object\n");
            throw std::runtime_error("Failed to create new VIO manager");
        }

        old_vio_manager = std::move(vio_manager);
        vio_manager = std::move(new_vio_manager);
    }
    catch (const std::exception &e)
    {
        fprintf(stderr, "[ERROR] Exception during VIO manager creation: %s\n", e.what());
        if (old_vio_manager)
        {
            vio_manager = std::move(old_vio_manager); // restore previous manager
        }
        else
        {
            std::cerr << "[HEALTH] Warning: no previous VIO manager to restore" << std::endl;
        }
        return -1;
    }

    // destroy old VIO manager
    old_vio_manager.reset();

    return 0;
}

/**
 * @brief Clear specific error codes
 *
 * Clears the specified error codes from the global error state.
 * This is useful when errors are resolved and should no longer
 * be reported.
 *
 * @param error_mask Bit mask of error codes to clear
 */
void HealthCheck::clearErrorCodes(uint32_t error_mask, bool clear_all)
{
    if (clear_all)
    {
        vio_error_codes.store(0);
    }
    else
    {
        uint32_t current_errors = vio_error_codes.load();
        uint32_t new_errors = current_errors & ~error_mask;
        vio_error_codes.store(new_errors);
    }

    if (en_debug)
    {
        std::cout << "[HEALTH] Cleared error codes: 0x" << std::hex << (int)error_mask << std::dec << std::endl;
    }
}
