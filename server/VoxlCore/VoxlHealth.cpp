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

        // Perform health checks
        analyzeErrorCodes();
        checkSystemConnectivity();
        monitorSystemPerformance();
        checkAutoResetConditions();

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
    bool current_imu_connected = is_imu_connected.load();
    bool current_cam_connected = is_cam_connected.load();

    // Check IMU connection changes
    if (current_imu_connected != last_imu_connected_)
    {
        if (current_imu_connected)
        {
            std::cout << "[HEALTH] IMU connected" << std::endl;
            // Clear IMU-related errors when connection is restored
            clearErrorCodes(ERROR_CODE_IMU_MISSING | ERROR_CODE_DROPPED_IMU);
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
            clearErrorCodes(ERROR_CODE_CAM_MISSING | ERROR_CODE_DROPPED_CAM);
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

    // Check for timestamp issues
    int64_t time_since_imu = current_time_ns - last_imu_timestamp_ns;
    int64_t time_since_camera = current_time_ns - last_cam_time;

    if (time_since_imu > 1000000000)
    { // 1 second
        std::cerr << "[HEALTH] WARNING: No IMU data for " << (time_since_imu / 1000000) << "ms" << std::endl;
        vio_error_codes |= ERROR_CODE_IMU_MISSING;
    }

    if (time_since_camera > 1000000000)
    { // 1 second
        std::cerr << "[HEALTH] WARNING: No camera data for " << (time_since_camera / 1000000) << "ms" << std::endl;
        vio_error_codes |= ERROR_CODE_CAM_MISSING;
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

    uint32_t current_error_codes = vio_error_codes.load();
    bool should_reset = false;
    std::string reset_reason;

    // Check for critical errors that warrant immediate reset
    if (current_error_codes & ERROR_CODE_COVARIANCE)
    {
        should_reset = true;
        reset_reason = "Covariance matrix issues";
    }
    else if (current_error_codes & ERROR_CODE_IMU_OOB)
    {
        should_reset = true;
        reset_reason = "IMU out of bounds";
    }
    else if (current_error_codes & ERROR_CODE_STALLED)
    {
        should_reset = true;
        reset_reason = "Frame processing stalled";
    }
    else if (current_error_codes & ERROR_CODE_BAD_TIMESTAMP)
    {
        should_reset = true;
        reset_reason = "Bad timestamps";
    }

    if (should_reset)
    {
        std::cerr << "[HEALTH] AUTO-RESET RECOMMENDED: " << reset_reason << std::endl;
        std::cerr << "[HEALTH] Current error codes: 0x" << std::hex << (int)current_error_codes << std::dec << std::endl;

        // Set reset flag (this would trigger reset in main loop)
        is_resetting.store(true);
    }
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
void HealthCheck::clearErrorCodes(uint32_t error_mask)
{
    uint32_t current_errors = vio_error_codes.load();
    uint32_t new_errors = current_errors & ~error_mask;
    vio_error_codes.store(new_errors);

    if (en_debug)
    {
        std::cout << "[HEALTH] Cleared error codes: 0x" << std::hex << (int)error_mask << std::dec << std::endl;
    }
}
