/**
 * @file VoxlVars.cpp
 * @brief Global variable definitions for VOXL OpenVINS server
 * @author Zauberflote
 * @date 2025
 * @version 1.0
 *
 * This file contains the definitions of all global variables declared in VoxlVars.h.
 * It provides the actual storage for system state, configuration parameters,
 * and operational variables used throughout the VIO system.
 */

#include "VoxlVars.h"
#include "VoxlCommon.h"
#include <atomic>
#include <memory>
#include <mutex>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

// ============================================================================
// CORE VIO MANAGER
// ============================================================================

/** @brief VIO manager options */
ov_msckf::VioManagerOptions vio_manager_options;

/** @brief Main VIO manager instance */
std::unique_ptr<ov_msckf::VioManager> vio_manager;

// ============================================================================
// ATOMIC STATE VARIABLES
// ============================================================================

/** @brief Main process running flag */
volatile int main_running = 1;

/** @brief Current VIO state (initializing, tracking, etc.) */
std::atomic<uint8_t> vio_state(0);

/** @brief VIO error codes */
std::atomic<uint32_t> vio_error_codes(0);

/** @brief Flag indicating that system should reset */
std::atomic<bool> reset_requested(false);

/** @brief Flag indicating if system is currently resetting */
std::atomic<bool> is_resetting(false);

/** @brief Counter which increments on resets */
std::atomic<uint32_t> reset_num_counter{0};

/** @brief Number of callbacks inside the system */
std::atomic<uint32_t> active_callbacks{0};

/** @brief Mutex used by reset thread */
std::mutex reset_mtx;

/** @brief Reset conditional variable */
std::condition_variable reset_cv;

/** @brief Flag indicating if system is armed */
std::atomic<bool> is_armed(false);

/** @brief Flag indicating if camera is connected */
std::atomic<bool> is_cam_connected(false);

/** @brief Flag indicating if IMU is connected */
std::atomic<bool> is_imu_connected(false);

// ============================================================================
// SERVER CONFIGURATION VARIABLES
// ============================================================================

/** @brief Enable automatic reset functionality */
int en_auto_reset = 0;

/** @brief Maximum velocity threshold for auto reset */
float auto_reset_max_velocity = 0.0f;

/** @brief Maximum velocity covariance for instant reset */
float auto_reset_max_v_cov_instant = 0.0f;

/** @brief Maximum velocity covariance for timeout reset */
float auto_reset_max_v_cov = 0.0f;

/** @brief Timeout duration for velocity covariance reset (seconds) */
float auto_reset_max_v_cov_timeout_s = 0.0f;

/** @brief Minimum number of features for auto reset */
int auto_reset_min_features = 0;

/** @brief Minimum feature timeout for auto reset (seconds) */
float auto_reset_min_feature_timeout_s = 0.0f;

/** @brief Minimum amount of time after initialization that quality is held low (CEP) */
float ok_state_grace_timeout_s = 3.0f;

/** @brief Auto fallback timeout (seconds) */
float auto_fallback_timeout_s = 0.0f;

/** @brief Minimum velocity for auto fallback */
float auto_fallback_min_v = 0.0f;

/** @brief Enable continuous yaw checks */
bool en_cont_yaw_checks = false;

/** @brief Fast yaw threshold */
float fast_yaw_thresh = 0.0f;

/** @brief Fast yaw timeout (seconds) */
float fast_yaw_timeout_s = 0.0f;

/** @brief using_stereo */
int using_stereo = 0;

/** @brief Base folder for yaml configuration files */
char folder_base[CHAR_BUF_SIZE] = "/etc/modalai/VoxlConfig/starling2";

/** @brief Enable debug output */
int en_debug = 0;

/** @brief Enable verbose output */
int en_verbose = 0;

/** @brief Configuration only mode */
int config_only = 0;

/** @brief Enable IMU body measurements */
bool en_imu_body = false;

// ============================================================================
// SENSOR CONFIGURATION VARIABLES
// ============================================================================

/** @brief IMU device name */
char imu_name[64];

/** @brief Vector of camera configuration information */
std::vector<cam_info> cam_info_vec;

// ============================================================================
// IMU-SPECIFIC VARIABLES
// ============================================================================

/** @brief Timestamp of last IMU data (nanoseconds) */
volatile int64_t last_imu_timestamp_ns = 0;

/** @brief Estimated IMU sampling rate in Hz */
std::atomic<double> imu_rate_hz(1024.0);

/** @brief Global frame transform instance */
voxl::FrameTransform frame_transform;

// ============================================================================
// CAMERA-SPECIFIC VARIABLES
// ============================================================================

/** @brief Takeoff camera identifier */
int takeoff_cam = 0;

/** @brief Enable takeoff camera functionality */
int en_takeoff_cam = 0;

/** @brief Vector of takeoff camera identifiers */
std::vector<int> takeoff_cams;

/** @brief Timestamp of last camera data (nanoseconds) */
volatile int64_t last_cam_time = 0;

/** @brief Number of cameras currently in use */
int cameras_used = 0;

/** @brief Altitude z */
std::atomic<float> alt_z(0.0f);

/** @brief Takeoff altitude threshold */
float takeoff_alt_threshold = 0.5f;

/** @brief Occlude stereo left */
bool occlude_stereo_left = false;

/** @brief Occlude stereo right */
bool occlude_stereo_right = false;

/** @brief Fusion rate in milliseconds */
float fusion_rate_dt_ms = 20.0; // 50Hz

// ============================================================================
// PIPE COMMUNICATION VARIABLES
// ============================================================================

/** @brief Camera pipe channels array */
int camera_pipe_channels[MAX_CAM_CNT] = {0};

// ============================================================================
// IMAGE PROCESSING VARIABLES
// ============================================================================

/** @brief Image ring buffer for processing */
voxl::img_ringbuf_packet *img_ringbuf = nullptr;

bool sync_config = true; ///< Flag to indicate if configuration synchronization is enabled

// ============================================================================
// FRAME TRANSFORM IMPLEMENTATION
// ============================================================================

void voxl::FrameTransform::update(const imu_data_t &data)
{
    if (!en_imu_body){
        printf("[INFO] IMU body measurements are disabled, THIS WON'T WORK AS EXPECTED.\n");
        return;
    }

    if (is_initialized)
    {
        detectJerk(data);
        return; // CHECK IF WE HAVE ALREADY DONE THIS
    }

    // FOR NOW WE ARE CHECKING WITH ONE SAMPLE -- PARTIALLY ASSUMING STATIC INITIALIZATION
    // MAX ELEMENT NORM IS THE AXIS WHERE GRAVITY IS MOST PREDOMINANT
    Eigen::Vector3d accel(data.accl_ms2[0], data.accl_ms2[1], data.accl_ms2[2]);
    accel = accel / accel.norm();

    Eigen::Vector3d grav_versor_expected = Eigen::Vector3d::UnitZ(); // Assuming gravity is along Z
    //now correct for negative z
    grav_versor_expected = -grav_versor_expected;
    auto dot_product = accel.dot(grav_versor_expected);
    dot_product = std::max(-1.0, std::min(1.0, dot_product));
    double angle_rad = std::acos(dot_product);
    double angle_deg = angle_rad * 180.0 / M_PI;
    printf("[INFO] IMU Initialization: Gravity angle = %.2f degrees\n", angle_deg);
    if (angle_deg > 15.0)
    {
        printf("[ERROR] IMU Initialization: Gravity angle too large, cannot initialize\n");
        is_initialized = false;
        return;
    } else
    {
        printf("[INFO] IMU Initialization: Gravity angle within acceptable range\n");
    }

    gravity_axis = Axis::Z;
    gravity_direction =  Direction::NEGATIVE;
    is_initialized = true;
    printf("[INFO] Frame transform initialized - Gravity axis: %d, Direction: %d\n",
           static_cast<int>(gravity_axis),
           static_cast<int>(gravity_direction));
}
void voxl::FrameTransform::detectJerk(const imu_data_t &data)
{
    // C++17 real-time optimization: Load velocity once with proper memory ordering
    const float vel_mag_current = vel_mag.load(std::memory_order_acquire);

    // FIX: Only skip jerk detection if in-flight AND we've completed at least one full cycle
    // This prevents deadlock on first initialization
    if (vel_mag_current > VEL_MAG_JERK_THRESHOLD)
    {
        // In-flight: Only reset if we've already collected enough samples
        // This allows first initialization to complete even if there's movement
        if (current_total_samples >= expected_total_samples * 0.5f)
        {
            resetJerkDetection();
            return;
        }
        // Otherwise, continue collecting samples for initial calibration
    }
    // C++17 Real-time optimization: Construct vectors in-place, avoid temporaries
    const Eigen::Vector3d acc_sample(data.accl_ms2[0], data.accl_ms2[1], data.accl_ms2[2]);
    const Eigen::Vector3d gyro_sample(data.gyro_rad[0], data.gyro_rad[1], data.gyro_rad[2]);

    // Accumulate data for jerk detection
    if (current_total_samples <= expected_total_samples * 0.5f)
    {
        avg_acc_1t0 += acc_sample;
        avg_gyro_1t0 += gyro_sample;
        acc1t0_samples.push_back(acc_sample);
        gyro1t0_samples.push_back(gyro_sample);
        current_total_samples += 1.0f;
        return;
    }
    else if (current_total_samples > expected_total_samples * 0.5f && current_total_samples < expected_total_samples)
    {
        // In the second half of the window
        avg_acc_2t1 += acc_sample;
        avg_gyro_2t1 += gyro_sample;
        acc2t1_samples.push_back(acc_sample);
        gyro2t1_samples.push_back(gyro_sample);
        current_total_samples += 1.0f;
    }
    else if (current_total_samples >= expected_total_samples)
    {
        // already have enough samples for this window, now compute variance per window segment

        double var_acc1t0 = 0.0, var_gyro1t0 = 0.0, var_acc2t1 = 0.0, var_gyro2t1 = 0.0;

        avg_acc_1t0 /= acc1t0_samples.size();
        avg_gyro_1t0 /= gyro1t0_samples.size();
        avg_acc_2t1 /= acc2t1_samples.size();
        avg_gyro_2t1 /= gyro2t1_samples.size();

        // note that the # of samples for the same segment is the same for acc and gyro
        for (size_t i = 0; i < acc1t0_samples.size(); ++i)
        {
            var_acc1t0 += (acc1t0_samples[i] - avg_acc_1t0).dot(acc1t0_samples[i] - avg_acc_1t0);
            var_gyro1t0 += (gyro1t0_samples[i] - avg_gyro_1t0).dot(gyro1t0_samples[i] - avg_gyro_1t0);
        }
        var_acc1t0 = std::sqrt(var_acc1t0 / (acc1t0_samples.size() - 1));
        var_gyro1t0 = std::sqrt(var_gyro1t0 / (gyro1t0_samples.size() - 1));

        for (size_t i = 0; i < acc2t1_samples.size(); ++i)
        {
            var_acc2t1 += (acc2t1_samples[i] - avg_acc_2t1).dot(acc2t1_samples[i] - avg_acc_2t1);
            var_gyro2t1 += (gyro2t1_samples[i] - avg_gyro_2t1).dot(gyro2t1_samples[i] - avg_gyro_2t1);
        }
        var_acc2t1 = std::sqrt(var_acc2t1 / (acc2t1_samples.size() - 1));
        var_gyro2t1 = std::sqrt(var_gyro2t1 / (gyro2t1_samples.size() - 1));
        if (en_debug)
        {
            printf("Accelerometer variance: %f, %f\n", var_acc1t0, var_acc2t1);
            printf("Gyroscope variance: %f, %f\n", var_gyro1t0, var_gyro2t1);
        }
        // C++17: Use structured bindings conceptually - evaluate jerk conditions
        bool accel_jerk_detected = false;
        bool gyro_jerk_detected = false;

        switch (jerk_opt)
        {
        case JerkOption::ACCEL_ONLY:
            accel_jerk_detected = (var_acc1t0 > ACC_VAR_THRESHOLD || var_acc2t1 > ACC_VAR_THRESHOLD);
            break;
        case JerkOption::GYRO_ONLY:
            gyro_jerk_detected = (var_gyro1t0 > GYRO_VAR_THRESHOLD || var_gyro2t1 > GYRO_VAR_THRESHOLD);
            break;
        case JerkOption::ACCEL_AND_GYRO:
            accel_jerk_detected = (var_acc1t0 > ACC_VAR_THRESHOLD || var_acc2t1 > ACC_VAR_THRESHOLD);
            gyro_jerk_detected = (var_gyro1t0 > GYRO_VAR_THRESHOLD || var_gyro2t1 > GYRO_VAR_THRESHOLD);
            break;
        case JerkOption::NONE:
            // Jerk detection disabled - report as jerked (non-static)
            resetJerkDetection();
            return;
        }

        // FIX: Report jerk status but DON'T reset counters - allows detection to continue
        // This prevents infinite reset loops on first initialization
        has_acc_jerk.store(accel_jerk_detected, std::memory_order_release);
        has_gyro_jerk.store(gyro_jerk_detected, std::memory_order_release);
        non_static.store(accel_jerk_detected || gyro_jerk_detected, std::memory_order_release);
        avg_acc_1t0.setZero();  // avg_acc_2t1;
        avg_gyro_1t0.setZero(); // avg_gyro_2t1;
        avg_acc_2t1.setZero();
        avg_gyro_2t1.setZero();
        current_total_samples = 0; // static_cast<float>(acc2t1_samples.size());
        acc1t0_samples.clear();    // acc2t1_samples;
        gyro1t0_samples.clear();   // gyro2t1_samples;
        acc2t1_samples.clear();    // acc2t1_samples;
        gyro2t1_samples.clear();   // gyro2t1_samples;

        return;
    }
}

void voxl::FrameTransform::resetJerkDetection()
{
    has_acc_jerk.store(true, std::memory_order_release);
    has_gyro_jerk.store(true, std::memory_order_release);
    non_static.store(true, std::memory_order_release);
    // reset for next window
    // note that we do not reset the entirity of the window, but only the second half
    avg_acc_1t0.setZero();  // avg_acc_2t1;
    avg_gyro_1t0.setZero(); // avg_gyro_2t1;
    avg_acc_2t1.setZero();
    avg_gyro_2t1.setZero();
    current_total_samples = 0; // static_cast<float>(acc2t1_samples.size());
    acc1t0_samples.clear();    // acc2t1_samples;
    gyro1t0_samples.clear();   // gyro2t1_samples;
    acc2t1_samples.clear();
    gyro2t1_samples.clear();
}
// ============================================================================
// JERK DETECTION VARIABLES
// ============================================================================

/**
 * @brief Flag indicating if accelerometer jerk is detected
 */
std::atomic<bool> has_acc_jerk(false);

/**
 * @brief Flag indicating if gyroscope jerk is detected
 */
std::atomic<bool> has_gyro_jerk(false);

/**
 * @brief Non-static flag for jerk detection
 */
std::atomic<bool> non_static(false);

/**
 * @brief Window size in seconds for jerk detection
 */
double window_size_s = 1.0;

std::atomic<float> vel_mag(0.0f); ///< Current velocity magnitude