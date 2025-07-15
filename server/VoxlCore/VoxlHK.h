/**
 * @file VoxlHK.h
 * @brief Housekeeping and data publishing for VOXL OpenVINS
 * @author Zauberflote
 * @date 2025
 * @version 1.0
 *
 * This header defines the housekeeping system for the VOXL OpenVINS server,
 * including data publishing, angular velocity calculations, and coordinate
 * frame transformations. It provides the interface for outputting VIO data
 * to external systems.
 */

#ifndef VOXL_HK_H
#define VOXL_HK_H
#pragma once

// Standard includes
#include <atomic>
#include <memory>

// Third-party includes
#include <modal_pipe.h>
#include <modal_json.h>
#include <voxl_common_config.h>
#include <core/VioManager.h>
#include <core/VioManagerOptions.h>
#include <types/LandmarkRepresentation.h>
#include <cstring>
#include <state/State.h>
#include <state/StateHelper.h>
#include <types/Landmark.h>
#include <types/Type.h>
#include <feat/Feature.h>
#include <cmath>
#include <algorithm>
#include <thread>
#include <chrono>
#include <Eigen/Eigenvalues>
// Local includes
#include "VoxlVars.h"
#include "VoxlCommon.h"

namespace voxl
{

    // ============================================================================
    // ENUMERATIONS
    // ============================================================================

    /**
     * @enum ANG_VEL_TYPE
     * @brief Types of angular velocity calculations
     *
     * This enumeration defines the different methods available for calculating
     * angular velocity from quaternion data.
     */
    enum class ANG_VEL_TYPE
    {
        QUAT_DIRTY, ///< Dirty quaternion-based angular velocity
        RPY_DIRTY,  ///< Dirty roll-pitch-yaw based angular velocity
        IMU_BIAS    ///< IMU bias-corrected angular velocity
    };

    // ============================================================================
    // UTILITY FUNCTIONS
    // ============================================================================

    /**
     * @brief Calculate dirty angular velocity from quaternions
     *
     * This function computes the angular velocity using a "dirty" method that
     * directly differentiates quaternions. It's computationally efficient but
     * may not be as accurate as more sophisticated methods.
     *
     * @param q_k Current quaternion (4x1 vector)
     * @param q_km1 Previous quaternion (4x1 vector)
     * @param dt Time difference in seconds
     * @return Angular velocity vector (3x1) in radians per second
     */
    inline Eigen::Matrix<double, 3, 1> dirtyOmega(const Eigen::Matrix<double, 4, 1> &q_k,
                                                  const Eigen::Matrix<double, 4, 1> &q_km1,
                                                  double dt)
    {
        dt *= 1e9; // Convert to nanoseconds for VOXL
        if (dt <= 0.0)
        {
            return Eigen::Vector3d::Zero(); // Bad dt
        }
        if (q_k.isZero(0) || q_km1.isZero(0))
        {
            return Eigen::Vector3d::Zero(); // First call
        }

        Eigen::Matrix<double, 4, 1> dq = ov_core::quat_multiply(q_k, ov_core::Inv(q_km1));
        return (2.0 / dt) * dq.head<3>();
    }

    /**
     * @brief Get OpenVINS to FRD coordinate frame transformation matrix
     *
     * This function returns the rotation matrix that transforms from the
     * OpenVINS coordinate frame to the Front-Right-Down (FRD) coordinate frame
     * commonly used in aerospace applications.
     *
     * TODO: Port this for greater possibility of orientations --> case by case etc
     *
     * @return 3x3 rotation matrix from OpenVINS to FRD frame
     */
    inline const Eigen::Matrix3d &R_OV_FRD()
    {
        static const Eigen::Matrix3d R{
            (Eigen::Matrix3d() << 1, 0, 0, 0, -1, 0, 0, 0, -1).finished()};
        return R;
    }

    // ============================================================================
    // PUBLISHER CLASS
    // ============================================================================

    /**
     * @class Publisher
     * @brief Singleton class for publishing VIO data
     *
     * This class manages the publication of VIO data to external systems through
     * pipe interfaces. It implements the singleton pattern to ensure only one
     * instance exists throughout the application lifecycle.
     *
     * The publisher handles:
     * - VIO state data formatting
     * - Track base information
     * - Coordinate frame transformations
     * - Data packet generation and transmission
     */
    class Publisher
    {
    public:
        /**
         * @brief Get singleton instance
         * @return Reference to the singleton Publisher instance
         */
        static Publisher &getInstance()
        {
            static Publisher instance;
            return instance;
        }

        // Delete copy constructor and assignment operator for singleton
        Publisher(const Publisher &) = delete;
        Publisher &operator=(const Publisher &) = delete;

        /**
         * @brief Start the publisher
         *
         * Initializes the publisher and prepares it for data transmission.
         */
        void start();

        /**
         * @brief Control pipe callback function 
         * 
         * Callback function to handle control pipe messages
         */
        static void ov_vio_control_pipe_cb(int ch, char *string, int bytes, void *context);

        /**
         * @brief Publish VIO data
         *
         * Publishes the current VIO state and tracking information to external
         * systems through the configured pipe interfaces.
         *
         * @param state Current VIO state
         * @param trackbase Current tracking information
         * @param corr_mat Correction matrix for coordinate transformations
         */
        void publish(std::shared_ptr<ov_msckf::State> state,
                     Eigen::Matrix3d corr_mat,
                     std::map<double, std::vector<std::shared_ptr<ov_core::Feature>>> used_features_map = {});

        /**
         * @brief Stop the publisher
         *
         * Stops the publisher and cleans up resources.
         */
        void stop();

        /**
         * @brief Check if auto-reset should be triggered
         *
         * Evaluates current VIO state and error conditions to determine
         * if an automatic reset should be triggered.
         *
         * @param state Current VIO state
         * @param quality Current quality value
         * @param n_features Number of tracked features
         * @param V_uncertainty Velocity uncertainty
         * @param yawrate Calculated yaw rate from angular velocity
         * @param current_velocity Current velocity magnitude
         * @param vel_x X-component of velocity
         * @param vel_y Y-component of velocity
         * @return true if auto-reset should be triggered, false otherwise
         */
        bool should_auto_reset(std::shared_ptr<ov_msckf::State> state,
                               int quality,
                               int n_features,
                               double V_uncertainty,
                               double yawrate,
                               double current_velocity,
                               double vel_x,
                               double vel_y);

        /**
         * @brief Calculate Quality of the VIO state
         *
         * This function computes the quality score based on the features used at
         * the current timestamp. The quality calculation considers:
         * 1. Grid distribution: 5x5 grid per camera with 50 features target (2 per grid)
         * 2. SLAM features: weighted by covariance largest eigenvalue and quality field
         * 3. MSCKF features: weighted by quality field and number of measurements
         * 
         * @param used_features_map Map of used features at the current timestamp
         * @param slam_features Map of SLAM features from the state
         * @param state Current VIO state for covariance access
         * @return Quality score (0-100, higher is better)
         */
        double calcQuality(const std::map<double, std::vector<std::shared_ptr<ov_core::Feature>>> &used_features_map, 
                          std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>> &slam_features,
                          std::shared_ptr<ov_msckf::State> state);

        /**
         * @brief Set the first packet flag
         * 
         * This function sets the first packet flag to the provided value.
         * 
         * @param first_packet The value to set the first packet flag to
         */
        void set_first_packet(bool first_packet_){
            first_packet = first_packet_;
        };
    private:
        /**
         * @brief Private constructor for singleton pattern
         */
        Publisher();

        /**
         * @brief Private destructor for singleton pattern
         */
        ~Publisher();

        // ============================================================================
        // PRIVATE MEMBER VARIABLES
        // ============================================================================

        /** @brief Flag indicating if this is the first packet */
        bool first_packet = true;

        /** @brief VIO data packet structure */
        vio_data_t vio_packet;

        /** @brief Previous VIO state for differential calculations */
        std::shared_ptr<ov_msckf::State> past_state;

        /** @brief Previous quaternion for angular velocity calculation */
        Eigen::Matrix<double, 4, 1> past_q_I_G;

        /** @brief Type of angular velocity calculation to use */
        ANG_VEL_TYPE ang_vel_type = ANG_VEL_TYPE::QUAT_DIRTY;
    };

    // ============================================================================
    // HEALTH CHECK CLASS
    // ============================================================================

    /**
     * @class HealthCheck
     * @brief Health monitoring system for VOXL OpenVINS
     *
     * This class provides comprehensive health monitoring capabilities for
     * the VIO system, including error code monitoring, system state checks,
     * and performance monitoring. It runs at 30Hz and continuously monitors
     * the system health.
     */
    class HealthCheck
    {
    public:
        /**
         * @brief Get singleton instance
         * @return Reference to the singleton HealthCheck instance
         */
        static HealthCheck &getInstance()
        {
            static HealthCheck instance;
            return instance;
        }

        // Delete copy constructor and assignment operator for singleton
        HealthCheck(const HealthCheck &) = delete;
        HealthCheck &operator=(const HealthCheck &) = delete;

        /**
         * @brief Start the health check system
         *
         * Initializes and starts the health monitoring thread that runs at 30Hz.
         * The thread continuously monitors system health and error conditions.
         */
        void start();

        /**
         * @brief Stop the health check system
         *
         * Stops the health monitoring thread and performs cleanup.
         */
        void stop();

        /**
         * @brief Check if health monitoring is running
         * @return true if health check is active, false otherwise
         */
        bool isRunning() const { return running_.load(); }

        /**
         * @brief Clear specific error codes
         *
         * Clears the specified error codes from the global error state.
         * This is useful when errors are resolved and should no longer
         * be reported.
         *
         * @param error_mask Bit mask of error codes to clear
         * @param clear_all If true, clear all error codes
         */
        static void clearErrorCodes(uint32_t error_mask){
            clearErrorCodes(error_mask, false);
        }
        static void clearErrorCodes(uint32_t error_mask, bool clear_all);

    private:
        /**
         * @brief Private constructor for singleton pattern
         */
        HealthCheck();

        /**
         * @brief Private destructor for singleton pattern
         */
        ~HealthCheck();

        /**
         * @brief Main health check loop
         *
         * Runs at 30Hz and performs comprehensive health monitoring including:
         * - Error code analysis and logging
         * - System state validation
         * - Performance monitoring
         * - Auto-reset condition checking
         */
        void healthCheckLoop();

        /**
         * @brief Analyze and log error codes
         *
         * Examines the current error codes and logs detailed information
         * about any active errors or warnings.
         */
        void analyzeErrorCodes();

        /**
         * @brief Check system connectivity
         *
         * Monitors the connection status of cameras and IMU, logging
         * any disconnection events or connectivity issues.
         */
        void checkSystemConnectivity();

        /**
         * @brief Monitor system performance
         *
         * Tracks system performance metrics including processing rates,
         * memory usage, and timing statistics.
         */
        void monitorSystemPerformance();

        /**
         * @brief Check auto-reset conditions
         *
         * Evaluates whether auto-reset conditions are met based on
         * current system state and error conditions.
         */
        void checkAutoResetConditions();

        /**
         * @brief Handle VINS reset command
         *
         * This function is invoked when a reset command is received.
         * It performs the necessary actions to reset the VIO system.
         */
        void checkVINSResetRequest();

        /**
         * @brief Perform a hard reset of the VIO system
         * 
         * This function creates a fresh instance of the 
         * VIO manager and reinitializes the system.,
         */
        int doHardReset();

        // ============================================================================
        // PRIVATE MEMBER VARIABLES
        // ============================================================================

        /** @brief Flag indicating if health check is running */
        std::atomic<bool> running_{false};

        /** @brief Health check thread */
        std::thread health_thread_;

        /** @brief Last error code state for change detection */
        uint32_t last_error_codes_{0};

        /** @brief Last VIO state for change detection */
        uint8_t last_vio_state_{0};

        /** @brief Last IMU connection state for change detection */
        bool last_imu_connected_{false};

        /** @brief Last camera connection state for change detection */
        bool last_cam_connected_{false};

        /** @brief Timestamp of last health check */
        int64_t last_health_check_ns_{0};

        /** @brief Counter for health check iterations */
        uint64_t health_check_count_{0};

        /** @brief Mutex for thread safety */
        mutable std::mutex health_mutex_;

        /** @brief Timeout interval before we can reset again */
        const uint64_t INIT_FAILURE_TIMEOUT_NS = 2000000000; // 2 seconds
        
        /** @brief Timestamp of last reset */
        uint64_t time_of_last_reset = 0;

    };

} // namespace voxl

#endif // VOXL_HK_H