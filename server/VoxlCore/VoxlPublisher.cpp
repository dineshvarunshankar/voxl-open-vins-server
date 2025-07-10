/**
 * @file VoxlPublisher.cpp
 * @brief Publisher implementation for VOXL OpenVINS
 * @author Zauberflote
 * @date 2025
 * @version 1.0
 *
 * This file implements the publisher for the VOXL OpenVINS server.
 */

#include "VoxlHK.h"
using namespace voxl;

#define STR_MATCH(s, lit)  (strncmp((s), (lit), strlen(lit)) == 0)

// ============================================================================
// PUBLISHER CLASS IMPLEMENTATION
// ============================================================================

/**
 * @brief Constructor for the Publisher class
 *
 * Initializes the publisher by zeroing out the VIO data packet structure.
 * This ensures that all fields start with known values.
 */
Publisher::Publisher()
{
    // Initialize the publisher
    memset(&vio_packet, 0, sizeof(vio_data_t));
}

/**
 * @brief Destructor for the Publisher class
 *
 * Performs cleanup by calling the stop method to ensure proper
 * resource deallocation.
 */
Publisher::~Publisher()
{
    // Stop the publisher
    stop();
}

/**
 * @brief Start the publisher
 *
 * Initializes the publisher and prepares it for data transmission.
 * Sets the first_packet flag to true to handle the initial angular
 * velocity calculation.
 */
void Publisher::start()
{
    // Initialize publisher if needed
    first_packet = true;

    // Set VIO state to initializing
    vio_state = VIO_STATE_INITIALIZING;

    // Start the health check system
    HealthCheck::getInstance().start();
}

/**
 * @brief Stop the publisher
 *
 * Stops the publisher and cleans up resources. Currently a placeholder
 * for future cleanup operations.
 */
void Publisher::stop()
{
    // Stop the health check system
    HealthCheck::getInstance().stop();

    // Clean up resources if needed
}



/**
 * @brief Control-pipe callback for VIO commands
 *
 * This function is invoked every time a message
 * is received on the VIO **control pipe**.  
 *
 * @param ch      Channel id supplied by the pipe framework.
 * @param string  Pointer to the received buffer.
 * @param bytes   Number of valid bytes in @p string.
 * @param context User context pointer supplied during registration.
 *
 * @note Matching is performed with the `STR_MATCH()` macro, which compares the
 *       prefix of @p string against the command literal.
 */
void Publisher::ov_vio_control_pipe_cb(__attribute__((unused)) int ch, 
                                        char *string,
                                        int bytes, 
                                        __attribute__((unused)) void *context)
{
    if (STR_MATCH(string, RESET_VIO_HARD))
    {

    }
    else if (STR_MATCH(string, RESET_VIO_SOFT))
    {
    
    }
    else
    {
        // Unrecognized command, log an error or handle appropriately
        printf("Unrecognized control command: %.*s\n", bytes, string);
    }
    
    return;
}


/**
 * @brief Publish VIO data to external systems
 *
 * This method formats and publishes the current VIO state and tracking
 * information to external systems through the configured pipe interfaces.
 *
 * The function performs the following operations:
 * - Formats VIO data packet with current state information
 * - Performs coordinate frame transformations (OpenVINS to FRD)
 * - Calculates angular velocity from quaternion differences
 * - Extracts and formats covariance matrices
 * - Handles camera-to-IMU extrinsic parameters
 * - Publishes both simple and extended VIO data packets
 *
 * The coordinate frame transformation involves:
 * - Converting from OpenVINS coordinate frame to Front-Right-Down (FRD)
 * - Handling initialization state with NED rotation zeroing
 * - Applying proper quaternion and rotation matrix transformations
 *
 * @param state Current VIO state containing pose, velocity, and covariance
 * @param trackbase Current tracking information for feature data
 * @param corr_mat Correction matrix for coordinate transformations (currently unused)
 */
void Publisher::publish(std::shared_ptr<ov_msckf::State> state, std::shared_ptr<ov_core::TrackBase> trackbase, Eigen::Matrix3d corr_mat)
{
    vio_packet.magic_number = VIO_MAGIC_NUMBER;
    vio_packet.timestamp_ns = state->_timestamp * 1e9;

    // NOW LET'S DEAL WITH THE ACTUAL STATE
    // RECALL: WE WANT TO EXPRESS THE GLOBAL FRAME IN THE IMU FRAME NOT THE IMU FRAME IN THE GLOBAL FRAME
    // QUICK CLARIFICATION: IF WE WANTED PURELY IMU FRAME ESTIMATES, THEN P = 0, HENCE, WE REALLY WANT THE ABOVE-MENTIONED!

    // LET'S GRAB THE QUATERNIONS FROM THE STATE: IN LATEX: {I}q_{G}
    // USING FEJ -- LESS NOISY BUT "DELAYED" --> NOT A PROBLEM DUE TO IMU RATE
    Eigen::Matrix<double, 4, 1> q_I_G = state->_imu->quat_fej();
    // TODO: Validate quat_fej() vs. quat() usage
    //  Eigen::Matrix<double, 4, 1> q_I_G = state->_imu->quat();

    // NOW DEAL WITH VELOCITY AND POSITION FROM THE STATE: IN LATEX: {G}p_{I} AND {G}v_{I}
    // GLOBAL VELOCITY IN IMU FRAME FOLLOWS: v_I = {I}q_{G} \otimes v_G \otimes {G}q_{I}
    Eigen::Matrix3d R_I_G = ov_core::quat_2_Rot(q_I_G);
    auto RPY = ov_core::rot2rpy(R_I_G);
    RPY(0) = -RPY(0);
    RPY(1) = -M_PI + RPY(1);
    RPY(2) = +M_PI - RPY(2);
    R_I_G = ov_core::rot_x(RPY(0)) * ov_core::rot_y(RPY(1)) * ov_core::rot_z(RPY(2));

    // NOW CONVERT IT TO FRD FRAME
    auto ov2frd = R_OV_FRD(); // TODO: PASS AN ARG FOR HINTING THE RIGHT BOARD ORIENTATION ETC
    // GLOBAL VELOCITY IN IMU AXIS:
    Eigen::Matrix<double, 3, 1> v_I_G = ov2frd * state->_imu->vel();
    // GLOBAL POSITION IN IMU AXIS:
    Eigen::Matrix<double, 3, 1> p_I_G = ov2frd * (state->_imu->pos());
    if (vio_manager->initialized())
    {
        static Eigen::Matrix<double, 3, 3> ned_rot_zero = R_I_G;
        R_I_G = R_I_G * ned_rot_zero.transpose();
        v_I_G = ned_rot_zero * v_I_G;
        p_I_G = ned_rot_zero * p_I_G;

        // Set VIO state to OK when system is initialized
        if (vio_state.load() == VIO_STATE_INITIALIZING)
        {
            vio_state = VIO_STATE_OK;
        }
    }
    // FILL IN THE VIO PACKET - Fix casting issues
    for (int i = 0; i < 3; i++)
    {
        vio_packet.T_imu_wrt_vio[i] = static_cast<float>(p_I_G(i));
        vio_packet.vel_imu_wrt_vio[i] = static_cast<float>(v_I_G(i));
    }

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            vio_packet.R_imu_to_vio[i][j] = static_cast<float>(R_I_G(i, j));
        }
    }
    q_I_G = ov_core::rot_2_quat(R_I_G);

    // NOW LET'S HANDLE THE ANGULAR VELOCITY
    if (first_packet)
    {

        for (int i = 0; i < 3; i++)
        {
            vio_packet.imu_angular_vel[i] = 0.0f;
        }
        past_q_I_G = q_I_G;
    }
    else
    {
        Eigen::Matrix<double, 3, 1> ang_vel_imu = dirtyOmega(q_I_G, past_q_I_G, state->_timestamp - past_state->_timestamp);
        for (int i = 0; i < 3; i++)
        {
            vio_packet.imu_angular_vel[i] = static_cast<float>(ang_vel_imu(i));
        }
        past_q_I_G = q_I_G;
    }

    // NOW HANDLE THE COVARIANCE, HAS TO BE DONE THIS WAY FOR MAVLINK
    std::vector<std::shared_ptr<ov_type::Type>> statevars;
    statevars.push_back(state->_imu->p());
    statevars.push_back(state->_imu->q());
    statevars.push_back(state->_imu->v());
    Eigen::Matrix<double, 9, 9> covariance_posori =
        ov_msckf::StateHelper::get_marginal_covariance(state,
                                                       statevars);

    // Fill covariances (upper triangular format)
    vio_packet.pose_covariance[0] = static_cast<float>(covariance_posori(0, 0));
    vio_packet.pose_covariance[6] = static_cast<float>(covariance_posori(1, 1));
    vio_packet.pose_covariance[11] = static_cast<float>(covariance_posori(2, 2));
    vio_packet.pose_covariance[15] = static_cast<float>(covariance_posori(3, 3));
    vio_packet.pose_covariance[18] = static_cast<float>(covariance_posori(4, 4));
    vio_packet.pose_covariance[20] = static_cast<float>(covariance_posori(5, 5));
    vio_packet.velocity_covariance[0] = static_cast<float>(covariance_posori(6, 6));
    vio_packet.velocity_covariance[6] = static_cast<float>(covariance_posori(7, 7));
    vio_packet.velocity_covariance[11] = static_cast<float>(covariance_posori(8, 8));

    // GRAVITY VECTOR
    float grav_vec[3] = {0, 0, static_cast<float>(9.81)}; // CHECK THIS VALUE OR CALCULATE IT BY MEASURING THE GRAVITY VECTOR
    memcpy(vio_packet.gravity_vector, grav_vec, sizeof(float) * 3);

    // NOW LET'S HANDLE THE EXTRINSICS CAMERA TO IMU
    Eigen::Matrix3d cam_out = ov_core::quat_2_Rot(state->_calib_IMUtoCAM[0]->quat()).transpose();
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            vio_packet.R_cam_to_imu[i][j] = static_cast<float>(cam_out(i, j));
        }
    }

    Eigen::Vector3d t_cam_wrt_imu = -(ov_core::quat_2_Rot(state->_calib_IMUtoCAM[0]->quat()).transpose() * state->_calib_IMUtoCAM[0]->pos());
    for (int i = 0; i < 3; i++)
    {
        vio_packet.T_cam_wrt_imu[i] = static_cast<float>(t_cam_wrt_imu(i));
    }

    // GYRO AND ACCELEROMETER BIAS --> NOT USING FEJ HERE...
    //  Note: These fields don't exist in the standard vio_data_t struct
    //  for(int i = 0; i < 3; i++) {
    //      vio_packet.gyro_bias[i] = static_cast<float>(state->_imu->bias_g()(i));
    //      vio_packet.accl_bias[i] = static_cast<float>(state->_imu->bias_a()(i));
    //  }
    // ERROR CODE - Update atomic variable and copy to packet
    // Check for covariance issues (negative diagonal elements)
    if (covariance_posori(3, 3) < 0.0 || covariance_posori(4, 4) < 0.0 || covariance_posori(5, 5) < 0.0)
    {
        fprintf(stderr, "ERROR: covariance diagonal went negative\n");
        vio_error_codes |= ERROR_CODE_COVARIANCE;
    }

    // Check for timestamp issues (packets from the past)
    static int64_t last_sent_timestamp_ns = 0;
    if (vio_packet.timestamp_ns < last_sent_timestamp_ns)
    {
        if (first_packet)
        {
            // During first packet, just update the timestamp without error
            first_packet = false;
        }
        else
        {
            // Only flag error if timestamp is significantly in the past (more than 1ms)
            int64_t time_diff = last_sent_timestamp_ns - vio_packet.timestamp_ns;
            if (time_diff > 1000000)
            { // 1ms in nanoseconds
                fprintf(stderr, "WARNING: skipping pose data from the past %ld %ld (diff: %ld ns)\n",
                        vio_packet.timestamp_ns, last_sent_timestamp_ns, time_diff);
                printf("[DEBUG-HK] Setting ERROR_CODE_BAD_TIMESTAMP in VIO packet\n");
                vio_error_codes |= ERROR_CODE_BAD_TIMESTAMP;
            }
        }
    }
    last_sent_timestamp_ns = vio_packet.timestamp_ns;

    // Check for velocity uncertainty issues
    double V_uncertainty = 0.0;
    V_uncertainty += covariance_posori(6, 6) * covariance_posori(6, 6);
    V_uncertainty += covariance_posori(7, 7) * covariance_posori(7, 7);
    V_uncertainty += covariance_posori(8, 8) * covariance_posori(8, 8);
    V_uncertainty = sqrt(V_uncertainty);

    if (is_armed && V_uncertainty > auto_reset_max_v_cov_instant)
    {
        fprintf(stderr, "ERROR: exceeded velocity uncertainty threshold %f vs %f\n",
                V_uncertainty, auto_reset_max_v_cov_instant);
        vio_error_codes |= ERROR_CODE_VEL_INST_CERT;
    }

    // Check for excessive velocity
    double current_velocity = state->_imu->vel().norm();
    if (current_velocity > auto_reset_max_velocity)
    {
        fprintf(stderr, "ERROR: exceeded maximum velocity %f vs %f\n",
                current_velocity, auto_reset_max_velocity);
        vio_error_codes |= ERROR_CODE_VEL_WINDOW_CERT;
    }

    // Check for insufficient features
    if (vio_packet.n_feature_points < auto_reset_min_features)
    {
        static int64_t last_good_feat_ts = 0;
        static bool wait_for_features = true;

        if (wait_for_features)
        {
            if (vio_packet.n_feature_points > 3)
            {
                last_good_feat_ts = vio_packet.timestamp_ns;
                wait_for_features = false;
            }
        }
        else
        {
            if (vio_packet.n_feature_points > auto_reset_min_features)
            {
                last_good_feat_ts = vio_packet.timestamp_ns;
            }

            double ts = (vio_packet.timestamp_ns - last_good_feat_ts) * 1e-9;
            if (ts > auto_reset_min_feature_timeout_s)
            {
                fprintf(stderr, "ERROR: insufficient features for too long! cur: %d, min_req: %d\n",
                        vio_packet.n_feature_points, auto_reset_min_features);
                vio_error_codes |= ERROR_CODE_NO_FEATURES;
                wait_for_features = true;
            }
        }
    }

    // Check for quality issues
    static int64_t last_good_qual_ts = 0;
    double ts_threshold = auto_reset_max_v_cov_timeout_s;

    if (vio_packet.quality >= 1)
    {
        last_good_qual_ts = vio_packet.timestamp_ns;
    }

    double ts = (vio_packet.timestamp_ns - last_good_qual_ts) * 1e-9;
    if (ts > ts_threshold)
    {
        fprintf(stderr, "ERROR: quality was bad for too long!\n");
        vio_error_codes |= ERROR_CODE_NOT_STATIONARY;
    }

    // Check for fast yaw changes (spinning in place)
    static int64_t start_spin_time = 0;
    static bool spinning_detected = false;

    double yawrate = vio_packet.imu_angular_vel[2];
    bool spinning_in_place = (fabs(yawrate) > fast_yaw_thresh &&
                              fabs(vio_packet.vel_imu_wrt_vio[0]) <= 1.0 &&
                              fabs(vio_packet.vel_imu_wrt_vio[1]) <= 1.0);

    if (!spinning_in_place)
    {
        start_spin_time = vio_packet.timestamp_ns;
        spinning_detected = false;
    }
    else if (!spinning_detected)
    {
        double spin_duration = (vio_packet.timestamp_ns - start_spin_time) * 1e-9;
        if (spin_duration > fast_yaw_timeout_s)
        {
            fprintf(stderr, "ERROR: exceeded spin rate over time threshold %f!\n", fast_yaw_timeout_s);
            vio_error_codes |= ERROR_CODE_IMU_OOB;
            spinning_detected = true;
        }
    }

    // Copy error codes from atomic variable to packet
    vio_packet.error_code = vio_error_codes.load();

    // QUALITY CALCULATION
    // Calculate uncertainty metrics
    double T_uncertainty = 0.0;
    T_uncertainty += covariance_posori(0, 0) * covariance_posori(0, 0);
    T_uncertainty += covariance_posori(1, 1) * covariance_posori(1, 1);
    T_uncertainty += covariance_posori(2, 2) * covariance_posori(2, 2);
    T_uncertainty = sqrt(T_uncertainty);

    double R_uncertainty = 0.0;
    R_uncertainty += covariance_posori(3, 3) * covariance_posori(3, 3);
    R_uncertainty += covariance_posori(4, 4) * covariance_posori(4, 4);
    R_uncertainty += covariance_posori(5, 5) * covariance_posori(5, 5);
    R_uncertainty = sqrt(R_uncertainty);

    // Map velocity uncertainty to quality (0-100 scale)
    double v_cov_quality = 100.0 - (V_uncertainty / auto_reset_max_v_cov_instant) * 100.0;
    v_cov_quality = std::max(0.0, std::min(100.0, v_cov_quality));

    // Calculate position-based quality
    double max_allowable_cep = 0.1; // 10cm CEP threshold
    double pos_quality = 100.0 - (T_uncertainty / max_allowable_cep) * 100.0;
    pos_quality = std::max(0.0, std::min(100.0, pos_quality));

    // Use the lower of the two qualities
    vio_packet.quality = static_cast<int32_t>(std::min(v_cov_quality, pos_quality));

    // Ensure quality is within bounds
    if (vio_packet.quality > 100)
        vio_packet.quality = 100;
    if (vio_packet.quality < 0)
        vio_packet.quality = 0;

    // Check for auto-reset conditions
    if (should_auto_reset(state, vio_packet.quality, vio_packet.n_feature_points, V_uncertainty, yawrate, current_velocity, vio_packet.vel_imu_wrt_vio[0], vio_packet.vel_imu_wrt_vio[1]))
    {
        fprintf(stderr, "WARNING: Auto-reset conditions detected! Quality: %d, Features: %d, V_uncertainty: %f\n",
                vio_packet.quality, vio_packet.n_feature_points, V_uncertainty);
        vio_packet.quality = -1;
        vio_packet.state = VIO_STATE_FAILED;
        vio_state = VIO_STATE_FAILED;
        // Note: Actual reset logic would be handled by the main VIO manager
    }
    else
    {
        vio_packet.state = VIO_STATE_OK;
        vio_state = VIO_STATE_OK;
    }

    // FRAME
    vio_packet.frame = 0; // Set appropriate frame value

    // NUMBER OF FEATURE POINTS
    // FOR NOW, WE ONLY CONSIDER SLAM FEATURES, AS THESE ARE THE ONLY ONES IN THE STATE
    std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>> SLAM_FEATS = state->_features_SLAM;
    vio_packet.n_feature_points = static_cast<uint16_t>(SLAM_FEATS.size());

    past_state = state;

    // publish the packet
    //  if (pipe_server_get_num_clients(SIMPLE_CH) > 0)
    pipe_server_write(SIMPLE_CH, (char *)&vio_packet, sizeof(vio_data_t));

    if (pipe_server_get_num_clients(EXTENDED_CH) > 0)
    {
        std::unordered_map<size_t, std::vector<cv::KeyPoint>> pts = trackbase->get_last_obs();
        std::unordered_map<size_t, std::vector<size_t>> ids = trackbase->get_last_ids();

        ext_vio_data_t ext_vio_packet;
        ext_vio_packet.v = vio_packet;
        ext_vio_packet.n_total_features = 0;

        for (const auto &pair : pts)
        {
            int cam_id = pair.first;
            const auto &kp_vec = pts[cam_id];
            const auto &id_vec = ids[cam_id];

            Eigen::Matrix3d R_I_C = ov_core::quat_2_Rot(state->_calib_IMUtoCAM[cam_id]->quat()).transpose();

            for (size_t i_local = 0; i_local < id_vec.size(); i_local++)
            {
                size_t id = id_vec[i_local];

                if (SLAM_FEATS.find(id) != SLAM_FEATS.end())
                {
                    if (ext_vio_packet.n_total_features >= VIO_MAX_REPORTED_FEATURES)
                        break;

                    int i_global = ext_vio_packet.n_total_features++;

                    ext_vio_packet.features[i_global].id = id;
                    ext_vio_packet.features[i_global].cam_id = cam_id;

                    ext_vio_packet.features[i_global].pix_loc[0] = pts[cam_id][i_local].pt.x;
                    ext_vio_packet.features[i_global].pix_loc[1] = pts[cam_id][i_local].pt.y;

                    Eigen::Vector3d p_FinG;
                    if (SLAM_FEATS[id]->_feat_representation == ov_type::LandmarkRepresentation::Representation::GLOBAL_3D)
                    {
                        p_FinG = SLAM_FEATS[id]->get_xyz(true); // GRAB FEJ VALUE --> AVOID SNAP BACK EFFECT IN VIZ
                        p_FinG = ov2frd * p_FinG;
                    }
                    else if (SLAM_FEATS[id]->_feat_representation == ov_type::LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH)
                    {
                        // FIX THIS --> THIS IS INCORRECT, NEED TO GRAB ROTATIONS AT ANCHOR FRAME
                        Eigen::Vector3d p_FinA = SLAM_FEATS[id]->get_xyz(false);
                        Eigen::Vector3d T_I_C(0, 0, 0); // translation from imu to camera position
                        p_FinG = R_I_G * (R_I_C * p_FinA + T_I_C) + p_I_G;
                    }
                    else
                    {
                        printf("[WARNING] feat representation not recognized, 3D point locations are likely invalid\n");
                    }

                    ext_vio_packet.features[i_global].tsf[0] = p_FinG[0];
                    ext_vio_packet.features[i_global].tsf[1] = p_FinG[1];
                    ext_vio_packet.features[i_global].tsf[2] = p_FinG[2];
                }
            }
        }

        pipe_server_write(EXTENDED_CH, (char *)&ext_vio_packet, sizeof(ext_vio_data_t));
    }
}

/**
 * @brief Check if auto-reset should be triggered
 *
 * This function evaluates the current VIO state and various error conditions
 * to determine if an automatic reset should be triggered. It implements the
 * same logic as the legacy code but in a more modular way.
 *
 * @param state Current VIO state
 * @param quality Current quality value
 * @param n_features Number of tracked features
 * @param V_uncertainty Velocity uncertainty
 * @return true if auto-reset should be triggered, false otherwise
 */
bool Publisher::should_auto_reset(std::shared_ptr<ov_msckf::State> state,
                                  int quality,
                                  int n_features,
                                  double V_uncertainty,
                                  double yawrate,
                                  double current_velocity,
                                  double vel_x,
                                  double vel_y)
{
    // Only check for auto-reset if enabled and system is stable
    if (!en_auto_reset)
    {
        return false;
    }

    // Check if VIO manager is in a bad state
    bool vio_manager_bad = false; // FUTURE IMPLEMENTATION WITH SfM, for now just auto false

    // Check quality conditions
    bool quality_bad = quality < 1;
    bool stable_quality_bad = false;

    // Check for stable quality issues (quality bad for extended period)
    static int64_t last_good_qual_ts = 0;
    if (quality >= 1)
    {
        last_good_qual_ts = state->_timestamp * 1e9;
    }
    double ts = (state->_timestamp * 1e9 - last_good_qual_ts) * 1e-9;
    if (ts > auto_reset_max_v_cov_timeout_s)
    {
        stable_quality_bad = true;
    }

    // Check feature conditions
    bool stable_features_bad = false;
    static int64_t last_good_feat_ts = 0;
    static bool wait_for_features = true;

    if (wait_for_features)
    {
        if (n_features > 3)
        {
            last_good_feat_ts = state->_timestamp * 1e9;
            wait_for_features = false;
        }
    }
    else
    {
        if (n_features > auto_reset_min_features)
        {
            last_good_feat_ts = state->_timestamp * 1e9;
        }

        double ts = (state->_timestamp * 1e9 - last_good_feat_ts) * 1e-9;
        if (ts > auto_reset_min_feature_timeout_s)
        {
            stable_features_bad = true;
            wait_for_features = true;
        }
    }

    // Check velocity conditions using passed values
    bool too_fast = current_velocity > auto_reset_max_velocity;
    bool too_uncertain = is_armed && V_uncertainty > auto_reset_max_v_cov_instant;

    // Check for excessive spinning using passed yawrate
    bool too_much_spinning = false;
    static int64_t start_spin_time = 0;
    static bool spinning_detected = false;

    // Use the passed yawrate value and actual velocity components
    bool spinning_in_place = (fabs(yawrate) > fast_yaw_thresh &&
                              fabs(vel_x) <= 1.0 &&
                              fabs(vel_y) <= 1.0);

    if (!spinning_in_place)
    {
        start_spin_time = state->_timestamp * 1e9;
        spinning_detected = false;
    }
    else if (!spinning_detected)
    {
        double spin_duration = (state->_timestamp * 1e9 - start_spin_time) * 1e-9;
        if (spin_duration > fast_yaw_timeout_s)
        {
            too_much_spinning = true;
            spinning_detected = true;
        }
    }

    // Return true if any condition is met
    return vio_manager_bad || quality_bad || stable_quality_bad ||
           stable_features_bad || too_fast || too_uncertain || too_much_spinning;
}