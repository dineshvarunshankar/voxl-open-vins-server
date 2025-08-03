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
        if (reset_requested.load() || is_resetting.load())
        {
            fprintf(stderr, "[WARNING] Already resetting VIO, ignoring hard reset command.\n");
            return;
        }

        reset_requested.store(true);
		fprintf(stderr, "[ERROR] Client requested hard reset\n");
        return;
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
 */
void Publisher::publish(std::shared_ptr<ov_msckf::State> state,
                        std::map<double, std::vector<std::shared_ptr<ov_core::Feature>>> used_features_map)
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
    // printf("gravity axis: %d, gravity direction: %d\n", static_cast<int>(frame_transform.gravity_axis), static_cast<int>(frame_transform.gravity_direction));
    //AT THIS POINT, BETTER TO ROTATE IT USING THE CORRECTION MATRIX...
    //EXECUTE THE FORBIDDEN TECHNIQUE:
    //CHECK IF THE IMU IS MOUNTED IN THE CORRECT WAY, I.E., 
    //GRAVITY IS POINTING DOWN Z
    //IF NOT, EXECUTE THE FORBIDDEN TECHNIQUE:
    float grav_vec[3];
    if (frame_transform.gravity_axis == FrameTransform::Axis::Z && frame_transform.gravity_direction == FrameTransform::Direction::POSITIVE){
        //FORBIDDEN TECHNIQUE (STINGER CASE)
        RPY(0) = -RPY(0);
        RPY(1) = M_PI - RPY(1);
        RPY(2) = -M_PI + RPY(2);
        R_I_G = ov_core::rot_x(RPY(0)) * ov_core::rot_y(RPY(1)) * ov_core::rot_z(RPY(2));
        // GRAVITY VECTOR
        grav_vec[0] = 0;
        grav_vec[1] = 0;
        grav_vec[2] = static_cast<float>(-9.81); // CHECK THIS VALUE OR CALCULATE IT BY MEASURING THE GRAVITY VECTOR
    } else {
        //CLASSIC CASE: STARLING2, STARLING MAX, D8V4, D8V5
        RPY(0) = -RPY(0);
        RPY(1) = -M_PI + RPY(1);
        RPY(2) = M_PI - RPY(2);
        R_I_G = ov_core::rot_x(RPY(0)) * ov_core::rot_y(RPY(1)) * ov_core::rot_z(RPY(2));
        // GRAVITY VECTOR
        grav_vec[0] = 0;
        grav_vec[1] = 0;
        grav_vec[2] = static_cast<float>(9.81); // CHECK THIS VALUE OR CALCULATE IT BY MEASURING THE GRAVITY VECTOR
    }
    memcpy(vio_packet.gravity_vector, grav_vec, sizeof(float) * 3);

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
    alt_z.store(static_cast<float>(p_I_G(2)), std::memory_order_release);
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
    // Get SLAM features from state
    std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>> SLAM_FEATS = state->_features_SLAM;
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

    // Get used features map from VioManager if not provided and NOT RESETTING!!
    double calculated_quality;
    if (!is_resetting.load(std::memory_order_relaxed) && vio_manager->initialized())
    {

        // Calculate quality using the new feature-based method
        calculated_quality = calcQuality(used_features_map, SLAM_FEATS, state);
    }
    else
    {
        // Map velocity uncertainty to quality (0-100 scale)
        double v_cov_quality = 100.0 - (V_uncertainty / auto_reset_max_v_cov_instant) * 100.0;
    

        // Calculate position-based quality
        double max_allowable_cep = 0.1; // 10cm CEP threshold
        double pos_quality = 100.0 - (T_uncertainty / max_allowable_cep) * 100.0;
        pos_quality = std::max(0.0, std::min(100.0, pos_quality));
        calculated_quality = std::min(v_cov_quality, pos_quality);
    }
    
    // Set the quality in the packet
    vio_packet.quality = static_cast<int32_t>(calculated_quality);



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
    vio_packet.n_feature_points = static_cast<uint16_t>(SLAM_FEATS.size());

    past_state = state;

    // publish the packet
    //  if (pipe_server_get_num_clients(SIMPLE_CH) > 0)
    pipe_server_write(SIMPLE_CH, (char *)&vio_packet, sizeof(vio_data_t));

    if (pipe_server_get_num_clients(EXTENDED_CH) > 0)
    {
        ext_vio_data_t ext_vio_packet;
        ext_vio_packet.v = vio_packet;
        ext_vio_packet.n_total_features = 0;

        double current_timestamp = state->_timestamp;
        auto timestamp_iter = used_features_map.find(current_timestamp);
        
        if (timestamp_iter != used_features_map.end())
        {
            const auto &features = timestamp_iter->second;
        
            // Group features by camera ID
            std::map<int, std::vector<std::shared_ptr<ov_core::Feature>>> features_by_camera;
            for (const auto &feature : features)
            {
                // Get camera ID from feature measurements
                for (const auto &cam_meas : feature->timestamps)
                {
                    int cam_id = static_cast<int>(cam_meas.first);
                    features_by_camera[cam_id].push_back(feature);
                }
            }
            
            ov_type::LandmarkRepresentation::Representation msckf_ref = state->_options.feat_rep_msckf;
            
            // Process each camera's features
            for (const auto &camera_pair : features_by_camera)
            {
                int cam_id = camera_pair.first;
                const auto &cam_features = camera_pair.second;

                Eigen::Matrix3d R_I_C = ov_core::quat_2_Rot(state->_calib_IMUtoCAM[cam_id]->quat()).transpose();
                Eigen::Vector3d p_I_C = state->_calib_IMUtoCAM[cam_id]->pos();

                for (const auto &feature : cam_features)
                {
                    // Check if this feature has measurements for this camera at current timestamp
                    if (feature->timestamps.find(cam_id) == feature->timestamps.end() ||
                        feature->uvs.find(cam_id) == feature->uvs.end())
                    {
                        continue;
                    }

                    // Find the measurement index for the current timestamp
                    const auto &timestamps = feature->timestamps.at(cam_id);
                    auto time_iter = std::find(timestamps.begin(), timestamps.end(), current_timestamp);
                    if (time_iter == timestamps.end())
                    {
                        continue;
                    }
                    
                    size_t meas_idx = std::distance(timestamps.begin(), time_iter);
                    const auto &uv_meas = feature->uvs.at(cam_id)[meas_idx];


                    // check if feature id found in SLAM set
                    if (SLAM_FEATS.find(feature->featid) != SLAM_FEATS.end())
                    {
                        if (ext_vio_packet.n_total_features >= VIO_MAX_REPORTED_FEATURES)
                            break;

                        int i_global = ext_vio_packet.n_total_features++;

                        ext_vio_packet.features[i_global].id = feature->featid;
                        ext_vio_packet.features[i_global].cam_id = cam_id;

                        // Extract pixel location from UV measurement
                        ext_vio_packet.features[i_global].pix_loc[0] = uv_meas(0);
                        ext_vio_packet.features[i_global].pix_loc[1] = uv_meas(1);

                        Eigen::Vector3d p_FinG;
                        if (SLAM_FEATS[feature->featid]->_feat_representation == ov_type::LandmarkRepresentation::Representation::GLOBAL_3D)
                        {
                            p_FinG = SLAM_FEATS[feature->featid]->get_xyz(true); // GRAB FEJ VALUE --> AVOID SNAP BACK EFFECT IN VIZ
                            p_FinG = ov2frd * p_FinG;
                        }
                        else if (SLAM_FEATS[feature->featid]->_feat_representation == ov_type::LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH)
                        {
                            // FIX THIS --> THIS IS INCORRECT, NEED TO GRAB ROTATIONS AT ANCHOR FRAME
                            Eigen::Vector3d p_FinA = SLAM_FEATS[feature->featid]->get_xyz(false);
                            Eigen::Vector3d T_I_C(0, 0, 0); // translation from imu to camera position
                            p_FinG = R_I_G * (R_I_C * p_FinA + p_I_C) + p_I_G;
                        }
                        else
                        {
                            printf("[WARNING] SLAM feat representation: %d not recognized, 3D point locations are likely invalid\n", SLAM_FEATS[feature->featid]->_feat_representation);
                        }

                        ext_vio_packet.features[i_global].tsf[0] = p_FinG[0];
                        ext_vio_packet.features[i_global].tsf[1] = p_FinG[1];
                        ext_vio_packet.features[i_global].tsf[2] = p_FinG[2];

                        ext_vio_packet.features[i_global].point_quality = HIGH;
                    }
                    // if not found in SLAM, then it is an MSCKF feature
                    else
                    {
                        if (ext_vio_packet.n_total_features >= VIO_MAX_REPORTED_FEATURES)
                            break;

                        int i_global = ext_vio_packet.n_total_features++;

                        ext_vio_packet.features[i_global].id = feature->featid;
                        ext_vio_packet.features[i_global].cam_id = cam_id;

                        // Extract pixel location from UV measurement
                        ext_vio_packet.features[i_global].pix_loc[0] = uv_meas(0);
                        ext_vio_packet.features[i_global].pix_loc[1] = uv_meas(1);

                        Eigen::Vector3d p_FinA = feature->p_FinA;
                        Eigen::Vector3d p_FinG = feature->p_FinG;

                        if (msckf_ref == ov_type::LandmarkRepresentation::Representation::GLOBAL_3D)
                        {
                            p_FinG = ov2frd * p_FinG;
                        }
                        else if (msckf_ref == ov_type::LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH)
                        {
                            // FIX THIS --> THIS IS INCORRECT, NEED TO GRAB ROTATIONS AT ANCHOR FRAME
                            p_FinG = R_I_G * (R_I_C * p_FinA + p_I_C) + p_I_G;
                        }
                        else
                        {
                            printf("[WARNING] MSCKF feat representation: %s not recognized, 3D point locations are likely invalid\n", ov_type::LandmarkRepresentation::as_string(msckf_ref).c_str());
                        }
                        ext_vio_packet.features[i_global].tsf[0] = p_FinG[0];
                        ext_vio_packet.features[i_global].tsf[1] = p_FinG[1];
                        ext_vio_packet.features[i_global].tsf[2] = p_FinG[2];

                        ext_vio_packet.features[i_global].point_quality = MEDIUM;
                    }
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


/**
 * @brief Calculate Quality of the VIO state
 *
 * This function computes the quality score based on the features used at
 * the current timestamp. The quality calculation considers:
 * 1. Grid distribution: 5x5 grid per camera with 50 features target (2 per grid)
 * 2. Active SLAM features: weighted by covariance largest eigenvalue and quality field
 * 3. MSCKF features: weighted by quality field and number of measurements
 * 
 * @param used_features_map Map of used features at the current timestamp
 * @param slam_features Map of SLAM features from the state
 * @param state Current VIO state for covariance access
 * @return Quality score (0-100, higher is better)
 */
double Publisher::calcQuality(const std::map<double, std::vector<std::shared_ptr<ov_core::Feature>>> &used_features_map, 
                             std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>> &slam_features,
                             std::shared_ptr<ov_msckf::State> state)
{
    // Constants for quality calculation
    const int GRID_SIZE = 5;  // 5x5 grid
    const int TARGET_FEATURES_PER_CAM = 50;  // 50 features per camera
    const int TARGET_FEATURES_PER_GRID = TARGET_FEATURES_PER_CAM / (GRID_SIZE * GRID_SIZE);  // 2 features per grid
    const int CLONES = 11;  // Number of clones for MSCKF weighting
    const double MAX_GRID_SCORE = 100.0;  // Maximum score when 50% of grids are filled
    const double GRID_FILL_THRESHOLD = 0.5;  // 50% of grids should be filled for max score
    const int en_qual_debug = 0;
    
    double total_quality = 0.0;
    int total_grids_filled = 0;
    int total_grids = 0;
    
    // Debug: Print total features available
    if (en_qual_debug) {
        printf("[QUALITY_DEBUG] Total features in used_features_map: %zu\n", used_features_map.size());
        printf("[QUALITY_DEBUG] Total SLAM features in state: %zu\n", slam_features.size());
    }
    
    // Get the current timestamp to find the most recent features
    double current_timestamp = state->_timestamp;
    if (en_qual_debug) printf("[QUALITY_DEBUG] Current timestamp: %.6f\n", current_timestamp);
    
    // Find the features for the current timestamp
    auto timestamp_iter = used_features_map.find(current_timestamp);
    if (timestamp_iter == used_features_map.end()) {
        // No features for current timestamp, return low quality
        if (en_qual_debug) printf("[QUALITY_DEBUG] No features found for current timestamp, returning low quality\n");
        return 10.0;
    }
    
    const auto &features = timestamp_iter->second;
    if (en_qual_debug) printf("[QUALITY_DEBUG] Features found for current timestamp: %zu\n", features.size());
    
    // Group features by camera ID
    std::map<int, std::vector<std::shared_ptr<ov_core::Feature>>> features_by_camera;
    for (const auto &feature : features) {
        // Get camera ID from feature measurements
        for (const auto &cam_meas : feature->timestamps) {
            int cam_id = static_cast<int>(cam_meas.first);
            features_by_camera[cam_id].push_back(feature);
        }
    }
    
    if (en_qual_debug) 
    {
        printf("[QUALITY_DEBUG] Features grouped by camera:\n");
        for (const auto &cam_pair : features_by_camera) {
            printf("[QUALITY_DEBUG]   Camera %d: %zu features\n", cam_pair.first, cam_pair.second.size());
        }
    }
    
    // Count active SLAM and MSCKF features
    int total_active_slam = 0;
    int total_active_msckf = 0;
    
    // Process each camera's features
    for (const auto &camera_pair : features_by_camera) {
        int cam_id = camera_pair.first;
        const auto &cam_features = camera_pair.second;
        
        if (en_qual_debug) printf("[QUALITY_DEBUG] Processing camera %d with %zu features\n", cam_id, cam_features.size());
        
        // Initialize 5x5 grid for this camera
        std::vector<std::vector<int>> grid(GRID_SIZE, std::vector<int>(GRID_SIZE, 0));
        std::vector<std::vector<double>> grid_quality(GRID_SIZE, std::vector<double>(GRID_SIZE, 0.0));
        
        int cam_slam_count = 0;
        int cam_msckf_count = 0;
        
        // Process features for this camera
        for (const auto &feature : cam_features) {
            // Get feature position in image for this camera
            if (feature->timestamps.find(cam_id) != feature->timestamps.end() && 
                !feature->uvs_norm.empty() && 
                feature->uvs_norm.find(cam_id) != feature->uvs_norm.end() &&
                !feature->uvs_norm.at(cam_id).empty()) {
                
                const auto &uv_norm = feature->uvs_norm.at(cam_id).back();
                if (en_qual_debug) printf("[QUALITY_DEBUG] Feature %zu - Camera %d - UV Norm: (%.3f, %.3f)\n", 
                                            feature->featid, cam_id, uv_norm(0), uv_norm(1));
                
                // Convert normalized coordinates to grid coordinates
                // Assuming normalized coordinates are in [-1, 1] range
                int grid_x = static_cast<int>((uv_norm(0) + 1.0) * 0.5 * GRID_SIZE);
                int grid_y = static_cast<int>((uv_norm(1) + 1.0) * 0.5 * GRID_SIZE);
                
                // Clamp to valid grid range
                grid_x = std::max(0, std::min(GRID_SIZE - 1, grid_x));
                grid_y = std::max(0, std::min(GRID_SIZE - 1, grid_y));
                
                // Calculate feature quality based on whether it's SLAM or MSCKF
                double feature_quality = 0.0;
                
                // Check if this is a SLAM feature
                if (slam_features.find(feature->featid) != slam_features.end()) {
                    // This is an active SLAM feature
                    cam_slam_count++;
                    auto slam_feat = slam_features[feature->featid];
                    
                    if (en_qual_debug) printf("[QUALITY_DEBUG] Feature %zu (SLAM) - Grid[%d][%d] - Quality field: %.3f, uniq_cam: %d, anchor_cam: %d\n", 
                                                feature->featid, grid_x, grid_y, slam_feat->_quality, slam_feat->_unique_camera_id, slam_feat->_anchor_cam_id);
                    //ATTENTION: COULD SOLVE THIS IN A BATCH MANNER, BUT FOR NOW DO THE FOR LOOP DIRECTLY
                    // Get covariance for this SLAM feature
                    Eigen::MatrixXd slam_cov;
                    if (slam_feat->id() >= 0) {
                        // Get the covariance block for this feature
                        std::vector<std::shared_ptr<ov_type::Type>> slam_feat_vec = {slam_feat};
                        slam_cov = ov_msckf::StateHelper::get_marginal_covariance(state, slam_feat_vec);
                    } else {
                        // Feature not in state yet, use default covariance
                        slam_cov = Eigen::MatrixXd::Identity(3, 3) * 0.1; // Default 10cm uncertainty
                    }
                    
                    // Calculate largest eigenvalue of covariance
                    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(slam_cov);
                    double max_eigenvalue = eigensolver.eigenvalues().maxCoeff();
                    
                    // Quality based on covariance (smaller is better)
                    double cov_quality = std::max(0.0, 1.0 - max_eigenvalue);
                    
                    // Combine with feature quality field (if available)
                    double feat_quality_field = (slam_feat->_quality >= 0) ? slam_feat->_quality : 0.0;
                    
                    // Weight SLAM features more heavily
                    feature_quality = 0.7 * cov_quality + 0.3 * feat_quality_field;
                    // feature->quality = feature_quality; // Store in feature

                    if (en_qual_debug) printf("[QUALITY_DEBUG]   SLAM Feature %zu - Max eigenvalue: %.6f, Cov quality: %.3f, Final quality: %.3f\n", 
                                        feature->featid, max_eigenvalue, cov_quality, feature_quality);
                    
                } else {
                    // This is an MSCKF feature
                    cam_msckf_count++;
                    
                    // Count number of measurements for this feature
                    int num_measurements = 0;
                    for (const auto &cam_meas : feature->timestamps) {
                        num_measurements += static_cast<int>(cam_meas.second.size());
                    }
                    
                    // Weight down if measurements/clones < 1
                    double measurement_weight = 1.0;
                    if (num_measurements < CLONES) {
                        measurement_weight = static_cast<double>(num_measurements) / CLONES;
                    }
                    
                    // Use feature quality field (if available)
                    double feat_quality_field = (feature->quality >= 0) ? feature->quality : 0.0;
                    
                    // MSCKF features weighted less than SLAM features
                    feature_quality = 0.3 * feat_quality_field * measurement_weight;
                    
                    if (en_qual_debug) printf("[QUALITY_DEBUG] Feature %zu (MSCKF) - Grid[%d][%d] - Quality field: %.3f, Measurements: %d, Weight: %.3f, Final quality: %.3f\n", 
                                                feature->featid, grid_x, grid_y, feat_quality_field, num_measurements, measurement_weight, feature_quality);
                }
                
                // Add feature to grid
                grid[grid_y][grid_x]++;
                grid_quality[grid_y][grid_x] += feature_quality;
            }
        }
        
        total_active_slam += cam_slam_count;
        total_active_msckf += cam_msckf_count;
        
        if (en_qual_debug) printf("[QUALITY_DEBUG] Camera %d summary - SLAM: %d, MSCKF: %d\n", cam_id, cam_slam_count, cam_msckf_count);
        
        // Calculate grid distribution score for this camera
        int grids_with_features = 0;
        double camera_grid_score = 0.0;
        
        if (en_qual_debug) printf("[QUALITY_DEBUG] Camera %d grid analysis:\n", cam_id);
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) {
                if (grid[i][j] > 0) {
                    grids_with_features++;
                    
                    // Calculate quality for this grid cell
                    double grid_cell_quality = 0.0;
                    if (grid[i][j] >= TARGET_FEATURES_PER_GRID) {
                        // Grid has enough features, use average quality
                        grid_cell_quality = grid_quality[i][j] / grid[i][j];
                    } else {
                        // Grid has insufficient features, penalize
                        double fill_ratio = static_cast<double>(grid[i][j]) / TARGET_FEATURES_PER_GRID;
                        grid_cell_quality = (grid_quality[i][j] / grid[i][j]) * fill_ratio;
                    }
                    camera_grid_score += grid_cell_quality;
                    
                    if (en_qual_debug) printf("[QUALITY_DEBUG]   Grid[%d][%d]: %d features, avg quality: %.3f\n", 
                                                i, j, grid[i][j], grid_quality[i][j] / grid[i][j]);
                }
            }
        }
        
        // Calculate grid distribution score
        double grid_fill_ratio = static_cast<double>(grids_with_features) / (GRID_SIZE * GRID_SIZE);
        double grid_distribution_score = 0.0;
        
        if (grid_fill_ratio >= GRID_FILL_THRESHOLD) {
            // 50% or more grids filled, give maximum score
            grid_distribution_score = MAX_GRID_SCORE;
        } else {
            // Linear interpolation for partial fill
            grid_distribution_score = (grid_fill_ratio / GRID_FILL_THRESHOLD) * MAX_GRID_SCORE;
        }
        
        // Combine grid distribution with feature quality
        double camera_score = 0.6 * grid_distribution_score + 0.4 * camera_grid_score;
        
        if (en_qual_debug) printf("[QUALITY_DEBUG] Camera %d scores - Grid fill ratio: %.3f, Grid distribution: %.3f, Grid quality: %.3f, Final camera score: %.3f\n", 
               cam_id, grid_fill_ratio, grid_distribution_score, camera_grid_score, camera_score);
        
        total_quality += camera_score;
        total_grids_filled += grids_with_features;
        total_grids += GRID_SIZE * GRID_SIZE;
    }
    
    if (en_qual_debug) printf("[QUALITY_DEBUG] Overall feature counts - Active SLAM: %d, Active MSCKF: %d\n", total_active_slam, total_active_msckf);
    
    // Calculate final quality score
    double final_quality = 0.0;
    if (!features_by_camera.empty()) {
        // Average across all cameras
        final_quality = total_quality / features_by_camera.size();
    }
    
    // Ensure quality is within bounds
    final_quality = std::max(0.0, std::min(100.0, total_quality));
    
    if (en_qual_debug) printf("[QUALITY_DEBUG] Final quality calculation - Total quality: %.3f, Num cameras: %zu, Final quality: %.3f\n", 
           total_quality, features_by_camera.size(), final_quality);
    
    return final_quality;
}

void Publisher::publishBlank()
{
    // Zero out the VIO packet
    memset(&vio_packet, 0, sizeof(vio_data_t));
    vio_packet.magic_number = VIO_MAGIC_NUMBER;
    vio_packet.timestamp_ns = _apps_time_monotonic_ns();

    // Set error codes for missing sensors
    uint32_t packet_error = 0;
    if (!is_imu_connected.load()) packet_error |= ERROR_CODE_IMU_MISSING;
    if (!is_cam_connected.load()) packet_error |= ERROR_CODE_CAM_MISSING;
    vio_packet.error_code = packet_error;

    // Indicate failed state
    vio_packet.quality = -1;
    vio_packet.state = VIO_STATE_FAILED;

    // Publish blank packet on simple channel
    pipe_server_write(SIMPLE_CH, (char *)&vio_packet, sizeof(vio_data_t));
}
