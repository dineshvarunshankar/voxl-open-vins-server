/**
 * @file VoxlConfigure.cpp
 * @brief Configuration management implementation for VOXL OpenVINS server
 * @author Zauberflote
 * @date 2025
 * @version 1.0
 *
 * This file implements the configuration management system for the VOXL OpenVINS
 * server. It provides functions for reading server configuration files and
 * synchronizing camera configurations with the system.
 *
 * The implementation handles:
 * - Camera configuration synchronization with system services
 * - Server configuration file parsing and validation
 * - YAML file management for OpenVINS parameters
 * - Camera calibration and extrinsic parameter handling
 * - Blind takeoff feature configuration
 */

#include "VoxlConfigure.h"

namespace voxl
{

    static void printExtrinsic(const vcc_extrinsic_t& ext)
    {
        /* guard: empty parent means this slot wasn’t filled in */  
        if (ext.parent[0] == '\0') {
            std::cout << "Extrinsic has no parent frame assigned.\n";
            return;
        }

        std::cout << "==========  Extrinsic calibration  ==========\n";
        std::cout << " Parent frame : " << ext.parent << '\n';
        std::cout << " Child  frame : " << ext.child  << '\n';

        // Translation vector (m)
        std::cout << " Translation T_child←parent [m] : "
                << std::fixed << std::setprecision(3)
                << ext.T_child_wrt_parent[0] << ", "
                << ext.T_child_wrt_parent[1] << ", "
                << ext.T_child_wrt_parent[2] << '\n';

        // Euler angles (deg)
        std::cout << " RPY parent→child  [deg]        : "
                << ext.RPY_parent_to_child[0] << ", "
                << ext.RPY_parent_to_child[1] << ", "
                << ext.RPY_parent_to_child[2] << '\n';

        // 3×3 rotation matrix
        std::cout << " Rotation matrix child←parent   :\n";
        for (int r = 0; r < 3; ++r) {
            std::cout << "   [ ";
            for (int c = 0; c < 3; ++c)
                std::cout << std::setw(10) << std::setprecision(6)
                        << ext.R_child_to_parent[r][c] << (c < 2 ? ' ' : '\0');
            std::cout << " ]\n";
        }
        std::cout << "=============================================\n";
    }

    /**
     * @brief Synchronize camera configuration with system services
     *
     * This function reads camera configuration from system services and
     * synchronizes the lens intrinsics and distortion model parameters
     * with the VIO system. It ensures that the camera calibration data
     * used by the VIO system matches the current system configuration.
     *
     * The function performs the following operations:
     * - Reads camera configuration from VIO camera configuration file
     * - Validates extrinsic and calibration data presence
     * - Ensures all cameras use the same IMU
     * - Updates OpenVINS estimator YAML with camera count
     * - Updates camera chain YAML with intrinsic parameters
     * - Configures blind takeoff feature based on camera occlusion
     * - Populates global camera information vector
     *
     * The function updates several YAML files:
     * - estimator_config.yaml
     * - kalibr_imucam_chain.yaml
     *
     *
     * @return 0 on success, -1 on failure
     * @see read_server_config()
     */
    int sync_cam_config(void)
    {
        // Initialize global variables
        takeoff_cam = -1;
        takeoff_cams.clear();
        cam_info_vec.clear();

        // GRAB CAMERA VIO CONF FILE
        vio_cam_t vio_cams[MAX_CAM_CNT];
        int n_cams = vcc_read_vio_cam_conf_file(vio_cams, MAX_CAM_CNT, 1);
        if (n_cams < 1)
            return -1;

        // Load body to IMU transformation if IMU body mode is enabled
        Eigen::Matrix4d T_imu_body = Eigen::Matrix4d::Identity();
        if (en_imu_body)
        {
            vcc_extrinsic_t body_imu_ext;
            // Fetch the body->imu extrinsic relation
            if (vcc_fetch_extrinsic("body", vio_cams[0].imu, &body_imu_ext) != 0)
            {
                fprintf(stderr, "Failed to fetch body->%s extrinsic. IMU body mode requires this extrinsic to be defined.\n", vio_cams[0].imu);
                return -1;
            }

            if (en_verbose) {
                printf("\n[INFO] IMU body mode enabled - loading body->%s transformation\n", vio_cams[0].imu);
                printExtrinsic(body_imu_ext);
            }

            // Convert extrinsic to transformation matrix T_imu_body (from body to imu)
            Eigen::Vector3d t_imu_wrt_body(
                body_imu_ext.T_child_wrt_parent[0],
                body_imu_ext.T_child_wrt_parent[1],
                body_imu_ext.T_child_wrt_parent[2]
            );
            const Eigen::Matrix<double,3,3,Eigen::RowMajor> R_imu_to_body(&body_imu_ext.R_child_to_parent[0][0]);

            // Compute T_imu_body (transformation from body frame to IMU frame)
            T_imu_body.block<3,3>(0,0) = R_imu_to_body.transpose();              // R_body_to_imu
            T_imu_body.block<3,1>(0,3) = -R_imu_to_body.transpose() * t_imu_wrt_body;  // translation

            if (en_verbose) {
                printf("\n[INFO] T_imu_body transformation matrix (body->imu):\n");
                std::cout << T_imu_body << std::endl;
            }
        }

        // CHECK IF WE HAVE EXTRINSICS AND CAMERA CALIBRATION
        for (int i = 0; i < n_cams; i++)
        {
            if (!vio_cams[i].is_extrinsic_present)
            {
                fprintf(stderr, "failed to find extrinsic config for vio cam %s\n", vio_cams[i].name);
                return -1;
            }
            if (!vio_cams[i].is_cal_present)
            {
                fprintf(stderr, "failed to find cam cal for vio cam %s\n", vio_cams[i].name);
                return -1;
            }
        }

        // FOR NOW ALL CAMERAS SHOULD BE BASED OFF THE SAME IMU
        // EXAMPLE: IF CAMERA 0 IS ATTACHED TO A DIFFERENT IMU THAN CAMERA 1, FLAG ERROR

        // CHECK IF WE HAVE A VALID IMU FOR ALL CAMERAS AND IF THEY ARE THE SAME
        // Security: Use strncpy with bounds checking and ensure null termination
        strncpy(imu_name, vio_cams[0].imu, sizeof(imu_name) - 1);
        imu_name[sizeof(imu_name) - 1] = '\0'; // Ensure null termination

        for (int i = 1; i < n_cams; i++)
        {
            if (strcmp(vio_cams[i].imu, imu_name) != 0)
            {
                fprintf(stderr, "vio cam %s has a different imu than vio cam %s\n", vio_cams[i].name, vio_cams[0].name);
                return -1;
            }
        }

        // If IMU body mode is enabled, append "_body" to the IMU name
        if (en_imu_body)
        {
            // Ensure we have enough space for "_body" suffix
            size_t current_len = strlen(imu_name);
            const char* suffix = "_body";
            size_t suffix_len = strlen(suffix);

            if (current_len + suffix_len >= sizeof(imu_name))
            {
                fprintf(stderr, "Error: IMU name '%s' is too long to append '_body' suffix\n", imu_name);
                return -1;
            }

            strncat(imu_name, suffix, sizeof(imu_name) - current_len - 1);

            if (en_verbose) {
                printf("[INFO] IMU body mode: subscribing to '%s' instead of '%s'\n",
                       imu_name, vio_cams[0].imu);
            }
        }
        // NOW SYNC ESTIMATOR YAML WITH CONF FILE
        // OPEN ESTIMATOR YAML FILE
        std::string yaml_estimator_path = std::string(folder_base) + "/estimator_config.yaml";
        YAML::Node config;
        try
        {
            config = YAML::LoadFile(yaml_estimator_path);
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Failed to load estimator YAML: %s\n", e.what());
            fprintf(stderr, "Creating new estimator config file\n");
            // Create a basic config if file doesn't exist
            config["max_cameras"] = n_cams;
        }
        // SYNC MAX CAMERAS
        if (config["max_cameras"])
        {
            int max_cams = config["max_cameras"].as<int>();
            if (max_cams != n_cams)
            {
                config["max_cameras"] = n_cams;
                if(sync_config){
                    try
                    {
                        std::ofstream fout(yaml_estimator_path);
                        // Write YAML header first
                        fout << "%YAML:1.0" << std::endl
                            << std::endl;
                        fout << config;
                    }
                    catch (const std::exception &e)
                    {
                        std::cerr << "Failed to write YAML: " << e.what() << std::endl;
                        return -1;
                    }
                }
            }
        }
        // NOW SYNC INTRINSICS AND DISTORTION MODEL

        // LOAD CAMCHAIN YAML FILE
        std::string yaml_camchain_path = std::string(folder_base) + "/kalibr_imucam_chain.yaml";
        YAML::Node camchain_config;
        try
        {
            camchain_config = YAML::LoadFile(yaml_camchain_path);
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Failed to load YAML: %s\n", e.what());
            return -1;
        }
        int is_there_an_occlluded_cam = 0;
        for (int i = 0; i < n_cams; i++)
        {

            std::string cam_key = "cam" + std::to_string(i);

            cam_info cam;
            // Security: Use strncpy with bounds checking and ensure null termination
            strncpy(cam.name, vio_cams[i].name, sizeof(cam.name) - 1);
            cam.name[sizeof(cam.name) - 1] = '\0';

            strncpy(cam.tracking_name, vio_cams[i].pipe_for_tracking, sizeof(cam.tracking_name) - 1);
            cam.tracking_name[sizeof(cam.tracking_name) - 1] = '\0';

            strncpy(cam.preview_name, vio_cams[i].pipe_for_preview, sizeof(cam.preview_name) - 1);
            cam.preview_name[sizeof(cam.preview_name) - 1] = '\0';

            cam.mode = using_stereo ? STEREO : MONO;
            cam.cam_id = i;

            // GRAB LIST OF REALIBLE CAMERAS FOR TAKEOFF
            if (takeoff_cam < 0 && !vio_cams[i].is_occluded_on_ground)
            {
                takeoff_cam = i;
            }
            printf("vio cam: %d is occluded: %d\n", i, vio_cams[i].is_occluded_on_ground);
            if (!vio_cams[i].is_occluded_on_ground)
            {
                takeoff_cams.push_back(i);
                cam.is_occluded_on_takeoff = 0;
            }
            if (vio_cams[i].is_occluded_on_ground){
                is_there_an_occlluded_cam = 1;
                cam.is_occluded_on_takeoff = vio_cams[i].is_occluded_on_ground;
            }

            // GRAB INTRINSICS AND DISTORTION MODEL
            // THIS WHOLE SECTION CAN BE REMOVED -- PENDING CHECK
            //  ------------------------------------------------------------
            cam.width = vio_cams[i].cal.width;
            cam.height = vio_cams[i].cal.height;
            cam.cam_calib_intrinsic(0, 0) = vio_cams[i].cal.fx;
            cam.cam_calib_intrinsic(1, 0) = vio_cams[i].cal.fy;
            cam.cam_calib_intrinsic(2, 0) = vio_cams[i].cal.cx;
            cam.cam_calib_intrinsic(3, 0) = vio_cams[i].cal.cy;
            cam.cam_calib_intrinsic(4, 0) = vio_cams[i].cal.D[0];
            cam.cam_calib_intrinsic(5, 0) = vio_cams[i].cal.D[1];
            cam.cam_calib_intrinsic(6, 0) = vio_cams[i].cal.D[2];
            cam.cam_calib_intrinsic(7, 0) = vio_cams[i].cal.D[3];
            // ------------------------------------------------------------
            if (vio_cams[i].cal.is_fisheye)
                cam.is_fisheye = 1;
            else
                cam.is_fisheye = 0;

            // INTRINSICS, DISTORTION MODEL, AND RESOLUTION
            //  Set intrinsics as array [fx, fy, cx, cy]
            YAML::Node intrinsics;
            intrinsics.push_back(vio_cams[i].cal.fx);
            intrinsics.push_back(vio_cams[i].cal.fy);
            intrinsics.push_back(vio_cams[i].cal.cx);
            intrinsics.push_back(vio_cams[i].cal.cy);
            camchain_config[cam_key]["intrinsics"] = intrinsics;

            // Set distortion coefficients as array [D[0], D[1], D[2], D[3]]
            YAML::Node distortion_coeffs;
            distortion_coeffs.push_back(vio_cams[i].cal.D[0]);
            distortion_coeffs.push_back(vio_cams[i].cal.D[1]);
            distortion_coeffs.push_back(vio_cams[i].cal.D[2]);
            distortion_coeffs.push_back(vio_cams[i].cal.D[3]);
            camchain_config[cam_key]["distortion_coeffs"] = distortion_coeffs;

            // Set resolution as array [width, height]
            YAML::Node resolution;
            resolution.push_back(vio_cams[i].cal.width);
            resolution.push_back(vio_cams[i].cal.height);
            camchain_config[cam_key]["resolution"] = resolution;

            camchain_config[cam_key]["distortion_model"] = vio_cams[i].cal.is_fisheye ? "equidistant" : "radtan";
            
            // Set extrinsics as a 4x4 matrix
            YAML::Node extrinsics = YAML::Node(YAML::NodeType::Sequence);
            
            vcc_extrinsic_t ext = vio_cams[i].extrinsic;
            Eigen::Vector3d t_pc_p(
                ext.T_child_wrt_parent[0],
                ext.T_child_wrt_parent[1],
                ext.T_child_wrt_parent[2]
            );
            const Eigen::Matrix<double,3,3,Eigen::RowMajor> R_cp(&ext.R_child_to_parent[0][0]);

            // Compute T_cam_imu (transformation from IMU to camera)
            Eigen::Matrix4d T_cam_imu = Eigen::Matrix4d::Identity();
            T_cam_imu.block<3,3>(0,0) =  R_cp.transpose();             // rotation
            T_cam_imu.block<3,1>(0,3) = -R_cp.transpose() * t_pc_p;    // translation

            // Apply body transformation if IMU body mode is enabled
            Eigen::Matrix4d T_final;
            if (en_imu_body)
            {
                // Convert T_cam_imu to T_cam_body by applying T_imu_body
                // T_cam_body = T_cam_imu * T_imu_body
                T_final = T_cam_imu * T_imu_body;

                if (en_verbose) {
                    printf("\n[INFO] Camera %d (%s) - Applied body transformation\n", i, vio_cams[i].name);
                    printf("  T_cam_body (body->cam):\n");
                    std::cout << T_final << std::endl;
                }
            }
            else
            {
                T_final = T_cam_imu;
            }

            for (int row = 0; row < 4; ++row) {
                YAML::Node row_node = YAML::Node(YAML::NodeType::Sequence);
                for (int col = 0; col < 4; ++col) {
                    row_node.push_back(T_final(row, col));
                }
                extrinsics.push_back(row_node);
            }
            // Note: When en_imu_body=1, T_cam_imu actually contains the body->cam transformation
            // This allows the VIO system to work in the body frame instead of the IMU frame
            camchain_config[cam_key]["T_cam_imu"] = extrinsics;

            // ADD THIS CAMERA TO OUR INFO VECTOR
            cam_info_vec.push_back(cam);
        }
        // NOW SAVE CAMCHAIN YAML FILE
        if(sync_config){
            try
            {
                printf("Writing synced camera chain YAML to %s\n", yaml_camchain_path.c_str());
                if (en_imu_body) {
                    printf("[INFO] IMU body mode enabled\n");
                }
                std::ofstream fout(yaml_camchain_path);
                // Write YAML header first
                fout << "%YAML:1.0" << std::endl
                    << std::endl;
                fout << camchain_config;
            }
            catch (const std::exception &e)
            {
                std::cerr << "Failed to write YAML: " << e.what() << std::endl;
                return -1;
            }
        }

        // BLIND TAKEOFF FEATURE PREPARATION
        if (takeoff_cam < 0)
            takeoff_cam = 0;
        if (takeoff_cams.empty())
            takeoff_cams.push_back(0);

        // if no cams were occluded, we can disable the blind takeoff feature
        if (!is_there_an_occlluded_cam)
        {
            takeoff_cam = -1;
            takeoff_cams.clear();
        }
        return 0;
    }

    /**
     * @brief Read and parse server configuration file
     *
     * This function reads the main server configuration file and parses
     * all the parameters needed for VIO operation. It handles JSON format
     * configuration files and validates the parameters.
     *
     * The function reads configuration for:
     * - Auto-reset parameters and thresholds
     * - Velocity covariance limits and timeouts
     * - Feature count requirements and timeouts
     * - Auto-fallback mode settings
     * - Yaw monitoring and fast yaw detection
     * - Debug output settings
     *
     * The function creates a new configuration file with default values
     * if one doesn't exist, and saves any modifications made during
     * parsing back to disk.
     *
     * Configuration file location: /etc/modalai/voxl-open-vins-server.conf
     *
     * @return 0 on success, -1 on failure
     * @see sync_cam_config()
     */
    int read_server_config(void)
    {
        // THESE ARE HIGHER LEVEL CONFIGS FOR THE OV SERVER NOT OV ITSELF!
        int ret = json_make_empty_file_with_header_if_missing(CONFIG_FILE, CONFIG_FILE_HEADER);
        if (ret < 0)
            return -1;
        else if (ret > 0)
            fprintf(stderr, "Creating new OV server config file: %s\n", CONFIG_FILE);

        cJSON *parent = json_read_file(CONFIG_FILE);
        if (parent == NULL)
            return -1;

        char string_holder[CHAR_BUF_SIZE];
        memset(string_holder, '\0', CHAR_BUF_SIZE);
        json_fetch_bool_with_default(parent, "en_auto_reset", &en_auto_reset, 1);
        json_fetch_float_with_default(parent, "auto_reset_max_velocity", &auto_reset_max_velocity, 20.0f);
        json_fetch_float_with_default(parent, "auto_reset_max_v_cov_instant", &auto_reset_max_v_cov_instant, 0.2f);
        json_fetch_float_with_default(parent, "auto_reset_max_v_cov", &auto_reset_max_v_cov, 0.2f);
        json_fetch_float_with_default(parent, "auto_reset_max_v_cov_timeout_s", &auto_reset_max_v_cov_timeout_s, 0.5f);
        json_fetch_int_with_default(parent, "auto_reset_min_features", &auto_reset_min_features, 1);
        json_fetch_float_with_default(parent, "auto_reset_min_feature_timeout_s", &auto_reset_min_feature_timeout_s, 10.0f);
        json_fetch_float_with_default(parent, "ok_state_grace_timeout_s", &ok_state_grace_timeout_s, 2.0f);
        json_fetch_float_with_default(parent, "auto_fallback_timeout_s", &auto_fallback_timeout_s, 3.0f);
        json_fetch_float_with_default(parent, "auto_fallback_min_v", &auto_fallback_min_v, 0.6f);
        json_fetch_bool_with_default(parent, "en_cont_yaw_checks", (int *)&en_cont_yaw_checks, 0);
        json_fetch_float_with_default(parent, "fast_yaw_thresh", &fast_yaw_thresh, 5.0f);
        json_fetch_float_with_default(parent, "fast_yaw_timeout_s", &fast_yaw_timeout_s, 1.75f);
        json_fetch_string_with_default(parent, "yaml_folder", folder_base, CHAR_BUF_SIZE, "/usr/share/modalai/voxl-open-vins/VoxlConfig/starling2");
        json_fetch_int_with_default(parent, "using_stereo", &using_stereo, 0);
        json_fetch_float_with_default(parent, "takeoff_alt_threshold", &takeoff_alt_threshold, 0.5f);
        json_fetch_bool_with_default(parent, "takeoff_occlude_stereo_left", (int *)&occlude_stereo_left, 0);
        json_fetch_bool_with_default(parent, "takeoff_occlude_stereo_right", (int *)&occlude_stereo_right, 0);
        json_fetch_bool_with_default(parent, "sync_config", (int *)&sync_config, 1);
        json_fetch_float_with_default(parent, "fusion_rate_dt_ms", &fusion_rate_dt_ms, 20.0f);
        json_fetch_bool_with_default(parent, "imu_body_frame_mode", (int *)&en_imu_body, 1);

        // Quality hysteresis threshold configuration
        json_fetch_int_with_default(parent, "quality_low_thresh_initial", &quality_low_thresh_initial, 15);
        json_fetch_int_with_default(parent, "quality_low_thresh_good", &quality_low_thresh_good, 14);
        json_fetch_int_with_default(parent, "quality_high_thresh", &quality_high_thresh, 35);
        json_fetch_int_with_default(parent, "quality_initial_to_bad_count", &quality_initial_to_bad_count, 20);
        json_fetch_int_with_default(parent, "quality_initial_to_good_count", &quality_initial_to_good_count, 50);
        json_fetch_int_with_default(parent, "quality_bad_to_good_count", &quality_bad_to_good_count, 60);
        json_fetch_int_with_default(parent, "quality_good_to_bad_count", &quality_good_to_bad_count, 45);

        if (json_get_parse_error_flag())
        {
            fprintf(stderr, "failed to parse config file %s\n", CONFIG_FILE);
            cJSON_Delete(parent);
            return -1;
        }

        // write modified data to disk if neccessary
        if (json_get_modified_flag())
        {
            printf("The config file was modified during parsing, saving the changes to disk\n");
            json_write_to_file_with_header(CONFIG_FILE, parent, CONFIG_FILE_HEADER);
        }
        cJSON_Delete(parent);
        return 0;
    }

} // namespace voxl
