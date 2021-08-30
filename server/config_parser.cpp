/*******************************************************************************
 * Copyright 2021 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

// Project Includes
#include "config_parser.h"

// Modal AI Includes
#include "voxl_common_config.h"
#include "rc_math.h"

// Includes from open_vins
#include <utils/quat_ops.h>

// C/C++ Includes
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cctype>


// #define VIO_CONFIG_FILE_HEADER_STRING "\
// /**\n\
//  * This file contains configuration that's specific to voxl-qvio-server.\n\
//  *\n\
//  * voxl-qvio-server also uses parameters from the following shared config files:\n\
//  * /etc/modalai/extrinsics.conf\n\
//  * /etc/modalai/cameras.cal\n\
//  *\n\
//  * parameter descriptions:\n\
//  * \n\
//  * imu_name: VOXL uses imu1 by defualt since it's the most reliable. On\n\
//  * VOXL-FLIGHT you can optionally try imu0 which is a newer icm42688\n\
//  *\n\
//  * cam_name: camera to use, defaults to tracking0\n\
//  *\n\
//  * odr_hz: Output data date is independent from the camera frame rate so you can\n\
//  * choose the desired output data rate. Note that voxl-imu-server defaults to\n\
//  * 500hz imu sampling but new data is received by voxl-qvio-server at 100hz by\n\
//  * default so requesting qvio data faster requires updating voxl-imu-server.\n\
//  *\n\
//  * use_camera_height_bootstrap: When enabled, the feature estimator will assume\n\
//  * the system starts up with the camera pointed partially at the ground and use\n\
//  * this to make an intial guess of the feature's depth. This should be enabled\n\
//  * for drones that have known-height landing gear.\n\
//  *\n\
//  * camera_height_off_ground_m: camera distance above ground (meters) for the\n\
//  * above bootstrap feature.\n\
//  *\n\
//  * enable_init_while_moving: optionally allow the algorithm to initialize or\n\
//  * reinitialize while moving. Use this if you want to be able to reinitialize\n\
//  * during flight. Based on camera motion, not IMU motion. Recommended to leave\n\
//  * this off unless absolutely desired.\n\
//  *\n\
//  * cam_imu_timeshift_s: Misalignment between camera and imu timestamp in\n\
//  * seconds.\n\
//  *\n\
//  * cam_imu_timeshift_s_uncertainty: uncertainty in camera imu timestamp\n\
//  * misalignment\n\
//  *\n\
//  * T_cam_wrt_imu_uncertainty[3] & R_cam_to_imu_uncertainty[3]: uncertainty in\n\
//  * camera-imu translation\n\
//  *\n\
//  * accl_fsr_ms2 & gyro_fsr_rad: Full scale range used to detect clipping. By\n\
//  * default this is set to a little under the real 16G and 2000DPS FSR so\n\
//  * clipping is detected reliably\n\
//  *\n\
//  * accl_noise_std_dev & gyro_noise_std_dev: standard deviation of accl and gyro\n\
//  * noise\n\
//  *\n\
//  * cam_noise_std_dev: Standard dev of camera noise per pixel.\n\
//  *\n\
//  * min_std_pixel_noise: Minimum of standard deviation of feature measurement\n\
//  * noise in pixels.\n\
//  *\n\
//  * fail_high_pixel_noise_points: Scales measurement noise and compares against\n\
//  * search area (is search area large enough to reliably compute measurement\n\
//  * noise covariance matrix).\n\
//  *\n\
//  * limited_imu_bw_trigger: To prevent tracking failure during/right after (hard)\n\
//  * landing: If sum of 3 consecutive accelerometer samples in any dimension\n\
//  * divided by 4.3 exceed this threshold, IMU measurement noise is increased (and\n\
//  * resets become more likely); if platform vibrates heavily during flight, this\n\
//  * may trigger mid- flight; if poseQuality in mvVISLAMPose drops to\n\
//  * MV_TRACKING_STATE_LOW_QUALITY during flight, improve mechanical dampening\n\
//  * (and/or increase threshold)\n\
//  *\n\
//  * gps_imu_time_alignment_s: Misalignment between GPS and IMU time in seconds\n\
//  *\n\
//  * T_gps_wrt_imu: location of gps with respect to the IMU in meters\n\
//  *\n\
//  * enable_mapping: rudimentary lightweight mapping of feature points, leave this\n\
//  * on.\n\
//  *\n\
//  * enable_gps_vel: allow gps velocity to be added to VIO, requires ModalAI\n\
//  * custom PX4 firmware to expose this data. This is an Alpha feature, don't\n\
//  * enable this unless instructed to.\n\
//  */\n"

#define VIO_CONFIG_FILE_HEADER_STRING "/**hi*/"

ConfigParser::ConfigParser(const std::string& config_filepath_in):
    config_filepath{config_filepath_in}
{

}


bool ConfigParser::parse_file()
{
    cJSON* json_root = NULL;
    try
    {
        // Create an empty JSON file if one is not preset
        json_make_empty_file_with_header_if_missing(this->config_filepath.c_str(), VIO_CONFIG_FILE_HEADER_STRING);

        // Read the JSON file (if it exists)
        json_root = json_read_file(this->config_filepath.c_str());
        if (json_root == NULL)
        {
            std::cerr << "Could not open file: " << this->config_filepath << std::endl;
            return false;
        }

        // Parse all the configs for the VIO system
        this->configs.imu_name = ConfigParser::parse_string_with_default(json_root, "imu_name", "imu0");
        this->configs.online_camera_to_imu_pose_calibration = ConfigParser::parse_bool_with_default(json_root, "online_camera_to_imu_pose_calibration", true);
        this->configs.online_camera_intrinsics_calibration = ConfigParser::parse_bool_with_default(json_root, "online_camera_intrinsics_calibration", true);
        this->configs.online_camera_imu_timestamp_calibration = ConfigParser::parse_bool_with_default(json_root, "online_camera_imu_timestamp_calibration", true);


        if (json_fetch_double_with_default(json_root, "dt_slam_delay_after_initing", &(this->configs.dt_slam_delay_after_initing), 0.0))
        {
            std::cerr << "Could not get double with default for node: dt_slam_delay_after_initing" << std::endl;
            throw std::runtime_error("");
        }

        // Parse the camera configs
        this->configs.camera_configs = this->parse_camera_configs(json_root, this->configs.imu_name);

        // write modified data to disk if necessary
        if (json_get_modified_flag())
        {
            json_write_to_file_with_header(this->config_filepath.c_str(), json_root, VIO_CONFIG_FILE_HEADER_STRING);
        }

    }
    catch (...)
    {
        // NO MEMORY LEAKS HERE!!!
        if (json_root != NULL)
        {
            cJSON_Delete(json_root);
        }

        return false;
    }


    return true;
}


void ConfigParser::print_configs()
{
    std::cout << "=================================================================" << std::endl;
    std::cout << "                       VIO Configs                               " << std::endl;
    std::cout << "=================================================================" << std::endl;

    // Make all the bools be "true" or "false" instead of "1" or "0"
    std::cout << std::boolalpha;


    std::cout << "imu_name:                               \"" << this->configs.imu_name << "\"" <<   std::endl;
    std::cout << "online_camera_to_imu_pose_calibration:   " << this->configs.online_camera_to_imu_pose_calibration << std::endl;
    std::cout << "online_camera_intrinsics_calibration:    " << this->configs.online_camera_intrinsics_calibration << std::endl;
    std::cout << "online_camera_imu_timestamp_calibration: " << this->configs.online_camera_imu_timestamp_calibration << std::endl;
    std::cout << "dt_slam_delay_after_initing:             " << std::fixed << this->configs.dt_slam_delay_after_initing  << " seconds" << std::endl;


    // Print the camera configs
    std::cout << std::endl;
    std::cout << "Camera Configs: " << std::endl;
    for (auto it = this->configs.camera_configs.begin(); it != this->configs.camera_configs.end(); it++)
    {
        std::cout << "\t-camera_name: \"" << it->camera_name << "\"" <<  std::endl;
        std::cout << "\t\t-camera_id:                   " << it->camera_id << std::endl;
        std::cout << "\t\t-camera_type:                 " << ConfigParser::camera_type_enum_string(it->camera_type) << std::endl;
        std::cout << "\t\t-image_width:                 " << it->image_width << std::endl;
        std::cout << "\t\t-image_height:                " << it->image_height << std::endl;
        std::cout << "\t\t-intrinsic_focal_point:       (" << it->intrinsic_focal_point[0] << "," << it->intrinsic_focal_point[1]  << ")"  << std::endl;
        std::cout << "\t\t-intrinsic_principal_point:   (" << it->intrinsic_principal_point[0] << "," << it->intrinsic_principal_point[1]  << ")"  << std::endl;
        std::cout << "\t\t-intrinsic_distortion_coeefs: [" << it->intrinsic_distortion_coeefs[0] << "," << it->intrinsic_distortion_coeefs[1] << "," << it->intrinsic_distortion_coeefs[2] << "," << it->intrinsic_distortion_coeefs[3]  << "]"  << std::endl;
        std::cout << "\t\t-extrinsic_quaternion:        [" << it->extrinsics_cam_to_imu.block(0,0,4,1).transpose()  << std::endl;
        std::cout << "\t\t-extrinsic_translation:       [" << it->extrinsics_cam_to_imu.block(4,0,3,1).transpose()  << std::endl;
    }


    std::cout << "=================================================================" << std::endl;
}

ConfigParser::VIOConfigs ConfigParser::get_configs()
{
    return this->configs;
}

ov_msckf::VioManagerOptions ConfigParser::generate_open_vins_manager_options()
{
    // Create the VIO Manager Options (aka the settings for the manager)
    ov_msckf::VioManagerOptions vio_manager_options;


    // Setting this doesnt matter since we will be feeding in mono images and the trackers
    // will automatically use mono images if mono is passed in but we should set this
    // to false for consistency
    vio_manager_options.use_stereo = false;

    // We did not compile in aruco so disable it
    vio_manager_options.use_aruco = false;

    // No need for multi-threading for this
    vio_manager_options.use_multi_threading = false;

    // Downsampling the cameras are not needed, minimal cpu savings so dont do this 
    vio_manager_options.downsample_cameras = false;

    // Load the configs that were read from the config file
    vio_manager_options.dt_slam_delay = this->configs.dt_slam_delay_after_initing;
    vio_manager_options.state_options.do_calib_camera_pose = this->configs.online_camera_to_imu_pose_calibration;
    vio_manager_options.state_options.do_calib_camera_intrinsics = this->configs.online_camera_intrinsics_calibration;
    vio_manager_options.state_options.do_calib_camera_timeoffset = this->configs.online_camera_imu_timestamp_calibration;

    // Init with zero velocity
    vio_manager_options.try_zupt = true;
    vio_manager_options.zupt_max_velocity = 0.1;
    vio_manager_options.zupt_only_at_beginning = true;
    vio_manager_options.zupt_noise_multiplier = 50;
    vio_manager_options.zupt_max_disparity = 0.5;


    // vio_manager_options.init_imu_thresh = 0.3;

    vio_manager_options.num_pts = 80;
    vio_manager_options.state_options.max_clone_size = 5;

    // Load the camera configs
    for(size_t i = 0; i < this->configs.camera_configs.size();i++)
    {
        // Extract the specific camera configs for this camera for convenience
        CameraConfigs &camera_config = this->configs.camera_configs[i];

        // Set the camera type
        if(camera_config.camera_type == CameraType::TRACKING)
        {
            vio_manager_options.camera_fisheye[camera_config.camera_id] = true;            
        }
        else
        {
            std::cerr << "Camera type not supported in \"generate_open_vins_manager_options(...)\"" << std::endl;
            exit(0);
        }

        // Set the dims, if stereo then this would be the size of 1 image not both
        vio_manager_options.camera_wh[camera_config.camera_id] = std::make_pair(camera_config.image_width, camera_config.image_height);

        // The camera intrinsics
        Eigen::Matrix<double, 8, 1> cam_calib_intrinsic;
        cam_calib_intrinsic(0, 0) = camera_config.intrinsic_focal_point[0]; // k0 fx
        cam_calib_intrinsic(1, 0) = camera_config.intrinsic_focal_point[1]; // k1 fy
        cam_calib_intrinsic(2, 0) = camera_config.intrinsic_principal_point[0]; // k2 x0
        cam_calib_intrinsic(3, 0) = camera_config.intrinsic_principal_point[1]; // k3 y0
        cam_calib_intrinsic(4, 0) = camera_config.intrinsic_distortion_coeefs[0]; // d0
        cam_calib_intrinsic(5, 0) = camera_config.intrinsic_distortion_coeefs[1]; // d1
        cam_calib_intrinsic(6, 0) = camera_config.intrinsic_distortion_coeefs[2]; // d2
        cam_calib_intrinsic(7, 0) = camera_config.intrinsic_distortion_coeefs[3]; // d3
        vio_manager_options.camera_intrinsics[camera_config.camera_id] = cam_calib_intrinsic;

        // The camera extrinsics
        vio_manager_options.camera_extrinsics[camera_config.camera_id] = camera_config.extrinsics_cam_to_imu;
    }


    return vio_manager_options;
}
















std::string ConfigParser::parse_string(cJSON* json_root, const std::string& node_name)
{
    // The max length that the string can be because apparently we are using C!!!!
    // Sigh I live for having a C++ interface for the core libs instead of C.
    const int MAX_STRING_BUFFER_LENGTH = 256;

    // The buffer we will use for reading since we are using C....
    char buffer[MAX_STRING_BUFFER_LENGTH];
    std::memset(buffer, 0, MAX_STRING_BUFFER_LENGTH);

    // Parse the node from the JSON parten
    if (json_fetch_string( json_root, node_name.c_str(), buffer, MAX_STRING_BUFFER_LENGTH) != 0)
    {
        std::cerr << "Could not parse node " << node_name << std::endl;
        throw std::runtime_error("");
    }

    // Convert to C++ for easy use!
    return std::string(buffer);
}


std::string ConfigParser::parse_string_with_default(cJSON* json_root, const std::string& node_name, const std::string& const_value)
{
    // The max length that the string can be because apparently we are using C!!!!
    // Sigh I live for having a C++ interface for the core libs instead of C.
    const int MAX_STRING_BUFFER_LENGTH = 256;

    // The buffer we will use for reading since we are using C....
    char buffer[MAX_STRING_BUFFER_LENGTH];
    std::memset(buffer, 0, MAX_STRING_BUFFER_LENGTH);

    // Parse the node from the JSON parten
    json_fetch_string_with_default( json_root, node_name.c_str(), buffer, MAX_STRING_BUFFER_LENGTH, const_value.c_str());

    // Convert to C++ for easy use!
    return std::string(buffer);
}

bool ConfigParser::parse_bool_with_default(cJSON* json_root, const std::string& node_name, bool default_value)
{
    // We need to do this because the lib uses freakin INTS to represent bools?!?!?! WHYYYYYY
    int bool_value;
    if (json_fetch_bool_with_default(json_root, node_name.c_str(), &bool_value, default_value) != 0)
    {
        std::cerr << "Could not get bool with default for node: " << node_name << std::endl;
        throw std::runtime_error("");
    }

    return (bool_value > 0);
}

std::vector<ConfigParser::CameraConfigs> ConfigParser::parse_camera_configs(cJSON* json_root, const std::string& imu_name)
{
    std::vector<ConfigParser::CameraConfigs> parsed_configs;

    // Extract the array of cameras
    int number_of_cameras = 0;
    cJSON* cameras_root = json_fetch_array_and_add_if_missing(json_root, "cameras", &number_of_cameras);

    // If there are no cameras then add a camera as a default
    if (number_of_cameras == 0)
    {
        // Create the new camera
        cJSON* new_camera_object = cJSON_CreateObject();
        cJSON_AddStringToObject(new_camera_object, "camera_name", "tracking");
        cJSON_AddNumberToObject(new_camera_object, "camera_id", 0);
        cJSON_AddStringToObject(new_camera_object, "camera_type", "tracking");

        // Use the default values for the Modal AI Tracking Camera
        cJSON_AddNumberToObject(new_camera_object, "image_width", 640);
        cJSON_AddNumberToObject(new_camera_object, "image_height", 480);

        double intrinsic_f_default_values[2];
        intrinsic_f_default_values[0] = 275.078;
        intrinsic_f_default_values[1] = 274.931;
        cJSON* intrinsic_f_array = cJSON_CreateDoubleArray(intrinsic_f_default_values, 2);
        cJSON_AddItemToObject(new_camera_object, "intrinsic_focal_point", intrinsic_f_array);

        double intrinsic_principal_point_default_values[2];
        intrinsic_principal_point_default_values[0] = 319.625;
        intrinsic_principal_point_default_values[1] = 243.144;
        cJSON* principal_point_array = cJSON_CreateDoubleArray(intrinsic_principal_point_default_values, 2);
        cJSON_AddItemToObject(new_camera_object, "intrinsic_principal_point", principal_point_array);

        double distortion_coeefs_default_values[4];
        distortion_coeefs_default_values[0]       = 0.003908;
        distortion_coeefs_default_values[1]       = -0.009574;
        distortion_coeefs_default_values[2]       = 0.010173;
        distortion_coeefs_default_values[3]       = -0.003329;
        cJSON* distortion_coeefs_array = cJSON_CreateDoubleArray(distortion_coeefs_default_values, 4);
        cJSON_AddItemToObject(new_camera_object, "intrinsic_distortion_coeefs", distortion_coeefs_array);

        // Add it to the JSON list
        cJSON_AddItemToArray(cameras_root, new_camera_object);

        // Re-fetch the JSON array since we added a new node
        cameras_root = json_fetch_array_of_objects(json_root, "cameras", &number_of_cameras);
    }


    // Extract the camera-imu extrinsic
    vcc_extrinsic_t all_extrinsics[VCC_MAX_EXTRINSICS_IN_CONFIG];
    int number_of_extrinsics = 0;
    if (vcc_read_extrinsic_conf_file(VCC_EXTRINSICS_PATH, all_extrinsics, &number_of_extrinsics, VCC_MAX_EXTRINSICS_IN_CONFIG) != 0)
    {
        std::cerr << "Could not open file: " << VCC_EXTRINSICS_PATH << std::endl;
        throw std::runtime_error("");
    }


    // Loop through the cameras and parse out the configs
    for (int i = 0; i < number_of_cameras; i++)
    {
        // Extract the specific root
        cJSON* camera_node_root = cJSON_GetArrayItem(cameras_root, i);

        // Parse the configs
        ConfigParser::CameraConfigs cam_config;
        cam_config.camera_name = ConfigParser::parse_string(camera_node_root, "camera_name");
        cam_config.camera_type = ConfigParser::string_to_camera_type_enum(ConfigParser::parse_string(camera_node_root, "camera_type"));

        if (json_fetch_int(camera_node_root, "camera_id", &(cam_config.camera_id)) != 0)
        {
            std::cerr << "Could not parse node " << "camera_id" << std::endl;
            throw std::runtime_error("");
        }

        if (json_fetch_int(camera_node_root, "image_width", &(cam_config.image_width)) != 0)
        {
            std::cerr << "Could not parse node " << "image_width" << std::endl;
            throw std::runtime_error("");
        }

        if (json_fetch_int(camera_node_root, "image_height", &(cam_config.image_height)) != 0)
        {
            std::cerr << "Could not parse node " << "image_height" << std::endl;
            throw std::runtime_error("");
        }

        if (json_fetch_fixed_vector(camera_node_root, "intrinsic_focal_point", cam_config.intrinsic_focal_point, 2) != 0)
        {
            std::cerr << "Could not parse node " << "intrinsic_focal_point" << std::endl;
            throw std::runtime_error("");
        }

        if (json_fetch_fixed_vector(camera_node_root, "intrinsic_principal_point", cam_config.intrinsic_principal_point, 2) != 0)
        {
            std::cerr << "Could not parse node " << "intrinsic_principal_point" << std::endl;
            throw std::runtime_error("");
        }


        if (json_fetch_fixed_vector(camera_node_root, "intrinsic_distortion_coeefs", cam_config.intrinsic_distortion_coeefs, 4) != 0)
        {
            std::cerr << "Could not parse node " << "intrinsic_distortion_coeefs" << std::endl;
            throw std::runtime_error("");
        }

        // The actual extracted extrinsics for the system
        vcc_extrinsic_t extrinsic;

        // If we need to invert the extrinsics.  We need the Camera to IMU extrinsics BUT we may extract the
        // IMU to camera extrinsics, in which case we need to get the inverse transformation
        bool needs_inverse_transformation = false;

        if (vcc_find_extrinsic_in_array(cam_config.camera_name.c_str(), imu_name.c_str(), all_extrinsics, number_of_extrinsics, &extrinsic) == 0)
        {
            needs_inverse_transformation = false;
        }
        else if (vcc_find_extrinsic_in_array(imu_name.c_str(), cam_config.camera_name.c_str(), all_extrinsics, number_of_extrinsics, &extrinsic) == 0)
        {
            needs_inverse_transformation = true;
        }
        else
        {
            std::cerr << "Could not find extrinsics between : " << cam_config.camera_name << " and " <<  imu_name << std::endl;
            throw std::runtime_error("");
        }

        // extrinsic.RPY_parent_to_child[0] = -extrinsic.RPY_parent_to_child[0];
        // extrinsic.RPY_parent_to_child[1] = -extrinsic.RPY_parent_to_child[1];
        // extrinsic.RPY_parent_to_child[2] = -extrinsic.RPY_parent_to_child[2];

        // Convert from degrees to radians
        extrinsic.RPY_parent_to_child[0] *= M_PI / 180.0;
        extrinsic.RPY_parent_to_child[1] *= M_PI / 180.0;
        extrinsic.RPY_parent_to_child[2] *= M_PI / 180.0;

        double q[4];
        rc_quaternion_from_tb_array(extrinsic.RPY_parent_to_child, q);


        // // Move everything to Eigen
        // Eigen::Matrix<double, 3, 3> rotation_matrix;
        // rotation_matrix(0, 0) = extrinsic.R_child_to_parent[0][0];
        // rotation_matrix(1, 0) = extrinsic.R_child_to_parent[1][0];
        // rotation_matrix(2, 0) = extrinsic.R_child_to_parent[2][0];
        // rotation_matrix(0, 1) = extrinsic.R_child_to_parent[0][1];
        // rotation_matrix(1, 1) = extrinsic.R_child_to_parent[1][1];
        // rotation_matrix(2, 1) = extrinsic.R_child_to_parent[2][1];
        // rotation_matrix(0, 2) = extrinsic.R_child_to_parent[0][2];
        // rotation_matrix(1, 2) = extrinsic.R_child_to_parent[1][2];
        // rotation_matrix(2, 2) = extrinsic.R_child_to_parent[2][2];


        // std::cout << rotation_matrix << std::endl;
        // std::cout << "-----" << std::endl;

        Eigen::Matrix<double, 3, 1> translation;
        translation[0] = extrinsic.T_child_wrt_parent[0];
        translation[1] = extrinsic.T_child_wrt_parent[1];
        translation[2] = extrinsic.T_child_wrt_parent[2];

        // Convert to a quaternion
        Eigen::Matrix<double, 4, 1> quaternion;//  = ov_core::rot_2_quat(rotation_matrix);
        quaternion(0,0) = q[1];
        quaternion(1,0) = q[2];
        quaternion(2,0) = q[3];
        quaternion(3,0) = q[0];

        // std::cout << quaternion << std::endl;
        // std::cout << "---------" << std::endl;



        // If we need to invert the transformation (aka we have A->B but we want B->A)
        if (needs_inverse_transformation)
        {
            quaternion = ov_core::Inv(quaternion);

            Eigen::Matrix<double, 3, 3> rot = ov_core::quat_2_Rot(quaternion);
            translation = rot * translation;
            translation = -translation;
        }

        // Pack it into the camera vector for the open_vins
        cam_config.extrinsics_cam_to_imu.block(0, 0, 4, 1) = quaternion;
        cam_config.extrinsics_cam_to_imu.block(4, 0, 3, 1) = translation;


        // Add it to the list of cameras
        parsed_configs.push_back(cam_config);
    }


    return parsed_configs;
}


ConfigParser::CameraType ConfigParser::string_to_camera_type_enum(const std::string& enum_string)
{
    // Convert the string to lower case to make it easier for the user
    std::string lower_cased_string = enum_string;
    std::transform(lower_cased_string.begin(), lower_cased_string.end(), lower_cased_string.begin(), [](unsigned char c) { return std::tolower(c); });

    if (lower_cased_string == "tracking")
    {
        return ConfigParser::CameraType::TRACKING;
    }
    else if (lower_cased_string == "stereo")
    {
        return ConfigParser::CameraType::STEREO;
    }

    // Hopefully we never get here
    return ConfigParser::CameraType::UNKNOWN;
}

std::string ConfigParser::camera_type_enum_string(ConfigParser::CameraType enum_val)
{
    switch (enum_val)
    {
        case ConfigParser::CameraType::TRACKING:
            return "TRACKING";
        case ConfigParser::CameraType::STEREO:
            return "STEREO";
        case ConfigParser::CameraType::UNKNOWN:
        default:
            return "UNKNOWN";
    }
}
