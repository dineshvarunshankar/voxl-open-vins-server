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
#ifndef CONFIG_PARSER_H
#define CONFIG_PARSER_H

// 3rd Party Library Includes
#include <Eigen/Eigen>

// Includes from open_vins
#include <core/VioManagerOptions.h>

// Modal AI Libraries
#include <modal_json.h>
#include <cJSON.h>

// C/C++ Includes
#include <vector>

class ConfigParser
{

public:


    /** The types of camera available on ModalAI platforms
     */
    enum CameraType
    {
        TRACKING = 0,
        STEREO, 
        UNKNOWN
    };

    /** Struct containing all the information we need for the cameras.
     *  This allows us to support multiple cameras if we need.
     */
    struct CameraConfigs
    {
        /** The ID for the camera, this is self assigned by the parsers.
         */
        int camera_id{0};

        /** The name of this camera.
         */
        std::string camera_name{""};

        /** The type of camera this is
         */
        CameraType camera_type {CameraType::UNKNOWN};

        /** The camera image dimensions
         */
        int image_width{0};
        int image_height{0};

        /** The intrinsics for the camera
         *  Note: OpenVins only supports 4 distortion coefficients (for Fisheye and RadTan models)
         */
        double intrinsic_focal_point[2];
        double intrinsic_principal_point[2];
        double intrinsic_distortion_coeffs[4]; 

        /** The extrinsic for this this camera and the IMU (CAM->IMU)
         */
        Eigen::Matrix<double, 7, 1> extrinsics_cam_to_imu;
    };

    /** Struct containing all the configs needed for launching the OpenVins system
     */
    struct VIOConfigs
    {
        /** The name of the IMU to use for this VIO.
        *  Unlike the cameras, we can only use 1 IMU for this VIO
        */
        std::string imu_name{""};

        /** Various calibration flags for calibrating things online while running the VIO
         *  Note: Its best to enable all these unless you have a specific use case for not enabling.
         */
        bool online_camera_to_imu_pose_calibration{true};
        bool online_camera_intrinsics_calibration{true};
        bool online_camera_imu_timestamp_calibration{true};

        /** The time delay between initting the system and starting tracking
         *  This is so you can put the drone down if you were doing an "initting sequence".
         */
        double dt_slam_delay_after_initing{0};

        /** If the cameras should be downsized to half resolution
         */
        bool downsample_cameras{false};

        /** The number of features to track
         */
        int number_of_features_to_track{80};

        /** The max clone size to use for the msckf
         */
        int max_clone_size{5};

        /** If we should use zero velocity detection in the estimator
         */
        bool use_zupt{false};

        /** The parameters to use if we are doing zupt
         */
        double zupt_max_velocity{0.1};
        bool zupt_only_at_beginning{false};
        double zupt_noise_multiplier{50.0};
        double zupt_max_disparity{0.5};

        /** If we are not using zupt then we need this
         */
        double init_imu_thresh{0.3};

        /** The configs for all the different cameras that we will be using
         */
        std::vector<CameraConfigs> camera_configs;
    };


    /** The constructor
     * 
     * @config_filepath_in The filepath for the config file to load
     */
    ConfigParser(const std::string& config_filepath_in);

    /** Parse the config file into the config struct
     * 
     * @return true if the file was parsed successfully, false on error
     */
    bool parse_file();

    /** Print the config file to the cout
     */
    void print_configs();

    /** Get the configs that were loaded
     *  
     *   @return The loaded configs if the configs were loaded successfully
     */
    VIOConfigs get_configs();

    /** Generate the VIOManagerOptions for open_vins from the configs that were loaded from the config file
     * 
     * @return The VIOManagerOptions
     */
    ov_msckf::VioManagerOptions generate_open_vins_manager_options();
    
private:

    /** CONSTANTS
     */
    static constexpr int PRINT_NODE_NAME_WIDTH = 10;
    static constexpr int PRINT_NODE_VALUE_WIDTH = 10;

    /** Parse out a string from a json object.
     *  
     * @param json_root The root of the json object to parse from
     * @param node_name The name of the node to get the string value from 
     * @returns The extracted string
     * @throws std::runtime_error If an error occurs while parsing 
     */
    static std::string parse_string(cJSON* json_root, const std::string& node_name);

    /** Parse out a string from a json object.  If the string does not exist then place a default value
     *  
     * @param json_root The root of the json object to parse from
     * @param node_name The name of the node to get the string value from 
     * @param const_value The default value to use if the string wasnt present
     * @returns The extracted string (or the default value if there was no string to extract)
     * @throws std::runtime_error If an error occurs 
     */
    static std::string parse_string_with_default(cJSON* json_root, const std::string& node_name, const std::string& const_value);

    /** Parse out a boolean. If it does not exist then add a default value
     * 
     * @param json_root The root to parse from
     * @param node_name The key of the node name to parse out
     * @param default_value The default value to fill in if the node was not present
     * @return The extracted bool value (or the default value if not present)
     * @throws std::runtime_error If an error occurs while parsing 
     */
    static bool parse_bool_with_default(cJSON* json_root, const std::string& node_name, bool default_value);

    /** Parse out the camera configs.  If no configs are present, then a default conf
     *  
     * @param json_root The root node of the json object that we will be parsing from
     * @param imu_name The name of the IM
     * @param return boolean for if the json was modified
     * @return A vector of the parsed configs
     * @throws std::runtime_error If an error occurs 
     */
    static std::vector<CameraConfigs> parse_camera_configs(cJSON* json_root, const std::string& imu_name, bool& json_was_modified);

    /** Convert a string value to a CameraType enum
     * 
     * @param enum_string The string value to convert
     * @returns A CameraType enum
     */
    static CameraType string_to_camera_type_enum(const std::string& enum_string);

    /** Convert a CameraType enum to a string value
     * 
     * @param enum_val The A CameraType enum to convert
     * @returns A string representation of the camera type enum
     */
    static std::string camera_type_enum_string(CameraType enum_val);

    /** Full filepath to the JSON file we will be parsing
     */
    std::string config_filepath;

    /** The configs that were parsed
     */
    VIOConfigs configs;
};

#endif // CONFIG_PARSER_H
