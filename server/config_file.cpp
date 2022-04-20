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

// Includes from open_vins
#include <core/VioManagerOptions.h>
#include <modal_json.h>
#include <rc_math.h>
#include <stdio.h>
#include <voxl_common_config.h>

#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stereo.hpp>

using namespace cv;
using namespace std;

#include "config_file.h"

// define all the externs from config_file.h
std::vector<camera_info> cam_info_vec(MAX_CAMERAS);
char imu_name[CHAR_BUF_SIZE];

// online calibration? think that just means it adjusts and tries to converge on these vals?
bool camera_to_imu_pose_calibration;
bool camera_intrinsics_calibration;
bool camera_imu_timestamp_calibration;

double delay_after_init;
bool downsample_cams;
int num_features_to_track;
int max_clone_size;

bool use_zupt;
double zupt_max_velocity;
bool zupt_only_at_beginning;
double zupt_noise_multiplier;
double zupt_max_disparity;
double init_imu_thresh;


static void create_ov_extrinsics(vcc_extrinsic_t &extrins, Eigen::Matrix<double, 7, 1> &cam_wrt_imu, bool needs_inverse){
    // read in our rotation
    Eigen::Matrix<double, 3, 3> rotation_ch_wrt_par;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rotation_ch_wrt_par(i, j) = extrins.R_child_to_parent[i][j];
        }
    }

    // reverse our rotation. this gives us parent->child, same as using the RPY vector would have
    Eigen::Matrix<double, 3, 3> rotation_par_wrt_ch = rotation_ch_wrt_par.transpose();

    // create our quaternion
    Eigen::Matrix<double, 4, 1> quaternion;

    // open_vins jpl creator from rotation matrix, using correct rotation based on needs_inverse relation
    quaternion = rot_2_quat(needs_inverse ? rotation_par_wrt_ch : rotation_ch_wrt_par);

    // create our translation vec
    Eigen::Matrix<double, 3, 1> translation;
    translation[0] = extrins.T_child_wrt_parent[0];
    translation[1] = extrins.T_child_wrt_parent[1];
    translation[2] = extrins.T_child_wrt_parent[2];

    if (needs_inverse) {
        // align translation into the new rotation frame
        translation = rotation_par_wrt_ch * translation;
        // finally, invert it and we're ready to go
        translation = -translation;
    }
    cam_wrt_imu.block(0, 0, 4, 1) = quaternion;
    cam_wrt_imu.block(4, 0, 3, 1) = translation;
    return;
}

int load_extrinsics_file() {
    vcc_extrinsic_t t[VCC_MAX_EXTRINSICS_IN_CONFIG];
    vcc_extrinsic_t extrins_holder;
    char ext_name[CHAR_BUF_SIZE];

    // now load in extrinsics
    int n_extrinsics;
    if (vcc_read_extrinsic_conf_file(VCC_EXTRINSICS_PATH, t, &n_extrinsics, VCC_MAX_EXTRINSICS_IN_CONFIG)) {
        fprintf(stderr, "ERROR: Unable to read extrinsics conf at %s\n", VCC_EXTRINSICS_PATH);
        return -1;
    }

    bool needs_inverse_transform = false;
    for (int i = 0; i < MAX_CAMERAS; i++){
        if (cam_info_vec[i].enable){
            // create a copy of the camera name
            memset(ext_name, '\0', CHAR_BUF_SIZE);
            strcpy(ext_name, cam_info_vec[i].name);

            // if it contains stereo, we know the extrinsics file will contain the name + _l since we cal to left sensor
            if (strstr(ext_name, "stereo") != NULL) strcat(ext_name, "_l");

            if (!vcc_find_extrinsic_in_array(ext_name, imu_name, t, n_extrinsics, &extrins_holder)) {
                needs_inverse_transform = false;
            } else if (!vcc_find_extrinsic_in_array(imu_name, ext_name, t, n_extrinsics, &extrins_holder)) {
                // translation is from cam -> imu, need translation from imu -> cam
                fprintf(stderr, "creating inverse transform\n");
                needs_inverse_transform = true;
            } else {
                fprintf(stderr, "ERROR: %s missing %s to %s transform\n", VCC_EXTRINSICS_PATH, imu_name, ext_name);
                return -1;
            }

            create_ov_extrinsics(extrins_holder, cam_info_vec[i].cam_wrt_imu, needs_inverse_transform);
            needs_inverse_transform = false;
        }
    }
    return 0;
}


int load_intrinsics_file() {
    char intrinsics_path[CHAR_BUF_SIZE];

    for (int i = 0; i < MAX_CAMERAS; i++){
        if (cam_info_vec[i].enable){
            memset(intrinsics_path, '\0', CHAR_BUF_SIZE);
            // opencv_cam_name_intrinsics.yml
            strcpy(intrinsics_path, "opencv_");
            strcat(intrinsics_path, cam_info_vec[i].name);
            strcat(intrinsics_path, "_intrinsics.yml");

            FileStorage fs(intrinsics_path, FileStorage::READ);
            if(!fs.isOpened()){
                fprintf(stderr, "Failed to load intrinsicss file %s\n", intrinsics_path);
                return -1;
            }

            FileNode n;
            Mat camMatrix;
            Mat distCoeffs;
            int is_fisheye = 0;
            int w, h;
            int has_m = 0;
            int has_d = 0;
            int has_w = 0;
            int has_h = 0;

            // opencv cal files don't have consistent names for matrices, so try a few
            // name for mono camera
            n = fs["M"];
            if (n.type() != FileNode::NONE) {
                n >> camMatrix;
                has_m = 1;
            }
            n = fs["D"];
            if (n.type() != FileNode::NONE) {
                n >> distCoeffs;
                has_d = 1;
            }
            // name for stereo left
            n = fs["M1"];
            if(n.type() != FileNode::NONE){
                n >> camMatrix;
                has_m = 1;
            }
            n = fs["D1"];
            if(n.type() != FileNode::NONE){
                n >> distCoeffs;
                has_d = 1;
            }

            // name used in aruco example
            n = fs["camera_matrix"];
            if(n.type() != FileNode::NONE){
                n >> camMatrix;
                has_m = 1;
            }
            n = fs["distortion_coefficients"];
            if(n.type() != FileNode::NONE){
                n >> distCoeffs;
                has_d = 1;
            }

            // check for fisheye model
            n = fs["distortion_model"];
            if (n.isString()) {
                std::string mdl_name = n;
                if (mdl_name.compare("fisheye") == 0) {
                    is_fisheye = 1;
                }
            }

            // check height and width
            n = fs["width"];
            if (n.type() != FileNode::NONE) {
                n >> w;
                has_w = 1;
            }
            n = fs["height"];
            if (n.type() != FileNode::NONE) {
                n >> h;
                has_h = 1;
            }

            // done with file now
            fs.release();

            // make sure we loaded the matrices in
            if (!has_m) {
                fprintf(stderr, "failed to find camera matrix in %s\n", intrinsics_path);
            }
            if (!has_d) {
                fprintf(stderr, "failed to find distortion coefficients in %s\n", intrinsics_path);
            }
            if (!has_w) {
                fprintf(stderr, "failed to find width in %s\n", intrinsics_path);
            }
            if (!has_h) {
                fprintf(stderr, "failed to find height in %s\n", intrinsics_path);
            }
            if (!has_m || !has_d || !has_w || !has_h) {
                return -1;
            }

            // populate the open_vins cam_calib_intrinsics with the data we have
            cam_info_vec[i].cam_calib_intrinsic(0, 0) = camMatrix.at<double>(0, 0);
            cam_info_vec[i].cam_calib_intrinsic(1, 0) = camMatrix.at<double>(1, 1);
            cam_info_vec[i].cam_calib_intrinsic(2, 0) = camMatrix.at<double>(0, 2);
            cam_info_vec[i].cam_calib_intrinsic(3, 0) = camMatrix.at<double>(1, 2);
            cam_info_vec[i].cam_calib_intrinsic(4, 0) = distCoeffs.at<double>(0);
            cam_info_vec[i].cam_calib_intrinsic(5, 0) = distCoeffs.at<double>(1);
            cam_info_vec[i].cam_calib_intrinsic(6, 0) = distCoeffs.at<double>(2);
            cam_info_vec[i].cam_calib_intrinsic(7, 0) = distCoeffs.at<double>(3);
            if (has_w) cam_info_vec[i].cam_calib_intrinsic(8, 0) = w;
            if (has_h) cam_info_vec[i].cam_calib_intrinsic(9, 0) = h;
            cam_info_vec[i].is_fisheye = is_fisheye;
        }
    }
    return 0;
}

int config_file_print(void) {
    printf("=================================================================\n");
    printf("==========================CAMERA 0================================\n");
    printf("enable:                           %s\n", cam_info_vec[0].enable ? "true" : "false");
    printf("name:                             %s\n", cam_info_vec[0].name);
    printf("==========================CAMERA 2================================\n"); 
    printf("enable:                           %s\n", cam_info_vec[1].enable ? "true" : "false");
    printf("name:                             %s\n", cam_info_vec[1].name);
    printf("==========================CAMERA 3================================\n");
    printf("enable:                           %s\n", cam_info_vec[2].enable ? "true" : "false");
    printf("name:                             %s\n", cam_info_vec[2].name);
    printf("==========================CAMERA 4================================\n");
    printf("enable:                           %s\n", cam_info_vec[3].enable ? "true" : "false");
    printf("name:                             %s\n", cam_info_vec[3].name);
    printf("=================================================================\n");
    printf("===========================IMU====================================\n");
    printf("imu pipe:                         %s\n", imu_name);
    printf("=================================================================\n");
    printf("==========================GENERAL=================================\n");
    printf("camera_to_imu_pose_calibration:   %s\n", camera_to_imu_pose_calibration ? "true" : "false");
    printf("camera_intrinsics_calibration:    %s\n", camera_intrinsics_calibration ? "true" : "false");
    printf("camera_imu_timestamp_calibration: %s\n", camera_imu_timestamp_calibration ? "true" : "false");
    printf("delay after init(seconds):        %6.5f\n", delay_after_init);
    printf("downsample cams:                  %s\n", downsample_cams ? "true" : "false");
    printf("features to track:                %d\n", num_features_to_track);
    printf("max clone size:                   %d\n", max_clone_size);
    printf("use zupt:                         %s\n", use_zupt ? "true" : "false");
    if (use_zupt) {
        printf("zupt max velocity:                %6.5f\n", zupt_max_velocity);
        printf("use zupt only at beginning:       %s\n", zupt_only_at_beginning ? "true" : "false");
        printf("zupt noise multiplier:            %6.5f\n", zupt_noise_multiplier);
        printf("zupt max disparity:               %6.5f\n", zupt_max_disparity);
    } else
        printf("init imu thresh:                  %6.5f\n", init_imu_thresh);
    printf("=================================================================\n");
    printf("=================================================================\n");
    return 0;
}

/**
 * load the config file and populate the above extern variables
 *
 * @return     0 on success, -1 on failure
 */
int config_file_read(void) {
    int ret = json_make_empty_file_with_header_if_missing(CONFIG_FILE, CONFIG_FILE_HEADER);
    if (ret < 0)
        return -1;
    else if (ret > 0)
        fprintf(stderr, "Creating new config file: %s\n", CONFIG_FILE);

    cJSON* parent = json_read_file(CONFIG_FILE);
    if (parent == NULL) return -1;

    // actually parse values
    json_fetch_bool_with_default(parent, "cam0_enable", (int*)&cam_info_vec[0].enable, 1);
    json_fetch_string_with_default(parent, "cam0_name", cam_info_vec[0].name, CHAR_BUF_SIZE, "tracking");
    
    json_fetch_bool_with_default(parent, "cam1_enable", (int*)&cam_info_vec[1].enable, 0);
    json_fetch_string_with_default(parent, "cam1_pipe", cam_info_vec[1].name, CHAR_BUF_SIZE, "stereo_front");

    json_fetch_bool_with_default(parent, "cam2_enable", (int*)&cam_info_vec[2].enable, 0);
    json_fetch_string_with_default(parent, "cam2_pipe", cam_info_vec[2].name, CHAR_BUF_SIZE, "stereo_rear");

    json_fetch_bool_with_default(parent, "cam3_enable", (int*)&cam_info_vec[3], 0);
    json_fetch_string_with_default(parent, "cam3_pipe", cam_info_vec[3].name, CHAR_BUF_SIZE, "/run/mpa/tracking");

    json_fetch_string_with_default(parent, "imu_name", imu_name, CHAR_BUF_SIZE, "imu0");

    json_fetch_bool_with_default(parent, "camera_to_imu_pose_calibration", (int*)&camera_to_imu_pose_calibration, 1);
    json_fetch_bool_with_default(parent, "camera_intrinsics_calibration", (int*)&camera_intrinsics_calibration, 1);
    json_fetch_bool_with_default(parent, "camera_imu_timestamp_calibration", (int*)&camera_imu_timestamp_calibration, 1);

    json_fetch_double_with_default(parent, "delay_after_init", &delay_after_init, 0.0);
    json_fetch_bool_with_default(parent, "downsample_cams", (int*)&downsample_cams, 0);
    json_fetch_int_with_default(parent, "num_features_to_track", &num_features_to_track, 80);
    json_fetch_int_with_default(parent, "max_clone_size", &max_clone_size, 5);

    json_fetch_bool_with_default(parent, "use_zupt", (int*)&use_zupt, 0);
    json_fetch_double_with_default(parent, "zupt_max_velocity", &zupt_max_velocity, 0.1);
    json_fetch_bool_with_default(parent, "zupt_only_at_beginning", (int*)&zupt_only_at_beginning, 0);
    json_fetch_double_with_default(parent, "zupt_noise_multiplier", &zupt_noise_multiplier, 50.0);
    json_fetch_double_with_default(parent, "zupt_max_disparity", &zupt_max_disparity, 0.5);

    json_fetch_double_with_default(parent, "init_imu_thresh", &init_imu_thresh, 0.3);

    if (json_get_parse_error_flag()) {
        fprintf(stderr, "failed to parse config file %s\n", CONFIG_FILE);
        cJSON_Delete(parent);
        return -1;
    }

    // write modified data to disk if neccessary
    if (json_get_modified_flag()) {
        printf("The config file was modified during parsing, saving the changes to disk\n");
        json_write_to_file_with_header(CONFIG_FILE, parent, CONFIG_FILE_HEADER);
    }
    cJSON_Delete(parent);
    return 0;
}