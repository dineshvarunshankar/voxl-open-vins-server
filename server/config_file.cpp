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
#include <voxl_common_config.h>
#include <rc_math.h>

#include <stdio.h>
#include <iostream>

#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stereo.hpp>
using namespace cv;
using namespace std;

#include "config_file.h"

#define TRACKING_INTR_PATH "/data/modalai/opencv_tracking_intrinsics.yml"


#define CONFIG_FILE_HEADER "\
/**\n\
 * This file contains configuration that's specific to voxl-open-vins-server.\n\
 */\n"


// define all the externs from config_file.h
char cam0_pipe[CHAR_BUF_SIZE];
bool cam0_enable;
char cam0_extrinsics_name[CHAR_BUF_SIZE];
Eigen::Matrix<double, 7, 1> cam0_wrt_imu;
Eigen::Matrix<double, 8, 1> cam0_calib_intrinsic;

char cam1_pipe[CHAR_BUF_SIZE];
bool cam1_enable;
char cam1_extrinsics_name[CHAR_BUF_SIZE];
Eigen::Matrix<double, 7, 1> cam1_wrt_imu;
Eigen::Matrix<double, 8, 1> cam1_calib_intrinsic;

char cam2_pipe[CHAR_BUF_SIZE];
bool cam2_enable;
char cam2_extrinsics_name[CHAR_BUF_SIZE];
Eigen::Matrix<double, 7, 1> cam2_wrt_imu;
Eigen::Matrix<double, 8, 1> cam2_calib_intrinsic;

char cam3_pipe[CHAR_BUF_SIZE];
bool cam3_enable;
char cam3_extrinsics_name[CHAR_BUF_SIZE];
Eigen::Matrix<double, 7, 1> cam3_wrt_imu;
Eigen::Matrix<double, 8, 1> cam3_calib_intrinsic;

char imu_pipe[CHAR_BUF_SIZE];
char imu_name[CHAR_BUF_SIZE] = "imu0";

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

int load_extrinsics_file()
{
    vcc_extrinsic_t t[VCC_MAX_EXTRINSICS_IN_CONFIG];
    vcc_extrinsic_t tmp;

    // now load in extrinsics
    int n_extrinsics;
    if(vcc_read_extrinsic_conf_file(VCC_EXTRINSICS_PATH, t, &n_extrinsics, VCC_MAX_EXTRINSICS_IN_CONFIG)){
        fprintf(stderr, "ERROR: Unable to read extrinsics conf at %s\n", VCC_EXTRINSICS_PATH);
        return -1;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    // camera time
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    bool needs_inverse_transformation = false;
    if (cam0_enable){
        if(!vcc_find_extrinsic_in_array(cam0_extrinsics_name, imu_name, t, n_extrinsics, &tmp)){
            needs_inverse_transformation = false;
        }
        else if(!vcc_find_extrinsic_in_array(imu_name, cam0_extrinsics_name, t, n_extrinsics, &tmp)) {
            needs_inverse_transformation = true;
        }
        else {
            fprintf(stderr, "ERROR: %s missing %s to %s transform\n", VCC_EXTRINSICS_PATH, imu_name, cam0_extrinsics_name);
            return -1;
        }
        // Convert from degrees to radians
        tmp.RPY_parent_to_child[0] *= M_PI / 180.0;
        tmp.RPY_parent_to_child[1] *= M_PI / 180.0;
        tmp.RPY_parent_to_child[2] *= M_PI / 180.0;

        double q[4];
        rc_quaternion_from_tb_array(tmp.RPY_parent_to_child, q);

        Eigen::Matrix<double, 3, 1> translation;
        translation[0] = tmp.T_child_wrt_parent[0];
        translation[1] = tmp.T_child_wrt_parent[1];
        translation[2] = tmp.T_child_wrt_parent[2];

        // Convert to a quaternion
        Eigen::Matrix<double, 4, 1> quaternion;
        quaternion(0, 0) = q[1];
        quaternion(1, 0) = q[2];
        quaternion(2, 0) = q[3];
        quaternion(3, 0) = q[0];

        // If we need to invert the transformation (aka we have A->B but we want B->A)
        if (needs_inverse_transformation){
            quaternion = ov_core::Inv(quaternion);

            Eigen::Matrix<double, 3, 3> rot = ov_core::quat_2_Rot(quaternion);
            translation = rot * translation;
            translation = -translation;
        }

        cam0_wrt_imu.block(0, 0, 4, 1) = quaternion;
        cam0_wrt_imu.block(4, 0, 3, 1) = translation;
    }
    if (cam1_enable){
        if(!vcc_find_extrinsic_in_array(cam1_extrinsics_name, imu_name, t, n_extrinsics, &tmp)){
            needs_inverse_transformation = false;
        }
        else if(!vcc_find_extrinsic_in_array(imu_name, cam1_extrinsics_name, t, n_extrinsics, &tmp)) {
            needs_inverse_transformation = true;
        }
        else {
            fprintf(stderr, "ERROR: %s missing %s to %s transform\n", VCC_EXTRINSICS_PATH, imu_name, cam1_extrinsics_name);
            return -1;
        }
        // Convert from degrees to radians
        tmp.RPY_parent_to_child[0] *= M_PI / 180.0;
        tmp.RPY_parent_to_child[1] *= M_PI / 180.0;
        tmp.RPY_parent_to_child[2] *= M_PI / 180.0;

        double q[4];
        rc_quaternion_from_tb_array(tmp.RPY_parent_to_child, q);

        Eigen::Matrix<double, 3, 1> translation;
        translation[0] = tmp.T_child_wrt_parent[0];
        translation[1] = tmp.T_child_wrt_parent[1];
        translation[2] = tmp.T_child_wrt_parent[2];

        // Convert to a quaternion
        Eigen::Matrix<double, 4, 1> quaternion;
        quaternion(0, 0) = q[1];
        quaternion(1, 0) = q[2];
        quaternion(2, 0) = q[3];
        quaternion(3, 0) = q[0];

        // If we need to invert the transformation (aka we have A->B but we want B->A)
        if (needs_inverse_transformation){
            quaternion = ov_core::Inv(quaternion);

            Eigen::Matrix<double, 3, 3> rot = ov_core::quat_2_Rot(quaternion);
            translation = rot * translation;
            translation = -translation;
        }

        cam1_wrt_imu.block(0, 0, 4, 1) = quaternion;
        cam1_wrt_imu.block(4, 0, 3, 1) = translation;
    }
    if (cam2_enable){
        if(!vcc_find_extrinsic_in_array(cam2_extrinsics_name, imu_name, t, n_extrinsics, &tmp)){
            needs_inverse_transformation = false;
        }
        else if(!vcc_find_extrinsic_in_array(imu_name, cam2_extrinsics_name, t, n_extrinsics, &tmp)) {
            needs_inverse_transformation = true;
        }
        else {
            fprintf(stderr, "ERROR: %s missing %s to %s transform\n", VCC_EXTRINSICS_PATH, imu_name, cam2_extrinsics_name);
            return -1;
        }
        // Convert from degrees to radians
        tmp.RPY_parent_to_child[0] *= M_PI / 180.0;
        tmp.RPY_parent_to_child[1] *= M_PI / 180.0;
        tmp.RPY_parent_to_child[2] *= M_PI / 180.0;

        double q[4];
        rc_quaternion_from_tb_array(tmp.RPY_parent_to_child, q);

        Eigen::Matrix<double, 3, 1> translation;
        translation[0] = tmp.T_child_wrt_parent[0];
        translation[1] = tmp.T_child_wrt_parent[1];
        translation[2] = tmp.T_child_wrt_parent[2];

        // Convert to a quaternion
        Eigen::Matrix<double, 4, 1> quaternion;
        quaternion(0, 0) = q[1];
        quaternion(1, 0) = q[2];
        quaternion(2, 0) = q[3];
        quaternion(3, 0) = q[0];

        // If we need to invert the transformation (aka we have A->B but we want B->A)
        if (needs_inverse_transformation){
            quaternion = ov_core::Inv(quaternion);

            Eigen::Matrix<double, 3, 3> rot = ov_core::quat_2_Rot(quaternion);
            translation = rot * translation;
            translation = -translation;
        }

        cam2_wrt_imu.block(0, 0, 4, 1) = quaternion;
        cam2_wrt_imu.block(4, 0, 3, 1) = translation;
    }
    if (cam3_enable){
        if(!vcc_find_extrinsic_in_array(cam3_extrinsics_name, imu_name, t, n_extrinsics, &tmp)){
            needs_inverse_transformation = false;
        }
        else if(!vcc_find_extrinsic_in_array(imu_name, cam3_extrinsics_name, t, n_extrinsics, &tmp)) {
            needs_inverse_transformation = true;
        }
        else {
            fprintf(stderr, "ERROR: %s missing %s to %s transform\n", VCC_EXTRINSICS_PATH, imu_name, cam3_extrinsics_name);
            return -1;
        }
        // Convert from degrees to radians
        tmp.RPY_parent_to_child[0] *= M_PI / 180.0;
        tmp.RPY_parent_to_child[1] *= M_PI / 180.0;
        tmp.RPY_parent_to_child[2] *= M_PI / 180.0;

        double q[4];
        rc_quaternion_from_tb_array(tmp.RPY_parent_to_child, q);

        Eigen::Matrix<double, 3, 1> translation;
        translation[0] = tmp.T_child_wrt_parent[0];
        translation[1] = tmp.T_child_wrt_parent[1];
        translation[2] = tmp.T_child_wrt_parent[2];

        // Convert to a quaternion
        Eigen::Matrix<double, 4, 1> quaternion;
        quaternion(0, 0) = q[1];
        quaternion(1, 0) = q[2];
        quaternion(2, 0) = q[3];
        quaternion(3, 0) = q[0];

        // If we need to invert the transformation (aka we have A->B but we want B->A)
        if (needs_inverse_transformation){
            quaternion = ov_core::Inv(quaternion);
            Eigen::Matrix<double, 3, 3> rot = ov_core::quat_2_Rot(quaternion);
            translation = rot * translation;
            translation = -translation;
        }
        cam3_wrt_imu.block(0, 0, 4, 1) = quaternion;
        cam3_wrt_imu.block(4, 0, 3, 1) = translation;
    }
    return 0;
}

int load_intrinsics_file(){
    // use opencv to open file
	FileStorage fs(TRACKING_INTR_PATH, FileStorage::READ);
	if(!fs.isOpened()){
		fprintf(stderr, "Failed to load lens cal file %s\n", TRACKING_INTR_PATH);
		return -1;
	}

	FileNode n;
	Mat camMatrix;
	Mat distCoeffs;
	int is_fisheye = 0;
	int w,h;
	int has_m = 0;
	int has_d = 0;
	int has_w = 0;
	int has_h = 0;

	// opencv cal files don't have consistent names for matrices, so try a few

	// name for mono camera
	n = fs["M"];
	if(n.type() != FileNode::NONE){
		n >> camMatrix;
		has_m = 1;
	}
	n = fs["D"];
	if(n.type() != FileNode::NONE){
		n >> distCoeffs;
		has_d = 1;
	}

	// // name for stereo left
	// n = fs["M1"];
	// if(n.type() != FileNode::NONE){
	// 	n >> camMatrix;
	// 	has_m = 1;
	// }
	// n = fs["D1"];
	// if(n.type() != FileNode::NONE){
	// 	n >> distCoeffs;
	// 	has_d = 1;
	// }

	// // name used in aruco example
	// n = fs["camera_matrix"];
	// if(n.type() != FileNode::NONE){
	// 	n >> camMatrix;
	// 	has_m = 1;
	// }
	// n = fs["distortion_coefficients"];
	// if(n.type() != FileNode::NONE){
	// 	n >> distCoeffs;
	// 	has_d = 1;
	// }

	// check for fisheye model
	n = fs["distortion_model"];
	if(n.isString()){
		std::string mdl_name = n;
		if(mdl_name.compare("fisheye") == 0){
			is_fisheye=1;
		}
	}

	// check height and width
	n = fs["width"];
	if(n.type() != FileNode::NONE){
		n >> w;
		has_w = 1;
	}
	n = fs["height"];
	if(n.type() != FileNode::NONE){
		n >> h;
		has_h = 1;
	}

	// done with file now
	fs.release();

	// make sure we loaded the matrices in
	if(!has_m){
		fprintf(stderr, "failed to find camera matrix in %s\n", TRACKING_INTR_PATH);
	}
	if(!has_d){
		fprintf(stderr, "failed to find distortion coefficients in %s\n", TRACKING_INTR_PATH);
	}
	if(!has_w){
		fprintf(stderr, "failed to find width in %s\n", TRACKING_INTR_PATH);
	}
	if(!has_h){
		fprintf(stderr, "failed to find height in %s\n", TRACKING_INTR_PATH);
	}
	if(!has_m || !has_d){
		return -1;
	}

	// populate an mcv undistortion map with the data we have
	cam0_calib_intrinsic(0,0) = camMatrix.at<double>(0,0);
	cam0_calib_intrinsic(1,0) = camMatrix.at<double>(1,1);
	cam0_calib_intrinsic(2,0) = camMatrix.at<double>(0,2);
	cam0_calib_intrinsic(3,0) = camMatrix.at<double>(1,2);
	cam0_calib_intrinsic(4,0) = distCoeffs.at<double>(0);
	cam0_calib_intrinsic(5,0) = distCoeffs.at<double>(1);
	cam0_calib_intrinsic(6,0) = distCoeffs.at<double>(2);
	cam0_calib_intrinsic(7,0) = distCoeffs.at<double>(3);

    return 0;


	// p.w = w;
	// p.h = h;
	// p.fx = camMatrix.at<double>(0,0);
	// p.fy = camMatrix.at<double>(1,1);
	// p.cx = camMatrix.at<double>(0,2);
	// p.cy = camMatrix.at<double>(1,2);

	// // fisheye mode for tracking
	// if(is_fisheye){
	// 	p.n_coeffs = 4;
	// 	p.is_fisheye = 1;
	// }
	// // polynomial mode for stereo
	// else{
	// 	p.n_coeffs = 5;
	// 	p.is_fisheye = 0;
	// }

	// // copy in distortion coefficients
	// for(int i=0;i<p.n_coeffs;i++){
	// 	p.D[i] = (float)distCoeffs.at<double>(i);
	// }
}

int config_file_print(void)
{
    printf("=================================================================\n");
    printf("==========================CAMERA 0================================\n");
    printf("pipe:                             %s\n",    cam0_pipe);
    printf("enable:                           %s\n",    cam0_enable ? "true" : "false");
    printf("extrinsics name:                  %s\n",    cam0_extrinsics_name);
    printf("==========================CAMERA 2================================\n");
    printf("pipe:                             %s\n",    cam1_pipe);
    printf("enable:                           %s\n",    cam1_enable ? "true" : "false");
    printf("extrinsics name:                  %s\n",    cam1_extrinsics_name);
    printf("==========================CAMERA 3================================\n");
    printf("pipe:                             %s\n",    cam2_pipe);
    printf("enable:                           %s\n",    cam2_enable ? "true" : "false");
    printf("extrinsics name:                  %s\n",    cam2_extrinsics_name);
    printf("==========================CAMERA 4================================\n");
    printf("pipe:                             %s\n",    cam3_pipe);
    printf("enable:                           %s\n",    cam3_enable ? "true" : "false");
    printf("extrinsics name:                  %s\n",    cam3_extrinsics_name);
    printf("===========================IMU====================================\n");
    printf("imu pipe:                         %s\n",    imu_pipe);
    printf("==========================GENERAL=================================\n");
    printf("camera_to_imu_pose_calibration:   %s\n",    camera_to_imu_pose_calibration ? "true" : "false");
    printf("camera_intrinsics_calibration:    %s\n",    camera_intrinsics_calibration ? "true" : "false");
    printf("camera_imu_timestamp_calibration: %s\n",    camera_imu_timestamp_calibration ? "true" : "false");
    printf("delay after init(seconds):        %6.5f\n",    delay_after_init);
    printf("downsample cams:                  %s\n",    downsample_cams ? "true" : "false");
    printf("features to track:                %d\n",    num_features_to_track);
    printf("max clone size:                   %d\n",    max_clone_size);
    printf("use zupt:                         %s\n",    use_zupt ? "true" : "false");
    if (use_zupt){
        printf("zupt max velocity:                %6.5f\n",    zupt_max_velocity);
        printf("use zupt only at beginning:       %s\n",       zupt_only_at_beginning ? "true" : "false");
        printf("zupt noise multiplier:            %6.5f\n",    zupt_noise_multiplier);
        printf("zupt max disparity:               %6.5f\n",    zupt_max_disparity);
    }
    else printf("init imu thresh:                  %6.5f\n",    init_imu_thresh);
    printf("=================================================================\n");
    return 0;
}

/**
 * load the config file and populate the above extern variables
 *
 * @return     0 on success, -1 on failure
 */
int config_file_read(void)
{
    int ret = json_make_empty_file_with_header_if_missing(CONFIG_FILE, CONFIG_FILE_HEADER);
    if(ret < 0) return -1;
    else if(ret>0) fprintf(stderr, "Creating new config file: %s\n", CONFIG_FILE);

    cJSON* parent = json_read_file(CONFIG_FILE);
    if(parent==NULL) return -1;

    // actually parse values
    json_fetch_string_with_default(parent, "cam0_pipe", cam0_pipe, CHAR_BUF_SIZE, "/run/mpa/tracking");
    json_fetch_bool_with_default(parent, "cam0_enable", (int*)&cam0_enable, 1);
    json_fetch_string_with_default(parent, "cam0_extrinsics_name", cam0_extrinsics_name, CHAR_BUF_SIZE, "tracking");

    json_fetch_string_with_default(parent, "cam1_pipe", cam1_pipe, CHAR_BUF_SIZE, "/run/mpa/tracking");
    json_fetch_bool_with_default(parent, "cam1_enable", (int*)&cam1_enable, 0);
    json_fetch_string_with_default(parent, "cam1_extrinsics_name", cam1_extrinsics_name, CHAR_BUF_SIZE, "tracking");
    
    json_fetch_string_with_default(parent, "cam2_pipe", cam2_pipe, CHAR_BUF_SIZE, "/run/mpa/tracking");
    json_fetch_bool_with_default(parent, "cam2_enable", (int*)&cam2_enable, 0);
    json_fetch_string_with_default(parent, "cam2_extrinsics_name", cam2_extrinsics_name, CHAR_BUF_SIZE, "tracking");

    json_fetch_string_with_default(parent, "cam3_pipe", cam3_pipe, CHAR_BUF_SIZE, "/run/mpa/tracking");
    json_fetch_bool_with_default(parent, "cam3_enable", (int*)&cam3_enable, 0);
    json_fetch_string_with_default(parent, "cam3_extrinsics_name", cam3_extrinsics_name, CHAR_BUF_SIZE, "tracking");

    json_fetch_string_with_default(parent, "imu_pipe", imu_pipe, CHAR_BUF_SIZE, "/run/mpa/imu0");

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

    if(json_get_parse_error_flag()){
        fprintf(stderr, "failed to parse config file %s\n", CONFIG_FILE);
        cJSON_Delete(parent);
        return -1;
    }

    // write modified data to disk if neccessary
    if(json_get_modified_flag()){
        printf("The config file was modified during parsing, saving the changes to disk\n");
        json_write_to_file_with_header(CONFIG_FILE, parent, CONFIG_FILE_HEADER);
    }
    cJSON_Delete(parent);
    return 0;
}