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

#include <core/VioManagerOptions.h>

#include <modal_json.h>
#include <rc_math.h>
#include <stdio.h>
#include <voxl_common_config.h>

#include <string>

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
float odr_hz;

/// STATE OPTIONS ///
bool do_fej;
bool imu_avg;
bool use_rk4_integration;

bool cam_to_imu_refinement;
bool cam_intrins_refinement;
bool cam_imu_ts_refinement;

int max_clone_size;
int max_slam_features;
int max_slam_in_update;
int max_msckf_in_update;

ov_type::LandmarkRepresentation::Representation feat_rep_msckf;
ov_type::LandmarkRepresentation::Representation feat_rep_slam;

double cam_imu_time_offset;
double slam_delay;

/// INERTIAL INITIALIZER OPTIONS ///
double gravity_mag;
double init_window_time;
double init_imu_thresh;

/// IMU NOISE OPTIONS ///
double imu_sigma_w;
double imu_sigma_wb;
double imu_sigma_a;
double imu_sigma_ab;
double imu_sigma_w_2;
double imu_sigma_wb_2;
double imu_sigma_a_2;
double imu_sigma_ab_2;

/// FEATURE OPTIONS ///
double msckf_chi2_multiplier;
double msckf_sigma_px;
double msckf_sigma_px_sq;

double slam_chi2_multiplier;
double slam_sigma_px;
double slam_sigma_px_sq;

double zupt_chi2_multiplier;
double zupt_sigma_px;
double zupt_sigma_px_sq;

bool use_stereo;

/// ZUPT OPTIONS ///
bool try_zupt;
double zupt_max_velocity;
bool zupt_only_at_beginning;
double zupt_noise_multiplier;
double zupt_max_disparity;

/// TRACKER + EXTRACTOR OPTIONS ///
bool use_klt;
int num_pts;
int fast_threshold;
int grid_x;
int grid_y;
int min_px_dist;
double knn_ratio;
bool downsample_cams;
bool use_multithreading;
bool use_mask;

static std::string feat_set_as_string(ov_type::LandmarkRepresentation::Representation feat_representation) {
    if (feat_representation == ov_type::LandmarkRepresentation::Representation::GLOBAL_3D)
      return "GLOBAL_3D";
    if (feat_representation == ov_type::LandmarkRepresentation::Representation::GLOBAL_FULL_INVERSE_DEPTH)
      return "GLOBAL_FULL_INVERSE_DEPTH";
    if (feat_representation == ov_type::LandmarkRepresentation::Representation::ANCHORED_3D)
      return "ANCHORED_3D";
    if (feat_representation == ov_type::LandmarkRepresentation::Representation::ANCHORED_FULL_INVERSE_DEPTH)
      return "ANCHORED_FULL_INVERSE_DEPTH";
    if (feat_representation == ov_type::LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH)
      return "ANCHORED_MSCKF_INVERSE_DEPTH";
    if (feat_representation == ov_type::LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE)
      return "ANCHORED_INVERSE_DEPTH_SINGLE";
    return "UNKNOWN";
  }

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
            if (strstr(ext_name, "stereo") != NULL){
                use_stereo = true;
                strcat(ext_name, "_l");
            }

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
            // opencv_cam_name_intrinsics.ymls
            strcpy(intrinsics_path, "/data/modalai/opencv_");
            strcat(intrinsics_path, cam_info_vec[i].name);
            strcat(intrinsics_path, "_intrinsics.yml");

            FileStorage fs(intrinsics_path, FileStorage::READ);
            if(!fs.isOpened()){
                fprintf(stderr, "Failed to load intrinsics file %s\n", intrinsics_path);
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
    printf("============================IMU==================================\n");
    printf("imu pipe:                         %s\n", imu_name);
	printf("odr_hz:                           %6.3f\n", (double)odr_hz);
    printf("=================================================================\n");
    printf("===========================STATE=================================\n");
    printf("do fej:                           %s\n", do_fej ? "true" : "false");
    printf("imu avg:                          %s\n", imu_avg ? "true" : "false");
    printf("use rk4 integration:              %s\n", use_rk4_integration ? "true" : "false");
    printf("cam to imu refinement:            %s\n", cam_to_imu_refinement ? "true" : "false");
    printf("cam intrins refinement:           %s\n", cam_intrins_refinement ? "true" : "false");
    printf("cam imu ts refinement:            %s\n", cam_imu_ts_refinement ? "true" : "false");
    printf("max clone size:                   %d\n", max_clone_size);
    printf("max slam features:                %d\n", max_slam_features);
    printf("max slam in update:               %d\n", max_slam_in_update);
    printf("max msckf in update:              %d\n", max_msckf_in_update);
    printf("feat rep msckf:                   %s\n", feat_set_as_string(feat_rep_msckf).c_str());
    printf("feat rep slam:                    %s\n", feat_set_as_string(feat_rep_slam).c_str());
    printf("cam imu time offset:              %6.5f\n", cam_imu_time_offset);
    printf("slam delay:                       %6.5f\n", slam_delay);
    printf("=================================================================\n");
    printf("=====================INERTIAL INITIALIZER========================\n");
    printf("gravity mag:                      %6.5f\n", gravity_mag);
    printf("init window time:                 %6.5f\n", init_window_time);
    printf("init imu thresh:                  %6.5f\n", init_imu_thresh);
    printf("=================================================================\n");
    printf("==========================IMU NOISE==============================\n");
    printf("imu sigma w:                      %6.5f\n", imu_sigma_w);
    printf("imu sigma wb:                     %6.5f\n", imu_sigma_wb);
    printf("imu sigma a:                      %6.5f\n", imu_sigma_a);
    printf("imu sigma ab:                     %6.5f\n", imu_sigma_ab);
    printf("imu sigma w^2:                    %6.5f\n", imu_sigma_w_2);
    printf("imu sigma wb^2:                   %6.5f\n", imu_sigma_wb_2);
    printf("imu sigma a^2:                    %6.5f\n", imu_sigma_a_2);
    printf("imu sigma ab^2:                   %6.5f\n", imu_sigma_ab_2);
    printf("=================================================================\n");
    printf("========================FEATURE NOISE============================\n");
    printf("msckf chi^2 multiplier:           %6.5f\n", msckf_chi2_multiplier);
    printf("msckf sigma px:                   %6.5f\n", msckf_sigma_px);
    printf("msckf sigma px^2:                 %6.5f\n", msckf_sigma_px_sq);
    printf("slam chi^2 multiplier:            %6.5f\n", slam_chi2_multiplier);
    printf("slam sigma px:                    %6.5f\n", slam_sigma_px);
    printf("slam sigma px^2:                  %6.5f\n", slam_sigma_px_sq);
    printf("zupt chi^2 multiplier:            %6.5f\n", zupt_chi2_multiplier);
    printf("zupt sigma px:                    %6.5f\n", zupt_sigma_px);
    printf("zupt sigma px^2:                  %6.5f\n", zupt_sigma_px_sq);
    printf("=================================================================\n");
    printf("=============================ZUPT================================\n");
    printf("try zupt:                         %s\n", try_zupt ? "true" : "false");
    printf("zupt max velocity:                %6.5f\n", zupt_max_velocity);
    printf("zupt_only_at_beginning:           %s\n", zupt_only_at_beginning ? "true" : "false");
    printf("zupt noise multiplier:            %6.5f\n", zupt_noise_multiplier);
    printf("zupt max disparity:               %6.5f\n", zupt_max_disparity);
    printf("=================================================================\n");
    printf("=======================TRACKER/EXTRACTOR=========================\n");
    printf("use klt:                          %s\n", use_klt ? "true" : "false");
    printf("num pts:                          %d\n", num_pts);
    printf("fast threshold:                   %d\n", fast_threshold);
    printf("grid x:                           %d\n", grid_x);
    printf("grid y:                           %d\n", grid_y);
    printf("min px dist:                      %d\n", min_px_dist);
    printf("knn ratio:                        %6.5f\n", knn_ratio);
    printf("downsample cams:                  %s\n", downsample_cams ? "true" : "false");
    printf("use multithreading:               %s\n", use_multithreading ? "true" : "false");
    printf("use mask:                         %s\n", use_mask ? "true" : "false");
    printf("use stereo:                       %s\n", use_stereo ? "true" : "false");
    printf("=================================================================\n");
    printf("=================================================================\n");
    return 0;
}

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
    json_fetch_string_with_default(parent, "cam3_pipe", cam_info_vec[3].name, CHAR_BUF_SIZE, "hires");

    json_fetch_string_with_default(parent, "imu_name", imu_name, CHAR_BUF_SIZE, "imu_apps");
	json_fetch_float_with_default(parent, "odr_hz", &odr_hz, 30);

    json_fetch_bool_with_default(parent, "do_fej", (int*)&do_fej, 1);
    json_fetch_bool_with_default(parent, "imu_avg", (int*)&imu_avg, 1);
    json_fetch_bool_with_default(parent, "use_rk4_integration", (int*)&use_rk4_integration, 1);

    json_fetch_bool_with_default(parent, "cam_to_imu_refinement", (int*)&cam_to_imu_refinement, 1);
    json_fetch_bool_with_default(parent, "cam_intrins_refinement", (int*)&cam_intrins_refinement, 1);
    json_fetch_bool_with_default(parent, "cam_imu_ts_refinement", (int*)&cam_imu_ts_refinement, 1);

    json_fetch_int_with_default(parent, "max_clone_size", &max_clone_size, 25);
    json_fetch_int_with_default(parent, "max_slam_features", &max_slam_features, 50);
    json_fetch_int_with_default(parent, "max_slam_in_update", &max_slam_in_update, 25);
    json_fetch_int_with_default(parent, "max_msckf_in_update", &max_msckf_in_update, 40);

    json_fetch_int_with_default(parent, "feat_rep_msckf", (int*)&feat_rep_msckf, 0);
    json_fetch_int_with_default(parent, "feat_rep_slam", (int*)&feat_rep_slam, 4);

    json_fetch_double_with_default(parent, "cam_imu_time_offset", &cam_imu_time_offset, -0.002);
    json_fetch_double_with_default(parent, "slam_delay", &slam_delay, 1.0);

    json_fetch_double_with_default(parent, "gravity_mag", &gravity_mag, 9.81);
    json_fetch_double_with_default(parent, "init_window_time", &init_window_time, 2.0);
    json_fetch_double_with_default(parent, "init_imu_thresh", &init_imu_thresh, 1.5);

    json_fetch_double_with_default(parent, "imu_sigma_w", &imu_sigma_w, 1.6968e-02);
    json_fetch_double_with_default(parent, "imu_sigma_wb", &imu_sigma_wb, 1.9393e-02);
    json_fetch_double_with_default(parent, "imu_sigma_a", &imu_sigma_a, 2.0000e-2);
    json_fetch_double_with_default(parent, "imu_sigma_ab", &imu_sigma_ab, 3.0000e-02);
    json_fetch_double_with_default(parent, "imu_sigma_w_2", &imu_sigma_w_2, pow(1.6968e-02, 2));
    json_fetch_double_with_default(parent, "imu_sigma_wb_2", &imu_sigma_wb_2, pow(1.9393e-02, 2));
    json_fetch_double_with_default(parent, "imu_sigma_a_2", &imu_sigma_a_2, pow(2.0000e-2, 2));
    json_fetch_double_with_default(parent, "imu_sigma_ab_2", &imu_sigma_ab_2, pow(3.0000e-02, 2));

    json_fetch_double_with_default(parent, "msckf_chi2_multiplier", &msckf_chi2_multiplier, 0.1);
    json_fetch_double_with_default(parent, "msckf_sigma_px", &msckf_sigma_px, 5.0);
    json_fetch_double_with_default(parent, "msckf_sigma_px_sq", &msckf_sigma_px_sq, 25.0);

    json_fetch_double_with_default(parent, "slam_chi2_multiplier", &slam_chi2_multiplier, 0.1);
    json_fetch_double_with_default(parent, "slam_sigma_px", &slam_sigma_px, 5.0);
    json_fetch_double_with_default(parent, "slam_sigma_px_sq", &slam_sigma_px_sq, 25.0);

    json_fetch_double_with_default(parent, "zupt_chi2_multiplier", &zupt_chi2_multiplier, 0.0);
    json_fetch_double_with_default(parent, "zupt_sigma_px", &zupt_sigma_px, 1.0);
    json_fetch_double_with_default(parent, "zupt_sigma_px_sq", &zupt_sigma_px_sq, 1.0);

    json_fetch_bool_with_default(parent, "try_zupt", (int*)&try_zupt, 1);
    json_fetch_double_with_default(parent, "zupt_max_velocity", &zupt_max_velocity, 0.1);
    json_fetch_bool_with_default(parent, "zupt_only_at_beginning", (int*)&zupt_only_at_beginning, 1);
    json_fetch_double_with_default(parent, "zupt_noise_multiplier", &zupt_noise_multiplier, 50.0);
    json_fetch_double_with_default(parent, "zupt_max_disparity", &zupt_max_disparity, 1.5);

    json_fetch_bool_with_default(parent, "use_klt", (int*)&use_klt, 1);
    json_fetch_int_with_default(parent, "num_pts", &num_pts, 80);
    json_fetch_int_with_default(parent, "fast_threshold", &fast_threshold, 15);
    json_fetch_int_with_default(parent, "grid_x", &grid_x, 20);
    json_fetch_int_with_default(parent, "grid_y", &grid_y, 16);
    json_fetch_int_with_default(parent, "min_px_dist", &min_px_dist, 10);
    json_fetch_double_with_default(parent, "knn_ratio", &knn_ratio, 0.70);
    json_fetch_bool_with_default(parent, "downsample_cams", (int*)&downsample_cams, 0);
    json_fetch_bool_with_default(parent, "use_multithreading", (int*)&use_multithreading, 0);
    json_fetch_bool_with_default(parent, "use_mask", (int*)&use_mask, 0);
    json_fetch_bool_with_default(parent, "use_stereo", (int*)&use_stereo, 0);


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