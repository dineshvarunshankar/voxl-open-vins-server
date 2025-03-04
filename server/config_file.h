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


#ifndef CONFIG_FILE_H
#define CONFIG_FILE_H

#include <Eigen/Core>
#include <types/LandmarkRepresentation.h>
#include <vector>

// cameras are enumerated individually, so a stereo pair counts as two
#define CHAR_BUF_SIZE 128
#define CONFIG_FILE "/etc/modalai/voxl-open-vins-server.conf"
#define CONFIG_FILE_HEADER \
    "\
/**\n\
 * This file contains configuration that's specific to voxl-open-vins-server.\n\
 * \n\
 * *NOTE*: all time variables are measured in seconds\n\
 * \n\
 * OpenVins param breakdown:\n\
 * \n\
 * do_fej: whether or not to do first estimate Jacobians\n\
 * imu_avg: whether or not use imu message averaging\n\
 * use_rk4_integration: if we should use Rk4 imu integration.\n\
 * cam_to_imu_refinement: whether or not to refine the imu-to-camera pose\n\
 * cam_intrins_refinement: whether or not to refine camera intrinsics\n\
 * cam_imu_ts_refinement: whether or not to calibrate cam to IMU time offset\n\
 * max_clone_size: max clone size of sliding window\n\
 * max_slam_features: max number of estimated SLAM features\n\
 * max_slam_in_update: max number of SLAM features in a single EKF update\n\
 * max_msckf_in_update: max number of MSCKF features used at an image timestep\n\n\
 * \n\
 * Feature Reps can be any of the following:\n\
 * 0 - GLOBAL_3D\n\
 * 1 - GLOBAL_FULL_INVERSE_DEPTH\n\
 * 2 - ANCHORED_3D\n\
 * 3 - ANCHORED_FULL_INVERSE_DEPTH\n\
 * 4 - ANCHORED_MSCKF_INVERSE_DEPTH\n\
 * 5 - ANCHORED_INVERSE_DEPTH_SINGLE\n\
 * feat_rep_msckf: (int) what representation our msckf features are in\n\
 * feat_rep_slam: (int) what representation our slam features are in\n\n\
 * cam_imu_time_offset: time offset between camera and IMU\n\
 * slam_delay: delay that we should wait from init before estimating SLAM features\n\
 * gravity_mag: gravity magnitude in the global frame\n\
 * init_window_time: amount of time to initialize over\n\
 * init_imu_thresh: variance threshold on our accel to be classified as moving\n\
 * \n\
 * imu_sigma_w: gyroscope white noise (rad/s/sqrt(hz))\n\
 * imu_sigma_wb: gyroscope random walk (rad/s^2/sqrt(hz))\n\
 * imu_sigma_a: accelerometer white noise (m/s^2/sqrt(hz))\n\
 * imu_sigma_ab: accelerometer random walk (m/s^3/sqrt(hz))\n\
 * imu_sigma_w_2: gyroscope white noise covariance\n\
 * imu_sigma_wb_2: gyroscope random walk covariance\n\
 * imu_sigma_a_2: accelerometer white noise covariance\n\
 * imu_sigma_ab_2: accelerometer random walk covariance\n\
 * \n\
 * ****_chi2_multiplier: what chi-squared multipler we should apply\n\
 * ****_sigma_px: noise sigma for our raw pixel measurements\n\
 * ****_sigma_px_sq: covariance for our raw pixel measurements\n\
 * use_stereo: if feed_measurement_camera is called with more than one\n\
 * image, this determines behavior. if true, they are treated as a stereo\n\
 * pair, otherwise treated as binocular system\n\
 * if you enable a camera with stereo in the name, this will be set to true\n\
 * automatically\n\
 * \n\
 * try_zupt: if we should try to use zero velocity update\n\
 * zupt_max_velocity: max velocity we will consider to try to do a zupt\n\
 * zupt_only_at_beginning: if we should only use the zupt at the very beginning\n\
 * zupt_noise_multiplier: multiplier of our zupt measurement IMU noise matrix\n\
 * zupt_max_disparity: max disparity we will consider to try to do a zupt\n\
 * *NOTE*: set zupt_max_disparity to 0 for only imu based zupt, and\n\
 * zupt_chi2_multipler to 0 for only display based zupt\n\
 * \n\
 * num_pts: number of points we should extract and track in each image frame\n\
 * fast_threshold: fast extraction threshold\n\
 * grid_x: number of column-wise grids to do feature extraction in\n\
 * grid_y: number of row-wise grids to do feature extraction in\n\
 * min_px_dist: after doing KLT track will remove any features closer than this\n\
 * knn_ratio: KNN ration between top two descriptor matchers for good match\n\
 * downsample_cams: will half image resolution\n\
 * use_nultithreading: if we should use multi-threading for stereo matching\n\
 * use_mask: if we should load a mask and use it to reject invalid features\n\
 */\n"



// auto reset parameters
extern int en_auto_reset;
 extern float auto_reset_max_velocity;
extern float auto_reset_max_v_cov_instant;
extern float auto_reset_max_v_cov;
extern float auto_reset_max_v_cov_timeout_s;
extern int   auto_reset_min_features;
extern float auto_reset_min_feature_timeout_s;
extern float auto_fallback_timeout_s;
extern float auto_fallback_min_v;
extern bool  en_cont_yaw_checks;
extern float fast_yaw_thresh;
extern float fast_yaw_timeout_s;

/// STATE OPTIONS ///
extern bool do_fej;
extern bool imu_avg;
extern bool use_rk4_integration;

extern bool cam_to_imu_refinement;
extern bool cam_intrins_refinement;
extern bool cam_imu_ts_refinement;

extern int max_clone_size;
extern int max_slam_features;
extern int max_slam_in_update;
extern int max_msckf_in_update;

extern ov_type::LandmarkRepresentation::Representation feat_rep_msckf;
extern ov_type::LandmarkRepresentation::Representation feat_rep_slam;

extern double cam_imu_time_offset;
extern double slam_delay;

/// INERTIAL INITIALIZER OPTIONS ///
extern double gravity_mag;
extern double init_window_time;
extern double init_imu_thresh;
extern double init_imu_thresh_accel;

extern bool init_dyn_use;

/// IMU NOISE OPTIONS ///
extern double imu_sigma_w;
extern double imu_sigma_wb;
extern double imu_sigma_a;
extern double imu_sigma_ab;

/// FEATURE OPTIONS ///
extern double msckf_chi2_multiplier;
extern double msckf_sigma_px;

extern double slam_chi2_multiplier;
extern double slam_sigma_px;

extern double zupt_chi2_multiplier;
extern double zupt_sigma_px;

extern bool use_stereo;
extern bool use_mask;

/// ZUPT OPTIONS ///
extern bool try_zupt;
extern double zupt_max_velocity;
extern bool zupt_only_at_beginning;
extern double zupt_noise_multiplier;
extern double zupt_max_disparity;

/// FEATURE INITIALIZER OPTIONS ///
extern bool triangulate_1d;
extern bool refine_features;
extern int max_runs;
extern double init_lamda;
extern double max_lamda;
extern double min_dx;
extern double min_dcost;
extern double lam_mult;
extern double min_dist;
extern double max_dist;
extern double max_baseline;
extern double max_cond_number;

extern int num_opencv_threads;
extern int fast_threshold;
extern ov_core::TrackBase::HistogramMethod histogram_method;
extern double knn_ratio;
extern double track_frequency;
extern int publish_frequency;
extern bool en_ov_stats;
extern bool use_baro;
extern double takeoff_threshold;
extern double max_allowable_cep;
extern bool en_force_init;
extern bool en_vio_always_on;
extern bool en_overlay_landscape;
extern bool en_force_ned_2_flu;
extern bool en_imu_frame_output;
extern int en_ext_feature_tracker;
extern int en_gpu_for_tracking;
extern int num_features_to_track;
extern bool en_thermal_enhance;
extern double thermal_brightness;
extern double thermal_brightness_bos;
extern bool raansac_gn;
extern bool raansac_tri;

// read only our own config file without printing the contents
int config_file_read(void);

// prints the current configuration values to the screen.
int config_file_print(void);


#endif // end CONFIG_FILE_H
