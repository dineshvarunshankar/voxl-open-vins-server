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
#include <stdio.h>

#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stereo.hpp>
#include <string>

#include <iostream>

using namespace cv;
using namespace std;

#include "config_file.h"

// define all the externs from config_file.h
// auto reset parameters
int en_auto_reset;
float auto_reset_max_velocity;
float auto_reset_max_v_cov_instant;
float auto_reset_max_v_cov;
float auto_reset_max_v_cov_timeout_s;
int   auto_reset_min_features;
float auto_reset_min_feature_timeout_s;

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

/// FEATURE OPTIONS ///
double msckf_chi2_multiplier;
double msckf_sigma_px;

double slam_chi2_multiplier;
double slam_sigma_px;

double zupt_chi2_multiplier;
double zupt_sigma_px;

/// ZUPT OPTIONS ///
bool try_zupt;
double zupt_max_velocity;
bool zupt_only_at_beginning;
double zupt_noise_multiplier;
double zupt_max_disparity;

/// FEATURE INITIALIZER OPTIONS ///
bool triangulate_1d;
bool refine_features;
int max_runs;
double init_lamda;
double max_lamda;
double min_dx;
double min_dcost;
double lam_mult;
double min_dist;
double max_dist;
double max_baseline;
double max_cond_number;
bool use_stereo;
bool use_mask;

int num_opencv_threads;
int fast_threshold;
ov_core::TrackBase::HistogramMethod histogram_method;
double knn_ratio;
double track_frequency;

bool en_ov_stats;
bool en_baro;

//std::string log_path;

static std::string feat_set_as_string(ov_type::LandmarkRepresentation::Representation feat_representation) {
    if (feat_representation == ov_type::LandmarkRepresentation::Representation::GLOBAL_3D)
        return "GLOBAL_3D";
    else if (feat_representation == ov_type::LandmarkRepresentation::Representation::GLOBAL_FULL_INVERSE_DEPTH)
        return "GLOBAL_FULL_INVERSE_DEPTH";
    else if (feat_representation == ov_type::LandmarkRepresentation::Representation::ANCHORED_3D)
        return "ANCHORED_3D";
    else if (feat_representation == ov_type::LandmarkRepresentation::Representation::ANCHORED_FULL_INVERSE_DEPTH)
        return "ANCHORED_FULL_INVERSE_DEPTH";
    else if (feat_representation == ov_type::LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH)
        return "ANCHORED_MSCKF_INVERSE_DEPTH";
    else if (feat_representation == ov_type::LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE)
        return "ANCHORED_INVERSE_DEPTH_SINGLE";
    else return "UNKNOWN";
}

static ov_core::TrackBase::HistogramMethod int_to_hist_method(int hist_type) {

	if (hist_type == 0)
			return ov_core::TrackBase::HistogramMethod::NONE;
	else if (hist_type == 1)
			return ov_core::TrackBase::HistogramMethod::HISTOGRAM;
	else if (hist_type == 2)
			return ov_core::TrackBase::HistogramMethod::CLAHE;
	else
		return ov_core::TrackBase::HistogramMethod::NONE;
}

static std::string hist_as_string(ov_core::TrackBase::HistogramMethod hist_rep) {
	if (hist_rep == ov_core::TrackBase::HistogramMethod::NONE)
			return "NONE";
	else if (hist_rep == ov_core::TrackBase::HistogramMethod::HISTOGRAM)
		return "HISTOGRAM";
	else if (hist_rep == ov_core::TrackBase::HistogramMethod::CLAHE)
		return "CLAHE";
	else
	    return "UNKNOWN";

}

int config_file_print(void) {
    printf("=================================================================\n");
    printf("=========================AUTO RESET==============================\n");
	printf("en_auto_reset:                    %d\n",    en_auto_reset);
	printf("auto_reset_max_velocity:          %6.3f\n", (double)auto_reset_max_velocity);
	printf("auto_reset_max_v_cov_instant:     %6.3f\n", (double)auto_reset_max_v_cov_instant);
	printf("auto_reset_max_v_cov:             %6.3f\n", (double)auto_reset_max_v_cov);
	printf("auto_reset_max_v_cov_timeout_s:   %6.3f\n", (double)auto_reset_max_v_cov_timeout_s);
	printf("auto_reset_min_features:          %d\n",    auto_reset_min_features);
	printf("auto_reset_min_feature_timeout_s: %6.3f\n", (double)auto_reset_min_feature_timeout_s);
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
    printf("=================================================================\n");
    printf("========================FEATURE NOISE============================\n");
    printf("msckf chi^2 multiplier:           %6.5f\n", msckf_chi2_multiplier);
    printf("msckf sigma px:                   %6.5f\n", msckf_sigma_px);
    printf("slam chi^2 multiplier:            %6.5f\n", slam_chi2_multiplier);
    printf("slam sigma px:                    %6.5f\n", slam_sigma_px);
    printf("zupt chi^2 multiplier:            %6.5f\n", zupt_chi2_multiplier);
    printf("zupt sigma px:                    %6.5f\n", zupt_sigma_px);
    printf("=================================================================\n");
    printf("=============================ZUPT================================\n");
    printf("try zupt:                         %s\n", try_zupt ? "true" : "false");
    printf("zupt max velocity:                %6.5f\n", zupt_max_velocity);
    printf("zupt only at beginning:           %s\n", zupt_only_at_beginning ? "true" : "false");
    printf("zupt noise multiplier:            %6.5f\n", zupt_noise_multiplier);
    printf("zupt max disparity:               %6.5f\n", zupt_max_disparity);
    printf("=================================================================\n");
    printf("use mask:                         %s\n", use_mask ? "true" : "false");
    printf("use stereo:                       %s\n", use_stereo ? "true" : "false");
    printf("=================================================================\n");
    printf("========================FEATURE INITIALIZER======================\n");
    printf("triangulate 1d:                   %s\n", triangulate_1d ? "true" : "false");
    printf("refine features:                  %s\n", refine_features ? "true" : "false");
    printf("max runs:                         %d\n", max_runs);
    printf("init lamda:                       %6.5f\n", init_lamda);
    printf("max lamda:                        %6.5f\n", max_lamda);
    printf("min dx:                           %6.5f\n", min_dx);
    printf("min dcost:                        %6.5f\n", min_dcost);
    printf("lam mult:                         %6.5f\n", lam_mult);
    printf("min dist:                         %6.5f\n", min_dist);
    printf("max dist:                         %6.5f\n", max_dist);
    printf("max baseline:                     %6.5f\n", max_baseline);
    printf("num_opencv_threads:                  %d\n", num_opencv_threads);
    printf("fast_threshold:                  %d\n", fast_threshold);
    printf("histogram_method:                  %s\n", hist_as_string(histogram_method).c_str());
    printf("knn_ratio:                  %6.5f\n", knn_ratio);
    printf("track_frequency:                  %6.5f\n", track_frequency);
    printf("use baro:                  %s\n", en_baro ? "true" : "false");
    printf("publish stats:                  %s\n", en_ov_stats ? "true" : "false");

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

    cJSON *parent = json_read_file(CONFIG_FILE);
    if (parent == NULL) return -1;

    char string_holder[CHAR_BUF_SIZE];
    memset(string_holder, '\0', CHAR_BUF_SIZE);
    // auto reset features
	json_fetch_bool_with_default(	parent, "en_auto_reset",				&en_auto_reset,					1);
	json_fetch_float_with_default(	parent, "auto_reset_max_velocity",		&auto_reset_max_velocity,		10.0f);
	json_fetch_float_with_default(	parent, "auto_reset_max_v_cov_instant",	&auto_reset_max_v_cov_instant,	0.5f);
	json_fetch_float_with_default(	parent, "auto_reset_max_v_cov",			&auto_reset_max_v_cov,			0.5f);
	json_fetch_float_with_default(	parent, "auto_reset_max_v_cov_timeout_s",&auto_reset_max_v_cov_timeout_s,0.5f);
	json_fetch_int_with_default(	parent, "auto_reset_min_features",		&auto_reset_min_features,		-1);
	json_fetch_float_with_default(	parent, "auto_reset_min_feature_timeout_s",&auto_reset_min_feature_timeout_s,1.0f);

    json_fetch_bool_with_default(parent, "do_fej", (int *)&do_fej, 1);
    json_fetch_bool_with_default(parent, "imu_avg", (int *)&imu_avg, 1);
    json_fetch_bool_with_default(parent, "use_rk4_integration", (int *)&use_rk4_integration, 1);

    json_fetch_bool_with_default(parent, "cam_to_imu_refinement", (int *)&cam_to_imu_refinement, 1);
    json_fetch_bool_with_default(parent, "cam_intrins_refinement", (int *)&cam_intrins_refinement, 1);
    json_fetch_bool_with_default(parent, "cam_imu_ts_refinement", (int *)&cam_imu_ts_refinement, 1);

    json_fetch_int_with_default(parent, "max_clone_size", &max_clone_size, 12);
    json_fetch_int_with_default(parent, "max_slam_features", &max_slam_features, 50);
    json_fetch_int_with_default(parent, "max_slam_in_update", &max_slam_in_update, 64);
    json_fetch_int_with_default(parent, "max_msckf_in_update", &max_msckf_in_update, 64);

    json_fetch_int_with_default(parent, "feat_rep_msckf", (int *)&feat_rep_msckf, 0);
    json_fetch_int_with_default(parent, "feat_rep_slam", (int *)&feat_rep_slam, 5);

    json_fetch_double_with_default(parent, "cam_imu_time_offset", &cam_imu_time_offset, 0.002);
    json_fetch_double_with_default(parent, "slam_delay", &slam_delay, 1.0);

    json_fetch_double_with_default(parent, "gravity_mag", &gravity_mag, 9.80665);
    json_fetch_double_with_default(parent, "init_window_time", &init_window_time, 1.5);
    json_fetch_double_with_default(parent, "init_imu_thresh", &init_imu_thresh, 1.0);

    json_fetch_double_with_default(parent, "imu_sigma_w", &imu_sigma_w, 0.0002);
    json_fetch_double_with_default(parent, "imu_sigma_wb", &imu_sigma_wb, 1.565e-06);
    json_fetch_double_with_default(parent, "imu_sigma_a", &imu_sigma_a, 0.05333);
    json_fetch_double_with_default(parent, "imu_sigma_ab", &imu_sigma_ab, 0.00357402);

    json_fetch_double_with_default(parent, "msckf_chi2_multiplier", &msckf_chi2_multiplier, 5.);
    json_fetch_double_with_default(parent, "msckf_sigma_px", &msckf_sigma_px, 1.);

    json_fetch_double_with_default(parent, "slam_chi2_multiplier", &slam_chi2_multiplier, 5.0);
    json_fetch_double_with_default(parent, "slam_sigma_px", &slam_sigma_px, 1.);

    json_fetch_double_with_default(parent, "zupt_chi2_multiplier", &zupt_chi2_multiplier, 0.0);
    json_fetch_double_with_default(parent, "zupt_sigma_px", &zupt_sigma_px, 1.0);

    json_fetch_bool_with_default(parent, "try_zupt", (int *)&try_zupt, 1);
    json_fetch_double_with_default(parent, "zupt_max_velocity", &zupt_max_velocity, 0.1);
    json_fetch_bool_with_default(parent, "zupt_only_at_beginning", (int *)&zupt_only_at_beginning, 1);
    json_fetch_double_with_default(parent, "zupt_noise_multiplier", &zupt_noise_multiplier, 1.0);
    json_fetch_double_with_default(parent, "zupt_max_disparity", &zupt_max_disparity, 1.0);

    json_fetch_bool_with_default(parent, "triangulate_1d", (int *)&triangulate_1d, 0);
    json_fetch_bool_with_default(parent, "refine_features", (int *)&refine_features, 0);
    json_fetch_int_with_default(parent, "max_runs", &max_runs, 1);
    json_fetch_double_with_default(parent, "init_lamda", &init_lamda, 1e-3);
    json_fetch_double_with_default(parent, "max_lamda", &max_lamda, 1e6);
    json_fetch_double_with_default(parent, "min_dx", &min_dx, 1e-6);
    json_fetch_double_with_default(parent, "min_dcost", &min_dcost, 1e-6);
    json_fetch_double_with_default(parent, "lam_mult", &lam_mult, 10);
    json_fetch_double_with_default(parent, "min_dist", &min_dist, 0.1);
    json_fetch_double_with_default(parent, "max_dist", &max_dist, 16);
    json_fetch_double_with_default(parent, "max_baseline", &max_baseline, 0.08);
    json_fetch_double_with_default(parent, "max_cond_number", &max_cond_number, 50000);

    json_fetch_bool_with_default(parent, "use_mask", (int *)&use_mask, 0);
    json_fetch_bool_with_default(parent, "use_stereo", (int *)&use_stereo, 0);

    json_fetch_int_with_default(parent, "num_opencv_threads", &num_opencv_threads, 6);
    json_fetch_int_with_default(parent, "fast_threshold", &fast_threshold, 20);

    int tmp_hist = 0;
    json_fetch_int_with_default(parent, "histogram_method", &tmp_hist, 10);
    histogram_method = int_to_hist_method(tmp_hist);

    json_fetch_double_with_default(parent, "knn_ratio", &knn_ratio, 0.85);
    json_fetch_double_with_default(parent, "track_frequency", &track_frequency, 12.0);

    json_fetch_bool_with_default(parent, "user_baro", (int *)&en_baro, 1);
    json_fetch_bool_with_default(parent, "use_stats", (int *)&en_ov_stats, 0);


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
