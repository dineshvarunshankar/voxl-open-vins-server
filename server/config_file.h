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

// 3rd Party Library Includes
#include <Eigen/Eigen>

// Modal AI Libraries
#include <modal_json.h>

// C/C++ Includes
#include <vector>


#define CHAR_BUF_SIZE 128
#define CONFIG_FILE "/etc/modalai/voxl-open-vins-server.conf"
#define CONFIG_FILE_HEADER \
    "\
/**\n\
 * This file contains configuration that's specific to voxl-open-vins-server.\n\
 */\n"
#define MAX_CAMERAS 4

typedef struct camera_info {
    bool enable;
    char name[CHAR_BUF_SIZE];
    Eigen::Matrix<double, 7, 1> cam_wrt_imu;
    Eigen::Matrix<double, 10, 1> cam_calib_intrinsic;
    bool is_fisheye;   
} camera_info;

extern std::vector<camera_info> cam_info_vec;
extern char imu_name[CHAR_BUF_SIZE];

// online calibration? think that just means it adjusts and tries to converge on these vals?
extern bool camera_to_imu_pose_calibration;
extern bool camera_intrinsics_calibration;
extern bool camera_imu_timestamp_calibration;

extern double delay_after_init;
extern bool downsample_cams;
extern int num_features_to_track;
extern int max_clone_size;

extern bool use_zupt;
extern double zupt_max_velocity;
extern bool zupt_only_at_beginning;
extern double zupt_noise_multiplier;
extern double zupt_max_disparity;
extern double init_imu_thresh;

// read only our own config file without printing the contents
int config_file_read(void);

// prints the current configuration values to the screen.
int config_file_print(void);

// load the common extrinsics config files
int load_extrinsics_file();

// load the common intrinsics config files
int load_intrinsics_file();


#endif // end CONFIG_FILE_H
