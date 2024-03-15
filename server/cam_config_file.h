/*******************************************************************************
 * Copyright 2022 ModalAI Inc.
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
#ifndef CAM_CONFIG_FILE_H
#define CAM_CONFIG_FILE_H

#include <opencv2/opencv.hpp>
#include "common.h"

#define MAX_CAMERAS 4

#define CAM_CONFIG_FILE "/etc/modalai/voxl-open-vins-standalone.conf"
#define CAM_CONFIG_FILE_HEADER \
    "\
/**\n\
 * This file contains configuration that's specific to voxl-open-vins-server. It is a duplicate of that used by voxl-feature-tracker.\n\
 * \n\
 */\n"

// TODO need a lib to extract global camera data instead of pull from VFT
typedef enum {
    TRACKER_OCV  = 0,    // run the original OpenCV feature tracker
    TRACKER_CVP  = 1,    // run the custom CVP feature tracker
    TRACKER_BOTH = 2     // run both of the above
} tracker_type_t;

typedef struct tracker_input_t {
    char input_pipe[CM_CHAR_BUF_SIZE];
    char output_pipe[CM_CHAR_BUF_SIZE];
    char overlay_pipe[CM_CHAR_BUF_SIZE];
    int num_features;
    tracker_type_t tracker_type;
} tracker_input_t;

extern std::vector<camera_info_set> cam_info_set_vec;
extern char tmp_imu_name[CM_CHAR_BUF_SIZE];
extern int width;
extern int height;
extern bool en_gyro;
extern bool en_descriptors;
extern int num_features_to_track;
extern int grid_x;
extern int grid_y;
extern int min_pix_dist;
extern int pyramid_levels;
extern int window_size;
extern int single_cam_in_use;
extern bool en_ext_feature_tracker;
;

extern bool en_database;
extern int database_size;
extern double max_angular_rate_before_blur;

// cv::Mat for openvins world flip
extern cv::Matx33d tmp_world_correction;
extern cJSON *cam_json;

// read only our own config file without printing the contents
int cam_config_file_read(int is_color_cam, int is_single_cam);

// prints the current configuration values to the screen.
int cam_config_file_print(void);

// load the intrinsics config files per enabled cam
int cam_load_intrinsics_file();
int cam_load_extrinsics_file();
int get_config_as_json();

#endif // CAM_CONFIG_FILE_H
