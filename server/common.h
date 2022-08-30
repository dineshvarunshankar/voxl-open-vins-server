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
#ifndef VFT_COMMON_H
#define VFT_COMMON_H

#include <stdint.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <functional>
#include <Eigen/Core>
#include <modal_pipe.h>


/// magic number for vft_feature_packet 
#define VOXL_FT_MAGIC_NUMBER (0x54555249)
/// magic number for calibration_packets
#define VOXL_CALIB_MAGIC_NUMBER (0x43414C49)
/// magic number for param_packets
#define VOXL_PARAM_MAGIC_NUMBER (0x50415241)

/// arbiitrary str buffer size for names etc
#define CM_CHAR_BUF_SIZE 64


//////////////////////////////////////////////////////////////////////////////
// EXTERNAL PACKETS                                                         //
// All structs defined below are types we are sending out over a pipe       //
//////////////////////////////////////////////////////////////////////////////


/**
 * @struct vft_feature
 * voxl-feature-tracker feature, containing all necessary info to describe a feature point
 * 
 * *NOTE* will likely expand in the future
 * 
 * @field id        unique id for the feature point, should correspond to a match in the previous frame
 * @fielf cam_id    unique id for the camera the feature was seen from
 * @field x         sub-pixel refined x coord of our feature
 * @field y         sub-pixel refined y coord of our feature
 */
typedef struct vft_feature {
    size_t id;
    size_t cam_id;
    float x;
    float y;
} __attribute((packed))__vft_feature;


/**
 * @struct vft_feature_packet
 * voxl-feature-tracker packet, contains an entire "track update"
 * this is essentially our metadata struct
 * 
 * @field magic_number      expected to be VOXL_FT_MAGIC_NUMBER
 * @field timestamp_ns      timestamp of the image extracted from
 * @field num_feats         number of vft_feature packets that will follow this message
 */
typedef struct vft_feature_packet{
    uint32_t magic_number;
    int64_t timestamp_ns;
    uint32_t num_feats;
} __attribute((packed))__vft_feature_packet;


/**
 * @struct vft_calib_packet
 * packet to send back to open vins only (for now) containing required setup info about our system
 * 
 * @field magic_number              expected to be VOXL_CALIB_MAGIC_NUMBER
 * @field timestamp_ns              timestamp of packet write
 * @field cam_id                    camera id this packet corresponds to 
 * @field num_cams                  number of cameras in the overall system
 * @field cam_wrt_imu               quaternion followed by tranlation of camera into the imu frame
 * @field cam_calib_intrinsic       fx, fy, px, py, distortion coefficients (4), w, h
 * @field is_fisheye                    fisheye flag, parsed from calib
 */
typedef struct vft_calib_packet {
    uint32_t magic_number;
    int64_t timestamp_ns;
    size_t cam_id;
    size_t num_cams;
    Eigen::Matrix<double, 7, 1> cam_wrt_imu;
    Eigen::Matrix<double, 10, 1> cam_calib_intrinsic;
    bool is_fisheye;
} vft_calib_packet;


/**
 * @struct vft_param_packet
 * extra parameter packet for any non-standard params that need to be sent to another server
 * as of now, only sent to open-vins to get the last params we setup here
 * 
 * @field magic_number              expected to be VOXL_PARAM_MAGIC_NUMBER
 * @field imu_name                  name of the imu we are subscribing to (optional param for voxl-feature-tracker)
 * @field num_features_to_track     number of features we are trying to track frame by frame
 */
typedef struct vft_param_packet {
    uint32_t magic_number;
    char imu_name[CM_CHAR_BUF_SIZE];
    int num_features_to_track;
} vft_param_packet;


/**
 * @brief timing helper function
 * used across mai projects
 * 
 * @return int64_t monotonic time in nanoseconds 
 */
static int64_t _apps_time_monotonic_ns() {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts)) {
        fprintf(stderr, "ERROR calling clock_gettime\n");
        return -1;
    }
    return (int64_t)ts.tv_sec * 1000000000 + (int64_t)ts.tv_nsec;
}


#endif // VFT_COMMON_H
