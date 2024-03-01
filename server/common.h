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


/// magic number for calibration_packets
#define VOXL_CALIB_MAGIC_NUMBER (0x43414C49)
/// magic number for param_packets
#define VOXL_PARAM_MAGIC_NUMBER (0x50415241)

/// arbiitrary str buffer size for names etc
#define CM_CHAR_BUF_SIZE 64

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI/180.0)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0/M_PI)
#endif



typedef enum camera_mode {
    UNKNOWN = -1,
    MONO = 0,
    STEREO = 1,
    STEREO_LEFT_ONLY = 2,
    STEREO_RIGHT_ONLY = 3
} camera_mode;


static std::string camera_mode_as_string(camera_mode cm){
    if (cm == MONO){
        return "MONO";
    }
    else if (cm == STEREO){
        return "STEREO";
    }
    else if (cm == STEREO_LEFT_ONLY){
        return "STEREO_LEFT_ONLY";
    }
    else if (cm == STEREO_RIGHT_ONLY){
        return "STEREO_RIGHT_ONLY";
    }
    else return "UNKNOWN";
}

static camera_mode string_camera_mode_to_enum(const char* str_cm){
    if (!strncmp(str_cm, "MONO", sizeof("MONO"))){
        return MONO;
    }
    if (!strncmp(str_cm, "STEREO", sizeof("STEREO"))){
        return STEREO;
    }
    if (!strncmp(str_cm, "STEREO_LEFT_ONLY", sizeof("STEREO_LEFT_ONLY"))){
        return STEREO_LEFT_ONLY;
    }
    if (!strncmp(str_cm, "STEREO_RIGHT_ONLY", sizeof("STEREO_RIGHT_ONLY"))){
        return STEREO_RIGHT_ONLY;
    }
    else return UNKNOWN;
}

/**
 * @struct image_data
 * base packet that is fed to all of our trackers
 *
 * @field timestamp_ns      timestamp of image
 * @field tracker_ids       vec of ids per camera, matching order of images + masks
 * @field images            vec of images to track across, in order matching ids vec
 * @field masks             vec of masks to denote regions of non-interest, in order matching ids vec
 *                          mask regions with val == 255 will be ignored in tracking process
 */
typedef struct image_data {
    int64_t timestamp_ns;
    std::vector<size_t> tracker_ids;
    std::vector<cv::Mat> images;
    std::vector<cv::Mat> masks;
} image_data;


/**
 * @struct camera_info
 * internal struct used to parse out intrinsics data of cameras and store useful info
 *
 * @field name          name of the camera pipe
 * @field cam_mode      enum denoting the mode of this camera, to assist with setting up cam properties
 * @field is_fisheye    fisheye flag, parsed from calib
 * @field cam_mat       intrinsic calibrated camera matrix
 * @field dist_coeffs   intrinsic calibrated camera distortion coefficients
 * @field width         image width
 * @field height        image height
 */
typedef struct camera_info_set {
    char name[CM_CHAR_BUF_SIZE];
    camera_mode cam_mode;
    bool is_fisheye;
    std::vector<cv::Matx33d> cam_mat;
    std::vector<cv::Vec4d> dist_coeffs;
    std::vector<Eigen::Matrix<double, 7, 1>> cam_wrt_imu;
    std::vector<Eigen::Matrix<double, 10, 1>> cam_calib_intrinsic;
    std::vector<cv::Mat> cam_wrt_imu_rot;
    char extrinsics_extension_first[CM_CHAR_BUF_SIZE] = {0};
    char extrinsics_extension_second[CM_CHAR_BUF_SIZE] = {0};
    char extrinsics_extension_third[CM_CHAR_BUF_SIZE] = {0};
    char intrinsics_extension_first[CM_CHAR_BUF_SIZE] = {0};
    char intrinsics_extension_second[CM_CHAR_BUF_SIZE] = {0};
    char intrinsics_extension_third[CM_CHAR_BUF_SIZE] = {0};
    size_t cam_id;
} camera_info_set;


typedef struct camera_info {
    char name[128];
    camera_mode mode;
    Eigen::Matrix<double, 7, 1> cam_wrt_imu;
    Eigen::Matrix<double, 10, 1> cam_calib_intrinsic;
    bool is_fisheye;
    size_t cam_id;
} camera_info;



//////////////////////////////////////////////////////////////////////////////
// EXTERNAL PACKETS                                                         //
// All structs defined below are types we are sending out over a pipe       //
//////////////////////////////////////////////////////////////////////////////



/// magic number for vft_feature_packet
#define VOXL_FT_MAGIC_NUMBER (0x54555249)


/**
 * @struct vft_feature
 * voxl-feature-tracker feature, containing all necessary info to describe a feature point
 *
 * @field id             unique id for the feature point, should correspond to a match in the previous frame
 * @fielf cam_id         unique id for the camera the feature was seen from
 * @field x              sub-pixel refined x coord of our feature
 * @field y              sub-pixel refined y coord of our feature
 * @field x_prev         sub-pixel refined previous x coord of our feature
 * @field y_prev         sub-pixel refined previous y coord of our feature
 * @field descriptor     descriptor of the feature
 * @field age            how many frames the feature has been tracked for
 * @field score          custom metric to "score" the feature
 * @field pyr_lvl_mask   mask for which pyramid levels the feature is present on
 * @field cam_id         id of the camera
 * @field reserved_1     reserved
 * @field reserved_1     reserved
 */
typedef struct vft_feature {
    int64_t id;
    float x;
    float y;
    float x_prev;
    float y_prev;
    unsigned char descriptor[32] = {0};
    int32_t age;
    int8_t score;
    int8_t pyr_lvl_mask;
    int8_t cam_id;
    int8_t reserved_1;
    uint32_t reserved_2;
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
    int32_t num_feats[4];
    int32_t frame_ids[4];
    uint8_t reset;
    uint8_t n_cams;
    uint8_t reserved_1;
    uint8_t reserved_2;
} __attribute((packed))__vft_feature_packet;





////////////////////////////////////////////////////////////////
#ifdef DEPRECATED_SINCE_SDK_1_1_0
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

#endif
////////////////////////////////////////////////////////////////

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



// Converts a given Rotation Matrix to Euler angles
// Convention used is Y-Z-X Tait-Bryan angles
// Reference code implementation:
// https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToEuler/index.htm
static cv::Mat rot2euler(const cv::Mat & rotationMatrix)
{
    cv::Mat euler(3,1,CV_64F);

    double roll  = atan2(rotationMatrix.at<double>(2,1), rotationMatrix.at<double>(2,2));
    double pitch = asin(-rotationMatrix.at<double>(2,0));
    double yaw   = atan2(rotationMatrix.at<double>(1,0), rotationMatrix.at<double>(0,0));

    if(fabs(pitch - M_PI_2) < 0.001){
        roll = 0.0;
        pitch = atan2(rotationMatrix.at<double>(1,2), rotationMatrix.at<double>(0,2));
    }
    else if(fabs(pitch + M_PI_2) < 0.001) {
        roll = 0.0;
        pitch = atan2(-rotationMatrix.at<double>(1,2), -rotationMatrix.at<double>(0,2));
    }

    euler.at<double>(0) = roll;         // roll
    euler.at<double>(1) = pitch;     // pitch
    euler.at<double>(2) = yaw;      // yaw

    return euler;
}

static cv::Mat euler2rot(double heading, double attitude, double bank)
{
    cv::Mat m = cv::Mat(3,3,CV_64F);

    // Assuming the angles are in radians.
    double ch = cos(heading);
    double sh = sin(heading);
    double ca = cos(attitude);
    double sa = sin(attitude);
    double cb = cos(bank);
    double sb = sin(bank);

    m.at<double>(0,0)  = ch * ca;
    m.at<double>(0,1)  = sh*sb - ch*sa*cb;
    m.at<double>(0,2)  = ch*sa*sb + sh*cb;
    m.at<double>(1,0)  = sa;
    m.at<double>(1,1)  = ca*cb;
    m.at<double>(1,2)  = -ca*sb;
    m.at<double>(2,0)  = -sh*ca;
    m.at<double>(2,1)  = sh*sa*cb + ch*sb;
    m.at<double>(2,2)  = -sh*sa*sb + ch*cb;

    return m;
}


#endif // VFT_COMMON_H
