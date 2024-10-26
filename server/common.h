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
#include <unistd.h>


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
