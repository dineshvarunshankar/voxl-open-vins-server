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

#include <modal_json.h>
#include <stdio.h>
#include <voxl_common_config.h>

#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stereo.hpp>
#include <string>

#include <iostream>
#include <Eigen/Geometry>

#include "cam_config_file.h"

#include "rc_transform.h"

#define EXTRINS_BODY "body"

void extrinsicsNEDtoFLU(Eigen::Matrix<double,3,3>  &R, Eigen::Matrix<double, 4, 1> &quaternion);
void quat_2_rot(Eigen::Matrix<double, 4, 1> quaternion, Eigen::Matrix<double,3,3>  &R);

//DEPRECATED
int get_RPY_from_NED(Eigen::Matrix<double,3,3>  R, double* roll, double* pitch, double* yaw);
int get_RPY_from_NED_to_FLU(double roll, double pitch, double yaw, Eigen::Matrix<double,3,3>  &R);

std::vector<camera_info_set> cam_info_set_vec;

char tmp_imu_name[CM_CHAR_BUF_SIZE];
int num_features_to_track;
int grid_x;
int grid_y;
int min_pix_dist;
int pyramid_levels;
int window_size;
int width;
int height;
bool en_gyro;
bool en_descriptors;

bool en_database;
int database_size;

double max_angular_rate_before_blur;

cv::Matx33d tmp_world_correction = cv::Matx33d::eye();
cJSON *cam_json = NULL;

void extrinsicsNEDtoFLU(Eigen::Matrix<double,3,3>  &R, Eigen::Matrix<double, 4, 1> &quaternion) 
{
	static int camera_ctn = 0;
	double r,p,y;

	// Get the Euler angles in XYZ, RPY order.
	Eigen::Vector3d eulerAngles = R.eulerAngles(0, 1, 2);

	// TODO Fix anything needed for FLIPPED VOXL2 board, aka when IMU is upside down
	
	// Return the Euler angles.
	  r = eulerAngles(0);
	  p = M_PI - eulerAngles(1);
	  y = eulerAngles(2) - M_PI;
	  
	printf("[INFO] Camera: %d -- converted extrinsics *in FLU* are: Roll: %f, Pitch %f, Yaw %f\n", 
			camera_ctn++,
			r/M_PI*180,
			p/M_PI*180,
			y/M_PI*180);

	double c1 = cos( r / 2 );
	double c2 = cos( p / 2 );
	double c3 = cos( y / 2 );

	double s1 = sin( r / 2 );
	double s2 = sin( p / 2 );
	double s3 = sin( y / 2 );

	quaternion(0) = s1 * c2 * c3 + c1 * s2 * s3;
	quaternion(1) = c1 * s2 * c3 - s1 * c2 * s3;
	quaternion(2)  = c1 * c2 * s3 + s1 * s2 * c3;
	quaternion(3)  = c1 * c2 * c3 - s1 * s2 * s3;

	quat_2_rot(quaternion, R);
  
 }

void quat_2_rot(Eigen::Matrix<double, 4, 1> quaternion, Eigen::Matrix<double,3,3>  &R)
{
	double x = quaternion(0);
	double y = quaternion(1);
	double z = quaternion(2);
	double w = quaternion(3);
	double x2 = x + x;
	double y2 = y + y;
	double z2 = z + z;
	double xx = x * x2;
	double xy = x * y2;
	double xz = x * z2;
	double yy = y * y2;
	double yz = y * z2;
	double zz = z * z2;
	double wx = w * x2;
	double wy = w * y2;
	double wz = w * z2;

	R(0,0) = ( 1 - ( yy + zz ) );
	R(1,0) = ( xy + wz );
	R(2,0) = ( xz - wy );

	R(0,1) = ( xy - wz );
	R(1,1) = ( 1 - ( xx + zz ) );
	R(2,1) = ( yz + wx );

	R(0,2) = ( xz + wy );
	R(1,2) = ( yz - wx );
	R(2,2) = ( 1 - ( xx + yy ) );

}

//DEPRECATED
int get_RPY_from_NED(Eigen::Matrix<double,3,3>  R, double* roll, double* pitch, double* yaw)
{
    *roll  = atan2(R(2,1), R(2,2));
    *pitch = asin(-R(2,0));
    *yaw   = atan2(R(1,0), R(0,0));

    if(fabs(*pitch - M_PI_2) < 0.001){
        *roll = 0.0;
        *pitch = atan2(R(1,2), R(0,2));
        printf("flip\n");
    }
    else if(fabs(*pitch + M_PI_2) < 0.001) {
        *roll = 0.0;
        *pitch = atan2(-R(1,2), -R(0,2));
        printf("flip2\n");
    }
    return 0;
}

//DEPRECATED
int get_RPY_from_NED_to_FLU(double roll, double pitch, double yaw, Eigen::Matrix<double,3,3>  &R)
{
    double c1 = cos(yaw);
    double s1 = sin(yaw);
    double c2 = cos(pitch);
    double s2 = sin(pitch);
    double c3 = cos(roll);
    double s3 = sin(roll);

    R(0,0) = c1*c2;
    R(0,1) = (c1*s2*s3)-(c3*s1);
    R(0,2) = (s1*s3)+(c1*c3*s2);

    R(1,0) = c2*s1;
    R(1,1) = (c1*c3)+(s1*s2*s3);
    R(1,2) = (c3*s1*s2)-(c1*s3);

    R(2,0) = -s2;
    R(2,1) = c2*s3;
    R(2,2) = c2*c3;

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (fabs(R(i, j)) < 10e-6)
				R(i, j) = 0.;
		}
	}

    return 0;
}

#ifdef NOT_USED
// OV uses JPL  not Hamilton conventions
static Eigen::Matrix<double, 4, 1> rot_2_quat_1(
		const Eigen::Matrix<double, 3, 3> &R)
{
	if (R.determinant() != 1.0) {
	  throw std::runtime_error("The rotation matrix is not a pure rotation.");
	}
	  // Extract the quaternion from the rotation matrix.
	  Eigen::Quaterniond q(R);

	  Eigen::Matrix<double, 4, 1> rtn_q;
	  rtn_q(0) = q.x();
	  rtn_q(1) = q.y();
	  rtn_q(2) = q.z();
	  rtn_q(3) = q.w();

	  return rtn_q;
}
#endif

/**
 * Parallels OpenVINS core routine
 *
 * @brief Returns a JPL quaternion from a rotation matrix
 *  This is different from Eigen rot to quant as it unwinds the coordinates in ZYX as we
 *  convert from FLU to NED IMU space (not Hamilton?)
 *
 */
static Eigen::Matrix<double, 4, 1> rot_2_quat(
		const Eigen::Matrix<double, 3, 3> &rot)
{
	Eigen::Matrix<double, 4, 1> q;
	double T = rot.trace();
	if ((rot(0, 0) >= T) && (rot(0, 0) >= rot(1, 1))
			&& (rot(0, 0) >= rot(2, 2)))
	{
		q(0) = sqrt((1 + (2 * rot(0, 0)) - T) / 4);
		q(1) = (1 / (4 * q(0))) * (rot(0, 1) + rot(1, 0));
		q(2) = (1 / (4 * q(0))) * (rot(0, 2) + rot(2, 0));
		q(3) = (1 / (4 * q(0))) * (rot(1, 2) - rot(2, 1));

	}
	else if ((rot(1, 1) >= T) && (rot(1, 1) >= rot(0, 0))
			&& (rot(1, 1) >= rot(2, 2)))
	{
		q(1) = sqrt((1 + (2 * rot(1, 1)) - T) / 4);
		q(0) = (1 / (4 * q(1))) * (rot(0, 1) + rot(1, 0));
		q(2) = (1 / (4 * q(1))) * (rot(1, 2) + rot(2, 1));
		q(3) = (1 / (4 * q(1))) * (rot(2, 0) - rot(0, 2));
	}
	else if ((rot(2, 2) >= T) && (rot(2, 2) >= rot(0, 0))
			&& (rot(2, 2) >= rot(1, 1)))
	{
		q(2) = sqrt((1 + (2 * rot(2, 2)) - T) / 4);
		q(0) = (1 / (4 * q(2))) * (rot(0, 2) + rot(2, 0));
		q(1) = (1 / (4 * q(2))) * (rot(1, 2) + rot(2, 1));
		q(3) = (1 / (4 * q(2))) * (rot(0, 1) - rot(1, 0));
	}
	else
	{
		q(3) = sqrt((1 + T) / 4);
		q(0) = (1 / (4 * q(3))) * (rot(1, 2) - rot(2, 1));
		q(1) = (1 / (4 * q(3))) * (rot(2, 0) - rot(0, 2));
		q(2) = (1 / (4 * q(3))) * (rot(0, 1) - rot(1, 0));
	}

	if (q(3) < 0)
	{
		q = -q;
	}
	// normalize and return
	q = q / (q.norm());
	return q;
}

static void create_ov_extrinsics(rc_tf_t transform,
		Eigen::Matrix<double, 7, 1> &cam_wrt_imu, cv::Mat &cam_wrt_imu_rot)
{
	Eigen::Matrix<double, 3, 3> rotation_ch_wrt_par;
	Eigen::Matrix<double, 3, 3> open_vins_correction;

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			rotation_ch_wrt_par(i, j) = transform.d[i][j];
			open_vins_correction(i, j) = tmp_world_correction(i, j);
		}
	}

	// this matrix needs to be unaffected by the open vins world flip
	cam_wrt_imu_rot = cv::Mat(3, 3, CV_64F);

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			cam_wrt_imu_rot.at<double>(i, j) = rotation_ch_wrt_par(i, j);
		}
	}

	// try inverting it first
	Eigen::Matrix<double, 3, 3> rotation_par_wrt_ch =
			rotation_ch_wrt_par.transpose();
	// now, correct matrices for open vins, left multiply by transpose to get Rcam->imucorrected
	rotation_par_wrt_ch = rotation_par_wrt_ch
			* open_vins_correction.transpose();

	// create our quaternion
	Eigen::Matrix<double, 4, 1> quaternion;

	// jpl creator from rotation matrix
	quaternion = rot_2_quat(rotation_par_wrt_ch);

	// create our translation vec, needs to also be inverted
	Eigen::Matrix<double, 3, 1> translation;
	translation[0] = transform.d[0][3];
	translation[1] = transform.d[1][3];
	translation[2] = transform.d[2][3];

	translation = rotation_par_wrt_ch * translation;
	// finally, invert it and we're ready to go
	translation = -translation;

	cam_wrt_imu.block(0, 0, 4, 1) = quaternion;
	cam_wrt_imu.block(4, 0, 3, 1) = translation;
	std::cout << "CAM ROTATION MATRIX:" << std::endl << rotation_ch_wrt_par
			<< std::endl;
	std::cout << "CAM TRANSLATION:" << std::endl << translation << std::endl;
	return;
}

static int axis_angle_to_rotation(double axis[3], double axis_norm,
		double angle, cv::Matx33d &rot)
{
	double s = sin(angle);
	double c = cos(angle);
	double omcos = 1.0 - c; // "one minus cos"

	if (fabs(axis_norm) < 0.00001)
	{
		fprintf(stderr,
				"ERROR in rc_axis_angle_to_rotation_matrix, axis vector must have nonzero length\n");
		return -1;
	}

	double x = axis[0] / axis_norm;
	double y = axis[1] / axis_norm;
	double z = axis[2] / axis_norm;

	rot(0, 0) = c + (x * x * omcos);
	rot(0, 1) = (x * y * omcos) - (z * s);
	rot(0, 2) = (x * z * omcos) + (y * s);

	rot(1, 0) = (x * y * omcos) + (z * s);
	rot(1, 1) = c + (y * y * omcos);
	rot(1, 2) = (y * z * omcos) - (x * s);

	rot(2, 0) = (x * z * omcos) - (y * s);
	rot(2, 1) = (y * z * omcos) + (x * s);
	rot(2, 2) = c + (z * z * omcos);

	return 0;
}

static int create_open_vins_world_rotation_from_file(cv::Matx33d *R_correction)
{
	vcc_extrinsic_t t[VCC_MAX_EXTRINSICS_IN_CONFIG];
	vcc_extrinsic_t extrins_holder;

	// now load in extrinsics
	int n_extrinsics;
	if (vcc_read_extrinsic_conf_file(VCC_EXTRINSICS_PATH, t, &n_extrinsics,
			VCC_MAX_EXTRINSICS_IN_CONFIG))
	{
		fprintf(stderr, "ERROR: %s Unable to read extrinsics conf at %s\n",
				__FUNCTION__, VCC_EXTRINSICS_PATH);
		return -1;
	}

	if (vcc_find_extrinsic_in_array(EXTRINS_BODY, tmp_imu_name, t, n_extrinsics,
			&extrins_holder))
		return -1;

	for (int j = 0; j < 3; j++)
	{
		for (int k = 0; k < 3; k++)
		{
			double val = extrins_holder.R_child_to_parent[j][k];
			if (fabs(val) < 10e-6)
				(*R_correction)(j, k) = 0.0;
			else
				(*R_correction)(j, k) = val;
		}
	}

	printf("World Rotation Matrix: \n");
	for (int j = 0; j < 3; j++)
	{
		for (int k = 0; k < 3; k++)
		{
			double val = extrins_holder.R_child_to_parent[j][k];
			if (fabs(val) < 10e-6)
				printf(" 0 ");
			else
				printf(" %f ", val);

		}
		printf("\n");
	}
	printf("\n");
	return 0;
}



//int _rotation_matrix_to_tait_bryan_xyz_intrinsic_eigen(Eigen::Matrix<double,3,3>  R, double* roll, double* pitch, double* yaw)
//{
//	*pitch = asin(R(0,2))*RAD_TO_DEG;
//	if(fabs(R(0,2))<0.9999999){
//		*roll = atan2(-R(1,2), R(2,2))*RAD_TO_DEG;
//		*yaw  = atan2(-R(0,1), R(0,0))*RAD_TO_DEG;
//	}
//	else{
//		*roll = atan2(-R(2,1), R(1,1))*RAD_TO_DEG;
//		*yaw  = 0.0;
//	}
//	return 0;
//}


int cam_load_extrinsics_file()
{
	printf("cam_load_extrinsics_file\n");
	vcc_extrinsic_t t[VCC_MAX_EXTRINSICS_IN_CONFIG];
	vcc_extrinsic_t extrins_holder;
	char ext_name[CM_CHAR_BUF_SIZE * 2];

	// open vins requires the imu to cam relation, and then internally constructs the cam to imu relation that is used
	create_open_vins_world_rotation_from_file(&tmp_world_correction);

	// now load in extrinsics
	int n_extrinsics;
	if (vcc_read_extrinsic_conf_file(VCC_EXTRINSICS_PATH, t, &n_extrinsics,
			VCC_MAX_EXTRINSICS_IN_CONFIG))
	{
		fprintf(stderr, "ERROR: %s Unable to read extrinsics conf at %s\n",
				__FUNCTION__, VCC_EXTRINSICS_PATH);
		return -1;
	}

	printf("load extrinsics %d\n", (int)cam_info_set_vec.size());

    for (size_t i = 0; i < cam_info_set_vec.size(); i++)
	{
		if (strstr(camera_mode_as_string(cam_info_set_vec[i].cam_mode).c_str(), "STEREO") != NULL)
		{
            int j, k;

	        //CAM 1
	        memset(ext_name, '\0', CM_CHAR_BUF_SIZE);
	        strcpy(ext_name, cam_info_set_vec[i].name);
	        strcat(ext_name, cam_info_set_vec[i].extrinsics_extension_first);
            if (vcc_find_extrinsic_in_array(tmp_imu_name, ext_name, t, n_extrinsics, &extrins_holder)){
                fprintf(stderr, "ERROR: Unable to find %s to %s in extrinsics conf\n", tmp_imu_name, ext_name);
            }
			Eigen::Matrix<double, 3, 3> rotation_temp_1;
			for (j = 0; j < 3; j++)
			{
				for (k = 0; k < 3; k++)
				{
					rotation_temp_1(j, k) = extrins_holder.R_child_to_parent[j][k];
					if (fabs(rotation_temp_1(j, k)) < 10e-6)
						rotation_temp_1(j, k) = 0.;
				}
			}

			std::cout << "\nprocessing NED extrinsics: " << std::endl;
			std::cout << rotation_temp_1 << std::endl;

			// convert to quat
			Eigen::Matrix<double, 4, 1> quaternion_1;
			Eigen::Matrix<double, 3, 1> translation_1;

			// convert extrinsics from NED  to FLU
			extrinsicsNEDtoFLU(rotation_temp_1, quaternion_1);

			translation_1[0] = extrins_holder.T_child_wrt_parent[0];
			translation_1[1] = -1 * extrins_holder.T_child_wrt_parent[1];
			translation_1[2] = -1 * extrins_holder.T_child_wrt_parent[2];

			cv::Mat cam_wrt_imu_rot_1 = cv::Mat(3, 3, CV_64F);

			for (int z = 0; z < 3; z++)
			{
				for (int j = 0; j < 3; j++)
				{
					if (fabs(rotation_temp_1(z, j)) < 10e-6)
						cam_wrt_imu_rot_1.at<double>(z, j) = 0.;
					else
						cam_wrt_imu_rot_1.at<double>(z, j) = rotation_temp_1(z, j);
				}
			}

			cam_info_set_vec[i].cam_wrt_imu_rot.push_back(cam_wrt_imu_rot_1);

			Eigen::Matrix<double, 7, 1> cam_wrt_imu_1;

			cam_wrt_imu_1.block(0, 0, 4, 1) = quaternion_1;
			cam_wrt_imu_1.block(4, 0, 3, 1) = translation_1;

			cam_info_set_vec[i].cam_wrt_imu.push_back(cam_wrt_imu_1);

	        //CAM 2
            memset(ext_name, '\0', CM_CHAR_BUF_SIZE);
            strcpy(ext_name, cam_info_set_vec[i].name);
            strcat(ext_name, cam_info_set_vec[i].extrinsics_extension_second);
            if (vcc_find_extrinsic_in_array(tmp_imu_name, ext_name, t, n_extrinsics, &extrins_holder)){
                fprintf(stderr, "ERROR: Unable to find %s to %s in extrinsics conf\n", tmp_imu_name, ext_name);
            }
			Eigen::Matrix<double, 3, 3> rotation_temp_2;
			for (j = 0; j < 3; j++)
			{
				for (k = 0; k < 3; k++)
				{
					rotation_temp_2(j, k) = extrins_holder.R_child_to_parent[j][k];
					if (fabs(rotation_temp_2(j, k)) < 10e-6)
						rotation_temp_2(j, k) = 0.;
				}
			}
			std::cout << "processing NED extrinsics: " << std::endl;
			std::cout << rotation_temp_2 << std::endl;

			// convert to quat
			Eigen::Matrix<double, 4, 1> quaternion_2;
			Eigen::Matrix<double, 3, 1> translation_2;
			
			// convert extrinsics from NED  to FLU
			extrinsicsNEDtoFLU(rotation_temp_2, quaternion_2);

			translation_2[0] = extrins_holder.T_child_wrt_parent[0];
			translation_2[1] = -1 * extrins_holder.T_child_wrt_parent[1];
			translation_2[2] = -1 * extrins_holder.T_child_wrt_parent[2];

			cv::Mat cam_wrt_imu_rot_2 = cv::Mat(3, 3, CV_64F);

			for (int z = 0; z < 3; z++)
			{
				for (int j = 0; j < 3; j++)
				{
					if (fabs(rotation_temp_2(z, j)) < 10e-6)
						cam_wrt_imu_rot_2.at<double>(z, j) = 0.;
					else
						cam_wrt_imu_rot_2.at<double>(z, j) = rotation_temp_2(z, j);
				}
			}

			cam_info_set_vec[i].cam_wrt_imu_rot.push_back(cam_wrt_imu_rot_2);

			Eigen::Matrix<double, 7, 1> cam_wrt_imu_2;

			cam_wrt_imu_2.block(0, 0, 4, 1) = quaternion_2;
			cam_wrt_imu_2.block(4, 0, 3, 1) = translation_2;

			cam_info_set_vec[i].cam_wrt_imu.push_back(cam_wrt_imu_2);
		}
		else
		{

			if (vcc_find_extrinsic_in_array(cam_info_set_vec[i].name, tmp_imu_name,
					t, n_extrinsics, &extrins_holder))
			{
				fprintf(stderr, "vcc_find_extrinsic_in_array failed for %s %s\n", cam_info_set_vec[i].name, tmp_imu_name);
				return -1;
			}

			// single cam
			rc_tf_t imu_to_cam;
			int j, k;

			// grab the imu -> camera extrinsics relation
			if (vcc_find_extrinsic_in_array(tmp_imu_name, cam_info_set_vec[i].name,
					t, n_extrinsics, &extrins_holder))
			{
				fprintf(stderr,
						"ERROR: Unable to find %s to %s in extrinsics conf\n",
						tmp_imu_name, ext_name);
			}

			Eigen::Matrix<double, 3, 3> rotation_temp;
			for (j = 0; j < 3; j++)
			{
				for (k = 0; k < 3; k++)
				{
					rotation_temp(j, k) = extrins_holder.R_child_to_parent[j][k];
					if (fabs(rotation_temp(j, k)) < 10e-6)
							rotation_temp(j, k) = 0.;
				}
			}

			// convert to quat
			Eigen::Matrix<double, 4, 1> quaternion;
			Eigen::Matrix<double, 3, 1> translation;

			// convert extrinsics from NED  to FLU
			extrinsicsNEDtoFLU(rotation_temp, quaternion);

			// WARNING: FLU coordindate system needs YZ flipped
			translation[0] = extrins_holder.T_child_wrt_parent[0];
			translation[1] = -1 * extrins_holder.T_child_wrt_parent[1];
			translation[2] = -1 * extrins_holder.T_child_wrt_parent[2];

			cv::Mat cam_wrt_imu_rot = cv::Mat(3, 3, CV_64F);

			for (int z = 0; z < 3; z++)
			{
				for (int j = 0; j < 3; j++)
				{
					if (fabs(rotation_temp(z, j)) < 10e-6)
						cam_wrt_imu_rot.at<double>(z, j) = 0.;
					else
						cam_wrt_imu_rot.at<double>(z, j) = rotation_temp(z, j);
				}
			}

			cam_info_set_vec[i].cam_wrt_imu_rot.push_back(cam_wrt_imu_rot);

			Eigen::Matrix<double, 7, 1> cam_wrt_imu;

			cam_wrt_imu.block(0, 0, 4, 1) = quaternion;
			cam_wrt_imu.block(4, 0, 3, 1) = translation;

			cam_info_set_vec[i].cam_wrt_imu.push_back(cam_wrt_imu);

			std::cout << "(local) CAMERA ROTATION MATRIX:" << std::endl
					<< cam_info_set_vec[i].cam_wrt_imu_rot[0] << std::endl;
			std::cout << "(local) CAMERA TRANSLATION:" << std::endl << translation
					<< std::endl;
		}
	}

	return 0;
}

int cam_load_intrinsics_file()
{
	char intrinsics_path[CM_CHAR_BUF_SIZE * 2];

	// this is here again, since opencv will define different matrices per stereo / mono
	// and because we need to pack the intrinsic vals up in two packets for ov if stereo
	bool stereo_setup = false;

	for (int i = 0; i < (int) cam_info_set_vec.size(); i++)
	{
		memset(intrinsics_path, '\0', CM_CHAR_BUF_SIZE);

		strcpy(intrinsics_path, "/data/modalai/opencv_");
		strcat(intrinsics_path, cam_info_set_vec[i].name);
		strcat(intrinsics_path, "_intrinsics.yml");

		printf("Loading OpenCV intrinsics cal file from: %s\n",
				intrinsics_path);

		// if it contains stereo, we gotta look for different matrix names
		if (strstr(camera_mode_as_string(cam_info_set_vec[i].cam_mode).c_str(), "STEREO") != NULL)
		{
			// here we go
			printf("Found STEREO config\n");
			stereo_setup = true;
		}

		cv::FileStorage fs(intrinsics_path, cv::FileStorage::READ);
		if (!fs.isOpened())
		{
			fprintf(stderr, "Failed to load intrinsics file %s\n",
					intrinsics_path);
			return -1;
		}

		cv::FileNode n;
		cv::Mat camMatrix;
		cv::Mat distCoeffs;
		cv::Mat camMatrix2;
		cv::Mat distCoeffs2;
		int is_fisheye = 0;
		int w, h;
		int has_m1 = 0;
		int has_d1 = 0;
		int has_m2 = 0;
		int has_d2 = 0;
		int has_w = 0;
		int has_h = 0;

		if (stereo_setup)
		{
			// stereo left
			n = fs["M1"];
			if (n.type() != cv::FileNode::NONE)
			{
				n >> camMatrix;
				has_m1 = 1;
			}
			n = fs["D1"];
			if (n.type() != cv::FileNode::NONE)
			{
				n >> distCoeffs;
				has_d1 = 1;
			}

			// stereo right
			n = fs["M2"];
			if (n.type() != cv::FileNode::NONE)
			{
				n >> camMatrix2;
				has_m2 = 1;
			}
			n = fs["D2"];
			if (n.type() != cv::FileNode::NONE)
			{
				n >> distCoeffs2;
				has_d2 = 1;
			}
		}
		else
		{
			// mono camera
			n = fs["M"];
			if (n.type() != cv::FileNode::NONE)
			{
				n >> camMatrix;
				has_m1 = 1;
			}
			n = fs["D"];
			if (n.type() != cv::FileNode::NONE)
			{
				n >> distCoeffs;
				has_d1 = 1;
			}

			// stereo left
			n = fs["M1"];
			if (n.type() != cv::FileNode::NONE)
			{
				n >> camMatrix;
				has_m1 = 1;
			}
			n = fs["D1"];
			if (n.type() != cv::FileNode::NONE)
			{
				n >> distCoeffs;
				has_d1 = 1;
			}

			// aruco example
			n = fs["camera_matrix"];
			if (n.type() != cv::FileNode::NONE)
			{
				n >> camMatrix;
				has_m1 = 1;
			}
			n = fs["distortion_coefficients"];
			if (n.type() != cv::FileNode::NONE)
			{
				n >> distCoeffs;
				has_d1 = 1;
			}
		}

		// check for fisheye model
		n = fs["distortion_model"];
		if (n.isString())
		{
			std::string mdl_name = n;
			if (mdl_name.compare("fisheye") == 0)
			{
				is_fisheye = 1;
			}
		}

		// check height and width
		n = fs["width"];
		if (n.type() != cv::FileNode::NONE)
		{
			n >> w;
			has_w = 1;
		}
		n = fs["height"];
		if (n.type() != cv::FileNode::NONE)
		{
			n >> h;
			has_h = 1;
		}

		// done with file now
		fs.release();

		// make sure we loaded the matrices in
		if (!has_m1 || (stereo_setup && !has_m2))
		{
			fprintf(stderr, "failed to find camera matrix in %s\n",
					intrinsics_path);
		}
		if (!has_d1 || (stereo_setup && !has_d2))
		{
			fprintf(stderr, "failed to find distortion coefficients in %s\n",
					intrinsics_path);
		}
		if (!has_w)
		{
			fprintf(stderr, "failed to find width in %s\n", intrinsics_path);
		}
		if (!has_h)
		{
			fprintf(stderr, "failed to find height in %s\n", intrinsics_path);
		}
		if (!has_m1 || !has_d1 || !has_w || !has_h
				|| (stereo_setup && (!has_m2 || !has_d2)))
		{
			return -1;
		}

		// logic here: if we are using a stereo pair, make sure we select the proper one if specified
		if (cam_info_set_vec[i].cam_mode != STEREO_RIGHT_ONLY)
		{
			cam_info_set_vec[i].cam_mat.push_back(cv::Matx33d(camMatrix));
			cam_info_set_vec[i].dist_coeffs.push_back(
					cv::Vec4d((double*) distCoeffs.ptr()));

			Eigen::Matrix<double, 10, 1> cam_intrins;

			cam_info_set_vec[i].cam_calib_intrinsic.push_back(cam_intrins);
			cam_info_set_vec[i].cam_calib_intrinsic[0](0, 0) = camMatrix.at<
					double>(0, 0);
			cam_info_set_vec[i].cam_calib_intrinsic[0](1, 0) = camMatrix.at<
					double>(1, 1);
			cam_info_set_vec[i].cam_calib_intrinsic[0](2, 0) = camMatrix.at<
					double>(0, 2);
			cam_info_set_vec[i].cam_calib_intrinsic[0](3, 0) = camMatrix.at<
					double>(1, 2);
			cam_info_set_vec[i].cam_calib_intrinsic[0](4, 0) = distCoeffs.at<
					double>(0);
			cam_info_set_vec[i].cam_calib_intrinsic[0](5, 0) = distCoeffs.at<
					double>(1);
			cam_info_set_vec[i].cam_calib_intrinsic[0](6, 0) = distCoeffs.at<
					double>(2);
			cam_info_set_vec[i].cam_calib_intrinsic[0](7, 0) = distCoeffs.at<
					double>(3);
			cam_info_set_vec[i].cam_calib_intrinsic[0](8, 0) = w;
			cam_info_set_vec[i].cam_calib_intrinsic[0](9, 0) = h;
			cam_info_set_vec[i].is_fisheye = is_fisheye;
		}
		else
		{
			Eigen::Matrix<double, 10, 1> cam_intrins_2;
			cam_info_set_vec[i].cam_calib_intrinsic.push_back(cam_intrins_2);

			cam_info_set_vec[i].cam_mat.push_back(cv::Matx33d(camMatrix2));
			cam_info_set_vec[i].dist_coeffs.push_back(
					cv::Vec4d((double*) distCoeffs2.ptr()));

			cam_info_set_vec[i].cam_calib_intrinsic[0](0, 0) = camMatrix2.at<
					double>(0, 0);
			cam_info_set_vec[i].cam_calib_intrinsic[0](1, 0) = camMatrix2.at<
					double>(1, 1);
			cam_info_set_vec[i].cam_calib_intrinsic[0](2, 0) = camMatrix2.at<
					double>(0, 2);
			cam_info_set_vec[i].cam_calib_intrinsic[0](3, 0) = camMatrix2.at<
					double>(1, 2);
			cam_info_set_vec[i].cam_calib_intrinsic[0](4, 0) = distCoeffs2.at<
					double>(0);
			cam_info_set_vec[i].cam_calib_intrinsic[0](5, 0) = distCoeffs2.at<
					double>(1);
			cam_info_set_vec[i].cam_calib_intrinsic[0](6, 0) = distCoeffs2.at<
					double>(2);
			cam_info_set_vec[i].cam_calib_intrinsic[0](7, 0) = distCoeffs2.at<
					double>(3);
			cam_info_set_vec[i].cam_calib_intrinsic[0](8, 0) = w;
			cam_info_set_vec[i].cam_calib_intrinsic[0](9, 0) = h;
			cam_info_set_vec[i].is_fisheye = is_fisheye;
		}

		if (cam_info_set_vec[i].cam_mode == STEREO)
		{
			Eigen::Matrix<double, 10, 1> cam_intrins_2;
			cam_info_set_vec[i].cam_calib_intrinsic.push_back(cam_intrins_2);

			cam_info_set_vec[i].cam_mat.push_back(cv::Matx33d(camMatrix2));
			cam_info_set_vec[i].dist_coeffs.push_back(
					cv::Vec4d((double*) distCoeffs2.ptr()));

			cam_info_set_vec[i].cam_calib_intrinsic[1](0, 0) = camMatrix2.at<
					double>(0, 0);
			cam_info_set_vec[i].cam_calib_intrinsic[1](1, 0) = camMatrix2.at<
					double>(1, 1);
			cam_info_set_vec[i].cam_calib_intrinsic[1](2, 0) = camMatrix2.at<
					double>(0, 2);
			cam_info_set_vec[i].cam_calib_intrinsic[1](3, 0) = camMatrix2.at<
					double>(1, 2);
			cam_info_set_vec[i].cam_calib_intrinsic[1](4, 0) = distCoeffs2.at<
					double>(0);
			cam_info_set_vec[i].cam_calib_intrinsic[1](5, 0) = distCoeffs2.at<
					double>(1);
			cam_info_set_vec[i].cam_calib_intrinsic[1](6, 0) = distCoeffs2.at<
					double>(2);
			cam_info_set_vec[i].cam_calib_intrinsic[1](7, 0) = distCoeffs2.at<
					double>(3);
			cam_info_set_vec[i].cam_calib_intrinsic[1](8, 0) = w;
			cam_info_set_vec[i].cam_calib_intrinsic[1](9, 0) = h;
			cam_info_set_vec[i].is_fisheye = is_fisheye;
		}
		width = std::max(w, width);
		height = std::max(h, height);
		cam_info_set_vec[i].is_fisheye = is_fisheye;
	}
	return 0;
}

int get_config_as_json()
{
	// add in optional fields to the info JSON file
	cam_json = cJSON_CreateObject();
	if (cam_json == NULL)
	{
		fprintf(stderr, "ERROR: in %s, failed to make new cam_json object\n",
				__FUNCTION__);
		return -1;
	}

	cJSON_AddStringToObject(cam_json, "imu", tmp_imu_name);

	// now loop through all active cameras
	// publish cam %d name, followed by calib info
	// followed by cam %d wrt imu
	std::string curr_cam_name;
	std::string curr_cam_mode;
	double *curr_cam_cal;
	double *curr_cam_distortion;
	double *curr_cam_wrt_imu;
	double *ov_cam_wrt_imu;
	double *ov_cam_calib;
	double *ov_world_correction;

	cJSON *json_camera_array = cJSON_AddArrayToObject(cam_json, "cameras");

	for (size_t i = 0; i < cam_info_set_vec.size(); i++)
	{
		cJSON *node = cJSON_CreateObject();
		curr_cam_name = std::string(cam_info_set_vec[i].name);
		cJSON_AddStringToObject(node, "cam name", curr_cam_name.c_str());

		curr_cam_mode = camera_mode_as_string(cam_info_set_vec[i].cam_mode);
		cJSON_AddStringToObject(node, "cam mode", curr_cam_mode.c_str());

		curr_cam_cal = (double*) cv::Mat(cam_info_set_vec[i].cam_mat[0]).data;
		curr_cam_distortion = (double*) cv::Mat(
				cam_info_set_vec[i].dist_coeffs[0]).data;
		curr_cam_wrt_imu =
				(double*) cam_info_set_vec[i].cam_wrt_imu_rot[0].data;

		cJSON *cam_cal = cJSON_CreateDoubleArray(curr_cam_cal, 9);
		cJSON *cam_dist = cJSON_CreateDoubleArray(curr_cam_distortion, 4);
		cJSON *cam_imu = cJSON_CreateDoubleArray(curr_cam_wrt_imu, 9);

		cJSON_AddItemToObject(node, "calibration", cam_cal);
		cJSON_AddItemToObject(node, "distortion", cam_dist);
		cJSON_AddItemToObject(node, "wrt_imu", cam_imu);

		ov_cam_wrt_imu = (double*) cam_info_set_vec[i].cam_wrt_imu[0].data();
		ov_cam_calib =
				(double*) cam_info_set_vec[i].cam_calib_intrinsic[0].data();
		ov_world_correction = (double*) cv::Mat(tmp_world_correction).data;

		cJSON *ov_cam_imu = cJSON_CreateDoubleArray(ov_cam_wrt_imu, 7);
		cJSON *ov_cam_cal = cJSON_CreateDoubleArray(ov_cam_calib, 10);
		cJSON *ov_world = cJSON_CreateDoubleArray(ov_world_correction, 9);

		cJSON_AddItemToObject(node, "ov_cam_wrt_imu", ov_cam_imu);
		cJSON_AddItemToObject(node, "ov_cam_cal", ov_cam_cal);
		cJSON_AddItemToObject(node, "ov_world_correction", ov_world);

		cJSON_AddBoolToObject(node, "fisheye",
				cam_info_set_vec[i].is_fisheye ? 1 : 0);

		cJSON_AddItemToArray(json_camera_array, node);

		if (cam_info_set_vec[i].cam_wrt_imu_rot.size() > 1)
		{ // stereo pair case
			cJSON *node2 = cJSON_CreateObject();

			cJSON_AddStringToObject(node2, "cam name", curr_cam_name.c_str());
			cJSON_AddStringToObject(node2, "cam mode", curr_cam_mode.c_str());

			curr_cam_cal =
					(double*) cv::Mat(cam_info_set_vec[i].cam_mat[1]).data;
			curr_cam_distortion = (double*) cv::Mat(
					cam_info_set_vec[i].dist_coeffs[1]).data;
			curr_cam_wrt_imu =
					(double*) cam_info_set_vec[i].cam_wrt_imu_rot[1].data;

			cJSON *cam_cal = cJSON_CreateDoubleArray(curr_cam_cal, 9);
			cJSON *cam_dist = cJSON_CreateDoubleArray(curr_cam_distortion, 4);
			cJSON *cam_imu = cJSON_CreateDoubleArray(curr_cam_wrt_imu, 9);

			cJSON_AddItemToObject(node2, "calibration", cam_cal);
			cJSON_AddItemToObject(node2, "distortion", cam_dist);
			cJSON_AddItemToObject(node2, "wrt_imu", cam_imu);

			ov_cam_wrt_imu =
					(double*) cam_info_set_vec[i].cam_wrt_imu[1].data();
			ov_cam_calib =
					(double*) cam_info_set_vec[i].cam_calib_intrinsic[1].data();

			cJSON *ov_cam_imu = cJSON_CreateDoubleArray(ov_cam_wrt_imu, 7);
			cJSON *ov_cam_cal = cJSON_CreateDoubleArray(ov_cam_calib, 10);

			cJSON_AddItemToObject(node2, "ov_cam_wrt_imu", ov_cam_imu);
			cJSON_AddItemToObject(node2, "ov_cam_cal", ov_cam_cal);

			cJSON_AddBoolToObject(node2, "fisheye",
					cam_info_set_vec[i].is_fisheye ? 1 : 0);

			cJSON_AddItemToArray(json_camera_array, node2);
		}
	}
	return 0;
}

int cam_config_file_print(void)
{
	printf(
			"=================================================================\n");
	for (int i = 0; i < (int) cam_info_set_vec.size(); i++)
	{
		printf(
				"==========================CAMERA %d================================\n",
				i);
		printf("name:                             %s\n",
				cam_info_set_vec[0].name);
		printf("mode:                             %s\n",
				camera_mode_as_string(cam_info_set_vec[0].cam_mode).c_str());
		for (int j = 0; j < (int) cam_info_set_vec[i].cam_mat.size(); j++)
		{
			printf("camera matrix:\n");
			for (int x = 0; x < 3; x++)
			{
				for (int y = 0; y < 3; y++)
				{
					printf("%6.5f ", cam_info_set_vec[i].cam_mat[j](x, y));
				}
				printf("\n");
			}
			printf("\n");
			printf("distortion coefficients:\n");
			for (int x = 0; x < 4; x++)
			{
				printf("%6.5f ", cam_info_set_vec[i].dist_coeffs[j][x]);
			}
			printf("\n");
		}
	}

// NOT USED in OV
//	printf("en_database:                      %d\n", en_database);
//	printf("database_size:                    %d\n", database_size);
//	printf("max_angular_rate_before_blur:  %3.2f\n",
//			max_angular_rate_before_blur);
	printf(
			"=============================KLT=================================\n");
	printf("num_features_to_track:            %d\n", num_features_to_track);
	printf("grid_x:                           %d\n", grid_x);
	printf("grid_y:                           %d\n", grid_y);
	printf("min_pix_dist:                     %d\n", min_pix_dist);
	printf("pyramid_levels:                   %d\n", pyramid_levels);
	printf("[block] window_size:                 (%d x %d)\n", window_size, window_size);
	printf(
			"===========================KLTGYRO===============================\n");
	printf("tmp_imu_name:                         %s\n", tmp_imu_name);
	for (size_t j = 0; j < cam_info_set_vec.size(); j++)
	{
		printf("%s wrt %s:\n", cam_info_set_vec[j].name, tmp_imu_name);
		for (int x = 0; x < 3; x++)
		{
			for (int y = 0; y < 3; y++)
			{
				printf("%6.5f ",
						cam_info_set_vec[j].cam_wrt_imu_rot[0].at<double>(x,
								y));
			}
			printf("\n");
		}
		if (cam_info_set_vec[j].cam_wrt_imu_rot.size() > 1)
		{
			printf("cam [pair_2] wrt %s (stereo pair):\n", tmp_imu_name);
			for (int x = 0; x < 3; x++)
			{
				for (int y = 0; y < 3; y++)
				{
					printf("%6.5f ",
							cam_info_set_vec[j].cam_wrt_imu_rot[1].at<double>(x,
									y));
				}
				printf("\n");
			}
		}
	}
	return 0;
}

int cam_config_file_read(void)
{
	int ret = json_make_empty_file_with_header_if_missing(CAM_CONFIG_FILE,
			CAM_CONFIG_FILE_HEADER);
	if (ret < 0)
	{
		fprintf(stderr, "FAILED new config file: %s\n", CAM_CONFIG_FILE);
		return -1;
	}
	else if (ret > 0)
	{
		fprintf(stderr, "Creating new config file: %s\n", CAM_CONFIG_FILE);
	}

	cJSON *parent = json_read_file(CAM_CONFIG_FILE);
	if (parent == NULL)
	{
		fprintf(stderr, "FAILED to parse: %s\n", CAM_CONFIG_FILE);
		return -1;
	}

	char string_holder[CM_CHAR_BUF_SIZE];
	memset(string_holder, '\0', CM_CHAR_BUF_SIZE);

	for (int i = 0; i < MAX_CAMERAS; i++)
	{
		std::stringstream ss;
		std::string std_holder;

		ss << "cam" << i << "_enable";
		std_holder = ss.str();

		int enabled = 0;
		int default_val = 0;
		if (i < 1)
			default_val = 1;
		json_fetch_int_with_default(parent, std_holder.c_str(), &enabled,
				default_val);

		std::cout << "Found camera entry: " << std_holder << " enabled? "<<  (enabled ? "Yes" : "No") << std::endl;

		if (!enabled)
			continue;

		ss.str("");
		ss << "cam" << i << "_name";
		std_holder = ss.str();
		json_fetch_string_with_default(parent, std_holder.c_str(),
				string_holder, CM_CHAR_BUF_SIZE, "tracking");
		if (strlen(string_holder) != 0)
		{
			camera_info_set curr_info;
			strncpy(curr_info.name, string_holder, CM_CHAR_BUF_SIZE);
			cam_info_set_vec.push_back(curr_info);
			memset(string_holder, '\0', CM_CHAR_BUF_SIZE);
		}

		ss.str("");
		ss << "cam" << i << "_mode";
		std_holder = ss.str();
		json_fetch_string_with_default(parent, std_holder.c_str(),
				string_holder, CM_CHAR_BUF_SIZE, "STEREO");
		if (strlen(string_holder) != 0)
		{
			camera_mode curr_mode;
			curr_mode = string_camera_mode_to_enum(string_holder);
			cam_info_set_vec.back().cam_mode = curr_mode;
			memset(string_holder, '\0', CM_CHAR_BUF_SIZE);
		}
		ss.str("");

		ss << "cam" << i << "_extrinsics_extension_first";
		std_holder = ss.str();
		json_fetch_string_with_default(parent, std_holder.c_str(),
				string_holder, CM_CHAR_BUF_SIZE, "_front");

		if (strlen(string_holder) != 0)
		{
			strncpy(cam_info_set_vec.back().extrinsics_extension_first,
					string_holder, CM_CHAR_BUF_SIZE);
			memset(string_holder, '\0', CM_CHAR_BUF_SIZE);
		}

		ss.str("");
		ss << "cam" << i << "_extrinsics_extension_second";
		std_holder = ss.str();
		json_fetch_string_with_default(parent, std_holder.c_str(),
				string_holder, CM_CHAR_BUF_SIZE, "_down");
		if (strlen(string_holder) != 0)
		{
			strncpy(cam_info_set_vec.back().extrinsics_extension_second,
					string_holder, CM_CHAR_BUF_SIZE);
			memset(string_holder, '\0', CM_CHAR_BUF_SIZE);
		}
	}

	json_fetch_string_with_default(parent, "tmp_imu_name", tmp_imu_name,
			CM_CHAR_BUF_SIZE, "imu_apps");
	json_fetch_int_with_default(parent, "num_features_to_track",
			&num_features_to_track, 20);
	json_fetch_int_with_default(parent, "grid_x", &grid_x, 10);
	json_fetch_int_with_default(parent, "grid_y", &grid_y, 10);
	json_fetch_int_with_default(parent, "min_pix_dist", &min_pix_dist, 10);
	json_fetch_int_with_default(parent, "pyramid_levels", &pyramid_levels, 3);
	json_fetch_int_with_default(parent, "window_size", &window_size, 25);
	json_fetch_bool_with_default(parent, "en_gyro", (int*) &en_gyro, 1);
	json_fetch_bool_with_default(parent, "en_descriptors",
			(int*) &en_descriptors, 0);
	json_fetch_bool_with_default(parent, "en_database", (int*) &en_database, 1);
	json_fetch_int_with_default(parent, "database_size", &database_size, 50);
	json_fetch_double_with_default(parent, "max_angular_rate_before_blur",
			&max_angular_rate_before_blur, 40.0); // negative means ignored/disabled

	if (json_get_parse_error_flag())
	{
		fprintf(stderr, "failed to parse config file %s\n", CAM_CONFIG_FILE);
		cJSON_Delete(parent);
		return -1;
	}

	// write modified data to disk if neccessary
	if (json_get_modified_flag())
	{
		printf(
				"The config file was modified during parsing, saving the changes to disk\n");
		json_write_to_file_with_header(CAM_CONFIG_FILE, parent,
				CAM_CONFIG_FILE_HEADER);
	}
	cJSON_Delete(parent);
	ret = cam_load_intrinsics_file();
	if (ret < 0)
	{
		fprintf(stderr, "FAILED cam_load_intrinsics_file: %d\n", ret);
		return ret;
	}

	ret = cam_load_extrinsics_file();
	if (ret < 0)
	{
		fprintf(stderr, "FAILED cam_load_extrinsics_file: %d\n", ret);
		return ret;
	}

	ret = get_config_as_json();
	if (ret < 0)
	{
		fprintf(stderr, "FAILED get_config_as_json: %d\n", ret);
		return ret;
	}
	printf("Done transfer camera configuration transfer\n");

	return 0;
}
