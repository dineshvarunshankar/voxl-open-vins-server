/*******************************************************************************
 * Copyright 2024 ModalAI Inc.
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
#include <core/VioManager.h>
#include <iostream>
#include <string>
#include <Eigen/Geometry>

#include "config_file.h"
#include "cam_config_file.h"

#include "rc_transform.h"


// extern variable in cam_config_file.hpp
std::vector<camera_info> cam_info_vec;
char imu_name[64];
Eigen::Matrix3d world_correction_eigen;
Eigen::Matrix<double, 4, 1> world_correction_q;

int takeoff_cam = -1; // set nonzero when we find a non-obscured cam

static void quat_2_rot(Eigen::Matrix<double, 4, 1> quaternion, Eigen::Matrix<double,3,3>  &R)
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
	return;
}


static void extrinsicsNEDtoFLU(Eigen::Matrix<double,3,3>  &R, Eigen::Matrix<double, 4, 1> &quaternion)
{
	static int camera_ctn = 0;
	double r,p,y;

	// Get the Euler angles in XYZ, RPY order.
	Eigen::Vector3d eulerAngles = R.eulerAngles(0, 1, 2);

	// TODO Fix anything needed for FLIPPED VOXL2 board, aka when IMU is upside down
	
	// en_force_ned_2_flu is set in the main config file, don't know what it does
	if (en_force_ned_2_flu)
	{
		// Case when user enters directly FLU RPY
		r = eulerAngles(0);
		p = eulerAngles(1);
		y = eulerAngles(2);
	}
	else
	{
		r = eulerAngles(0);
		p = M_PI - eulerAngles(1);
		y = eulerAngles(2) - M_PI;
	}
		
	  
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
	return;
}



int cam_config_file_read(void)
{

	// load first enbaled vio cam from common vio-cam config file
	vio_cam_t vio_cams[VCC_MAX_VIO_CAMS];
	int n_cams = vcc_read_vio_cam_conf_file(vio_cams, VCC_MAX_VIO_CAMS, 1);
	if(n_cams < 1) return -1;
	printf("vio_cam config:\n");
	vcc_print_vio_cam_conf(vio_cams, n_cams);
	printf("=================================================================\n");

	// check extrinsics and cam cal are present
	for(int i=0; i<n_cams; i++){
		if(!vio_cams[i].is_extrinsic_present){
			fprintf(stderr, "failed to find extrinsic config for vio cam %s\n", vio_cams[i].name);
			return -1;
		}
		if(!vio_cams[i].is_cal_present){
			fprintf(stderr, "failed to find cam cal for vio cam %s\n", vio_cams[i].name);
			return -1;
		}
	}

	// loop through all the cameras and put the common info into the local camera_info format
	// which mimics very closely the openvins lib formaty for intrinsics and extrinsics
	// for easy copying in main()
	int is_there_an_occlluded_cam = 0;
	for(int i=0; i<n_cams; i++){

		camera_info cam;
		strcpy(cam.name, vio_cams[i].name);
		cam.mode = MONO;
		cam.cam_id = i;

		// check for the first camera that's not obscured
		if(takeoff_cam<0 && !vio_cams[i].is_occluded_on_ground) takeoff_cam = i;
		if(vio_cams[i].is_occluded_on_ground) is_there_an_occlluded_cam = 1;

		// intrinsics
		// TODO validate the OV radtan model matches opencv pinhole model
		// it probably don't since we use a 5-coefficicent polynomial
		cam.width  = vio_cams[i].cal.width;
		cam.height = vio_cams[i].cal.height;
		cam.cam_calib_intrinsic(0,0) = vio_cams[i].cal.fx;
		cam.cam_calib_intrinsic(1,0) = vio_cams[i].cal.fy;
		cam.cam_calib_intrinsic(2,0) = vio_cams[i].cal.cx;
		cam.cam_calib_intrinsic(3,0) = vio_cams[i].cal.cy;
		cam.cam_calib_intrinsic(4,0) = vio_cams[i].cal.D[0];
		cam.cam_calib_intrinsic(5,0) = vio_cams[i].cal.D[1];
		cam.cam_calib_intrinsic(6,0) = vio_cams[i].cal.D[2];
		cam.cam_calib_intrinsic(7,0) = vio_cams[i].cal.D[3];
		if(vio_cams[i].cal.is_fisheye) cam.is_fisheye = 1;
		else cam.is_fisheye = 0;

		//extrinsics is awkward since we need to convert to FLU
		Eigen::Matrix<double, 3, 3> rotation_temp;
		for(int j = 0; j < 3; j++){
			for (int k = 0; k < 3; k++){
				rotation_temp(j, k) = vio_cams[i].extrinsic.R_child_to_parent[j][k];
				if (fabs(rotation_temp(j, k)) < 10e-6) rotation_temp(j, k) = 0.;
			}
		}

		// convert extrinsics from NED  to FLU
		Eigen::Matrix<double, 4, 1> quaternion;
		extrinsicsNEDtoFLU(rotation_temp, quaternion);

		// WARNING: FLU coordindate system needs YZ flipped
		Eigen::Matrix<double, 3, 1> translation;
		translation[0] = vio_cams[i].extrinsic.T_child_wrt_parent[0];
		translation[1] = -1 * vio_cams[i].extrinsic.T_child_wrt_parent[1];
		translation[2] = -1 * vio_cams[i].extrinsic.T_child_wrt_parent[2];

		// openvins lib wants extrinsics in this format
		cam.cam_wrt_imu.block(0, 0, 4, 1) = quaternion;
		cam.cam_wrt_imu.block(4, 0, 3, 1) = translation;

		// add this cam_info to our vector
		cam_info_vec.push_back(cam);
	}

	// if for some reason the vio-cams common config was malformed and all cams
	// were marked as occluded on ground, just default to the first cam for takeoff
	// this is usually tracking_front
	if(takeoff_cam<0) takeoff_cam = 0;

	// if no cams were occluded, we can disable the blind takeoff feature
	if(!is_there_an_occlluded_cam) takeoff_cam = -1;

	// openvins tries to be clever and rotate the whole local frame around
	// to align with gravity. So we need to figure out how the imu is mounted
	// in the drone to undo that nonsense.
	strcpy(imu_name, vio_cams[0].imu);
	double R_imu_to_body[3][3];
	if(vcc_fetch_R_child_to_body(imu_name, R_imu_to_body)){
		fprintf(stderr, "ERROR in %s, failed to find imu orientation in extrinsics\n", __FUNCTION__);
		return -1;
	}

	for(int i = 0; i < 3; ++i) {
		for(int j = 0; j < 3; ++j) {
			world_correction_eigen(i, j) = R_imu_to_body[i][j];
		}
	}
	world_correction_q = ov_core::rot_2_quat(world_correction_eigen);

	printf("WORLD QUATERNION: %f %f %f %f\n", world_correction_q.x(), world_correction_q.y(), world_correction_q.z(), world_correction_q.w());
	return 0;
}



