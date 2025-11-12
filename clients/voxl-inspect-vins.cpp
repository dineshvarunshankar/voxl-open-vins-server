/*******************************************************************************
 * Copyright 2025 ModalAI Inc.
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

#include <stdio.h>
#include <signal.h>
#include <getopt.h>
#include <unistd.h>	// for usleep()
#include <string.h>
#include <stdlib.h> // for atoi()
#include <math.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <cJSON.h>
#include <cstring>
#include <iostream>

#include <modal_pipe_client.h>
#include <modal_start_stop.h>

#include "voxl_common_config.h"

#define CLIENT_NAME		"voxl-inspect-vins"

#define DEG_TO_RAD	(3.14159265358979323846/180.0)
#define RAD_TO_DEG	(180.0/3.14159265358979323846)

static char pipe_path[MODAL_PIPE_MAX_PATH_LEN] = "ov";
static int en_imu_angular_vel = 0;
static int en_error_code = 1;
static int en_n_feature_points = 1;
static int en_gravity_vector = 0;
static int en_extrinsics = 0;
static int en_wrt_body_frame = 0;
static float R_imu_to_body[3][3] = {
	{1.0f, 0.0f, 0.0f},
	{0.0f, 1.0f, 0.0f},
	{0.0f, 0.0f, 1.0f}
};
static int en_newline = 0;
static int en_quality = 1;
static int en_state = 1;
static int en_timestamp_ns = 0;
static int en_vel_imu_wrt_vio = 0;
static int en_dt = 1;


#define DISABLE_WRAP		"\033[?7l"	// disables line wrap, be sure to enable before exiting
#define ENABLE_WRAP			"\033[?7h"	// default terminal behavior
#define RESET_FONT			"\x1b[0m"	// undo any font/color settings
#define FONT_BOLD			"\033[1m"	// bold font
#define CLEAR_LINE			"\033[2K"	// erases line but leaves curser in place


static void _print_usage(void)
{
	printf("\n\
typical usage\n\
/# voxl-inspect-vins\n\
/# voxl-inspect-vins -v\n\
\n\
This will print out vio data from Modal Pipe Architecture.\n\
By default this opens the vvhub_aligned_vio pipe from voxl-vision-hub\n\
but this can be changed by specifying a pipe name to inspect\n\
qvio or openvins data directly. The vvhub_aligned_vio pipe is\n\
a gravity-aligned version of whatever raw vio source is being consumed\n\
by voxl-vision-hub and being sent to the autopilot and represents the\n\
COM of the drone in local FRD frame.\n\
\n\
Position and rotation will always print. Additional options are:\n\
-b, --body_frame			print values wrt body frame\n\
-a, --imu_angular_vel       print imu_angular_vel\n\
-g, --gravity_vector        print gravity_vector\n\
-h, --help                  print this help message\n\
-m, --extrinsics            print cam to imu extrinsics\n\
-n, --newline               print each sample on a new line\n\
-t, --timestamp_ns          print timestamp_ns\n\
-v, --vel_imu_wrt_vio       print vel_imu_wrt_vio\n\
-z, --print_everything      print everything\n\
\n");
	return;
}


enum class Axis { Roll, Pitch, Yaw };

static void generate_rotation_matrix(float angle_rad, Axis axis, float R[3][3])
{
    float c = std::cos(angle_rad);
    float s = std::sin(angle_rad);

    // Initialize to identity
    std::memset(R, 0, sizeof(float) * 9);
    R[0][0] = R[1][1] = R[2][2] = 1.0f;

    switch (axis) {
        case Axis::Roll: // Rotation around X
            R[1][1] =  c; R[1][2] = -s;
            R[2][1] =  s; R[2][2] =  c;
            break;
        case Axis::Pitch: // Rotation around Y
            R[0][0] =  c; R[0][2] =  s;
            R[2][0] = -s; R[2][2] =  c;
            break;
        case Axis::Yaw: // Rotation around Z
            R[0][0] =  c; R[0][1] = -s;
            R[1][0] =  s; R[1][1] =  c;
            break;
    }
}

static void mat3x3_multiply(const float A[3][3], const float B[3][3], float C[3][3])
{
    for (int i = 0; i < 3; ++i) {         // rows of A
        for (int j = 0; j < 3; ++j) {     // columns of B
            C[i][j] = 0.0f;
            for (int k = 0; k < 3; ++k) { // shared dimension
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

static void mat3x3_vec3_multiply(const float A[3][3], const float x[3], float y[3])
{
    for (int i = 0; i < 3; ++i) {
        y[i] = 0.0f;
        for (int j = 0; j < 3; ++j) {
            y[i] += A[i][j] * x[j];
        }
    }
}

static void apply_body_rotation(const float vin[3], float vout[3])
{
	if(en_wrt_body_frame) {
		mat3x3_vec3_multiply(R_imu_to_body, vin, vout);
	} else {
		memcpy(vout, vin, sizeof(float) * 3);
	}
}

/*
 * Convert from Rotation matrix representing transformation from
 * frame 2 to frame 1.
 * The result will hold the angles defining the 3-2-1 intrinsic
 * Tait-Bryan rotation sequence from frame 1 to frame 2.
 * This is the usual nautical/aerospace order
 */
static void _rotation_to_tait_bryan(float R[3][3], float* roll, float* pitch, float* yaw)
{
	*roll  = atan2(R[2][1], R[2][2]);
	*pitch = asin(-R[2][0]);
	*yaw   = atan2(R[1][0], R[0][0]);

	if(fabs((double)*pitch - M_PI_2) < 1.0e-3){
		*roll = 0.0;
		*pitch = atan2(R[1][2], R[0][2]);
	}
	else if(fabs((double)*pitch + M_PI_2) < 1.0e-3) {
		*roll = 0.0;
		*pitch = atan2(-R[1][2], -R[0][2]);
	}
	return;
}

/*
 * Convert from Rotation matrix representing transformation from
 * frame 2 to frame 1.
 * The result will hold the angles defining the 1-2-3 intrinsic
 * Tait-Bryan rotation sequence from frame 1 to frame 2.
 * This is the order used for imu-camera extrinsic
 */
static void _rotation_to_tait_bryan_xyz_intrinsic(float R[3][3], float* roll, float* pitch, float* yaw)
{
	*pitch = asin(R[0][2]);
	if(fabs(R[0][2]) < 0.9999999){
		*roll = atan2(-R[1][2], R[2][2]);
		*yaw  = atan2(-R[0][1], R[0][0]);
	}
	else{
		*roll = atan2(R[2][1], R[1][1]);
		*yaw  = 0.0f;
	}
	return;
}



// called whenever we connect or reconnect to the server
static void _connect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
	// Security: Use fputs for constant strings to avoid format string warnings
	fputs(FONT_BOLD, stdout);
	printf("\n");
	if(en_dt)					printf(" dt(ms) |");
	printf("    T_imu_wrt_vio (m)   |");
	printf("Roll Pitch Yaw (deg)|");
	if(en_vel_imu_wrt_vio)		printf("   velocity (m/s)   |");
	if(en_imu_angular_vel)		printf(" angular_vel(deg/s) |");
	if(en_n_feature_points)		printf("features|");
	if(en_gravity_vector)		printf("gravity_vector(m/s2)|");
	if(en_extrinsics)			printf(" cam_wrt_imu XYZ(m) , imu_to_cam RPY(deg)|");
	if(en_quality)				printf("quality|");
	if(en_timestamp_ns)			printf(" timestamp (ns) |");
	if(en_state)				printf(" state|");
	if(en_error_code)			printf(" error_codes ");
	printf("\n");
	fputs(RESET_FONT, stdout);
	return;
}


// called whenever we disconnect from the server
static void _disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
	fprintf(stderr, "\nvoxl-openvins-server disconnected\n");
	return;
}


static void _print_data(vio_data_t d)
{
	// keep track of time between samples
	static int64_t t_last = 0;
	double dt_ms;
	if(t_last == 0) dt_ms = 0.0;
	else dt_ms = (double)(d.timestamp_ns-t_last)/1000000.0;
	t_last = d.timestamp_ns;


	if(!en_newline) printf("\r" CLEAR_LINE);

	if(en_dt) printf("%7.1f |", dt_ms);

	// always print translation and rotation
	float T_out[3];
	apply_body_rotation(d.T_imu_wrt_vio, T_out);
	printf("%8.2f%8.2f%8.2f|", (double)T_out[0], (double)T_out[1], (double)T_out[2]);

	float roll, pitch, yaw;
	if (en_wrt_body_frame) {
		float R_imu_to_vio[3][3];
		mat3x3_multiply(d.R_imu_to_vio, R_imu_to_body, R_imu_to_vio);
		_rotation_to_tait_bryan(R_imu_to_vio, &roll, &pitch, &yaw);
	} else {
		_rotation_to_tait_bryan(d.R_imu_to_vio, &roll, &pitch, &yaw);
	}
	printf("%6.1f %6.1f %6.1f|", (double)roll*RAD_TO_DEG, (double)pitch*RAD_TO_DEG, (double)yaw*RAD_TO_DEG);
	
	if(en_vel_imu_wrt_vio){
		float vel_out[3];
		apply_body_rotation(d.vel_imu_wrt_vio, vel_out);
		printf("%6.2f %6.2f %6.2f|", (double)vel_out[0], (double)vel_out[1], (double)vel_out[2]);
	}

	if(en_imu_angular_vel){
		printf("%6.1f %6.1f %6.1f|", (double)d.imu_angular_vel[0]*RAD_TO_DEG, (double)d.imu_angular_vel[1]*RAD_TO_DEG, (double)d.imu_angular_vel[2]*RAD_TO_DEG);
	}
	if(en_n_feature_points){
		printf("  %4d  |", d.n_feature_points);
	}
	if(en_gravity_vector){
		printf("%6.3f %6.3f %6.3f|", (double)d.gravity_vector[0], (double)d.gravity_vector[1], (double)d.gravity_vector[2]);
	}
	if(en_extrinsics){
		_rotation_to_tait_bryan_xyz_intrinsic(d.R_cam_to_imu, &roll, &pitch, &yaw);
		printf("%6.3f %6.3f %6.3f,", (double)d.T_cam_wrt_imu[0], (double)d.T_cam_wrt_imu[1], (double)d.T_cam_wrt_imu[2]);
		printf("%6.1f %6.1f %6.1f|", (double)roll*RAD_TO_DEG, (double)pitch*RAD_TO_DEG, (double)yaw*RAD_TO_DEG);
	}
	if(en_quality){
		printf("  %3d%% |", d.quality);
	}
	if(en_timestamp_ns){
		printf("%15ld |", d.timestamp_ns);
	}
	if(en_state){
		printf(" ");
		pipe_print_vio_state(d.state);
		printf(" |");
	}
	if(en_error_code){
		printf(" ");
		pipe_print_vio_error(d.error_code);
	}

	// cleanup the end of the line depending on mode
	if(en_newline)  printf("\n");
	fflush(stdout);
	return;
}


static void _helper_cb( __attribute__((unused)) int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
	// validate that the data makes sense
	int n_packets, i;
	vio_data_t* data_array = pipe_validate_vio_data_t(data, bytes, &n_packets);

	if(data_array == NULL) return;
	for(i=0;i<n_packets;i++) _print_data(data_array[i]);
	return;
}


static int _parse_opts(int argc, char* argv[])
{
	static struct option long_options[] =
	{
		{"imu_angular_vel",		no_argument,		0, 'a'},
		{"gravity_vector",		no_argument,		0, 'g'},
		{"help",				no_argument,		0, 'h'},
		{"body_frame",          no_argument,        0, 'b'},
		{"extrinsics",			no_argument,		0, 'm'},
		{"newline",				no_argument,		0, 'n'},
		{"timestamp_ns",		no_argument,		0, 't'},
		{"vel_imu_wrt_vio",		no_argument,		0, 'v'},
		{"print_everything",	no_argument,		0, 'z'},
		{0, 0, 0, 0}
	};

	int body_frame_set = 0;

	while(1)
	{
		int option_index = 0;
		int c = getopt_long(argc, argv, "aghbmntvz", long_options, &option_index);

		if(c == -1) break; // Detect the end of the options.

		int body_frame_set = 0;

		switch(c){
		case 0:
			// for long args without short equivalent that just set a flag
			// nothing left to do so just break.
			if (long_options[option_index].flag != 0) break;
			break;

		case 'b':
			vcc_extrinsic_t all_extrinsics[VCC_MAX_EXTRINSICS_IN_CONFIG];
			int n_extrinsics_read;
			if(vcc_read_extrinsic_conf_file(VCC_EXTRINSICS_PATH, all_extrinsics, &n_extrinsics_read, VCC_MAX_EXTRINSICS_IN_CONFIG)){
				fprintf(stderr, "ERROR in %s failed to read extrinsics file\n", __FUNCTION__);
				return -1;
			}
			
			double RPY_parent_to_child[3];
			for(int i = 0; i < n_extrinsics_read; i++) {
				if(strcmp(all_extrinsics[i].parent, "body") == 0 && strcmp(all_extrinsics[i].child, "imu_apps") == 0) {
					body_frame_set = 1;
					RPY_parent_to_child[0] = all_extrinsics[i].RPY_parent_to_child[0] * DEG_TO_RAD;
					RPY_parent_to_child[1] = all_extrinsics[i].RPY_parent_to_child[1] * DEG_TO_RAD;
					RPY_parent_to_child[2] = all_extrinsics[i].RPY_parent_to_child[2] * DEG_TO_RAD;
					break;
				}
			}

			if (body_frame_set == 0) {
				fprintf(stderr, "ERROR in %s: body frame extrinsics not found in %s\n", __FUNCTION__, VCC_EXTRINSICS_PATH);
				return -1;
			}
				
			float Rx[3][3], Ry[3][3], Rz[3][3], tmp[3][3];
			generate_rotation_matrix(RPY_parent_to_child[0], Axis::Roll,  Rx);
			generate_rotation_matrix(RPY_parent_to_child[1], Axis::Pitch, Ry);
			generate_rotation_matrix(RPY_parent_to_child[2], Axis::Yaw,   Rz);
			
			mat3x3_multiply(Rx, Ry, tmp);
			mat3x3_multiply(tmp, Rz, R_imu_to_body);
			
			float R_transposed[3][3];
			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 3; ++j) {
					R_transposed[i][j] = R_imu_to_body[j][i];
				}
			}
			
			memcpy(R_imu_to_body, R_transposed, sizeof(R_imu_to_body));
			en_wrt_body_frame = 1;
				
			break;

		case 'a':
			en_imu_angular_vel = 1;
			break;

		case 'g':
			en_gravity_vector = 1;
			break;

		case 'h':
			_print_usage();
			return -1;

		case 'm':
			en_extrinsics = 1;
			break;

		case 'n':
			en_newline = 1;
			break;

		case 't':
			en_timestamp_ns = 1;
			break;

		case 'v':
			en_vel_imu_wrt_vio = 1;
			break;

		case 'z':
			// print everything, keep this updated with new options!!!
			en_imu_angular_vel = 1;
			en_n_feature_points = 1;
			en_gravity_vector = 1;
			en_extrinsics = 1;
			en_quality = 1;
			en_timestamp_ns = 1;
			en_vel_imu_wrt_vio = 1;
			break;


		default:
			_print_usage();
			return -1;
		}
	}

	return 0;
}


int main(int argc, char* argv[])
{
	// check for options
	if(_parse_opts(argc, argv)) return -1;

	// set some basic signal handling for safe shutdown.
	// quitting without cleanup up the pipe can result in the pipe staying
	// open and overflowing, so always cleanup properly!!!
	enable_signal_handler();
	main_running = 1;

	// prints can be quite long, disable terminal wrapping
	// Security: Use fputs for constant strings to avoid format string warnings
	fputs(DISABLE_WRAP, stdout);

	// set up all our MPA callbacks
	pipe_client_set_simple_helper_cb(0, _helper_cb, NULL);
	pipe_client_set_connect_cb(0, _connect_cb, NULL);
	pipe_client_set_disconnect_cb(0, _disconnect_cb, NULL);

	// request a new pipe from the server
	printf("waiting for server\n");
	int ret = pipe_client_open(0, pipe_path, CLIENT_NAME, \
				EN_PIPE_CLIENT_SIMPLE_HELPER, \
				VIO_RECOMMENDED_READ_BUF_SIZE);

	// check for MPA errors
	if(ret<0){
		pipe_print_error(ret);
		fputs(ENABLE_WRAP, stdout);
		return -1;
	}

	// keep going until signal handler sets the running flag to 0
	while(main_running) usleep(200000);

	// all done, signal pipe read threads to stop
	printf("\nclosing and exiting\n");
	pipe_client_close_all();
	fputs(ENABLE_WRAP, stdout);

	return 0;
}
