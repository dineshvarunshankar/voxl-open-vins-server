/*******************************************************************************
 * Copyright 2020 ModalAI Inc.
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

#include <modal_pipe_client.h>
#include <modal_start_stop.h>

#define CLIENT_NAME		"voxl-inspect-vins"

#define DEG_TO_RAD	(3.14159265358979323846/180.0)
#define RAD_TO_DEG	(180.0/3.14159265358979323846)

static char pipe_path[MODAL_PIPE_MAX_PATH_LEN] = "/run/mpa/ov_extended";
static int en_imu_angular_vel = 0;
static int en_error_code = 1;
static int en_n_feature_points = 0;
static int en_gravity_vector = 0;
static int en_extrinsics = 0;
static int en_newline = 0;
static int en_quality = 0;
static int en_state = 1;
static int en_timestamp_ns = 0;
static int en_vel_imu_wrt_vio = 0;


#define DISABLE_WRAP		"\033[?7l"	// disables line wrap, be sure to enable before exiting
#define ENABLE_WRAP			"\033[?7h"	// default terminal behavior
#define RESET_FONT			"\x1b[0m"	// undo any font/color settings
#define FONT_BOLD			"\033[1m"	// bold font
#define CLEAR_LINE			"\033[2K"	// erases line but leaves curser in place


static void _print_usage(void)
{
	printf("\n\
typical usage\n\
/# voxl-inspect-open-vins\n\
\n\
This will print out vio data from Modal Pipe Architecture.\n\
By default this opens the extended qvio pipe /run/mpa/qvio_extended/\n\
but this can be changed with the --pipe option.\n\
\n\
Position and rotation will always print. Additional options are:\n\
-a, --imu_angular_vel       print imu_angular_vel\n\
-b, --accl_gyro_bias        print accl and gyro bias\n\
-c, --time_shift_s          print imu_cam_time_shift_s\n\
-f, --n_feature_points      print n_feature_points\n\
-g, --gravity_vector        print gravity_vector\n\
-h, --help                  print this help message\n\
-m, --extrinsics            print cam to imu extrinsics\n\
-n, --newline               print each sample on a new line\n\
-p, --pipe {pipe_name}      optionally specify the pipe name\n\
-q, --quality               print quality\n\
-t, --timestamp_ns          print timestamp_ns\n\
-v, --vel_imu_wrt_vio       print vel_imu_wrt_vio\n\
-z, --print_everything      print everything\n\
\n");
	return;
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
	printf(FONT_BOLD);
	printf("\n    T_imu_wrt_vio (m)   |");
	printf("Roll Pitch Yaw (deg)|");
	if(en_vel_imu_wrt_vio)		printf("   velocity (m/s)   |");
	if(en_imu_angular_vel)		printf(" angular_vel(deg/s) |");
	if(en_n_feature_points)		printf("features|");
	if(en_gravity_vector)		printf("gravity_vector(m/s2)|");
	if(en_extrinsics)			printf(" cam_wrt_imu XYZ(m) , imu_to_cam RPY(deg)|");
	if(en_quality)				printf("  quality |");
	if(en_timestamp_ns)			printf(" timestamp (ns) |");
	if(en_state)				printf(" state|");
	if(en_error_code)			printf(" error_code");
	printf("\n");
	printf(RESET_FONT);
	return;
}


// called whenever we disconnect from the server
static void _disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
	fprintf(stderr, "\nvoxl-open-vins-server disconnected\n");
	if(!en_newline) printf("\r" CLEAR_LINE);
	return;
}


static void _print_data(ext_vio_data_t d)
{
	if(!en_newline) printf("\r" CLEAR_LINE);

	// always print translation and rotation
	printf("%8.2f%8.2f%8.2f|", (double)d.v.T_imu_wrt_vio[0], (double)d.v.T_imu_wrt_vio[1], (double)d.v.T_imu_wrt_vio[2]);
	float roll, pitch, yaw;
	_rotation_to_tait_bryan(d.v.R_imu_to_vio, &roll, &pitch, &yaw);
	printf("%6.1f %6.1f %6.1f|", (double)roll, (double)pitch, (double)yaw);

	if(en_vel_imu_wrt_vio){
		printf("%6.2f %6.2f %6.2f|", (double)d.v.vel_imu_wrt_vio[0], (double)d.v.vel_imu_wrt_vio[1], (double)d.v.vel_imu_wrt_vio[2]);
	}

	if(en_imu_angular_vel){
		printf("%6.1f %6.1f %6.1f|", (double)d.v.imu_angular_vel[0]*RAD_TO_DEG, (double)d.v.imu_angular_vel[1]*RAD_TO_DEG, (double)d.v.imu_angular_vel[2]*RAD_TO_DEG);
	}
	if(en_n_feature_points){
		printf("  %4d  |", d.v.n_feature_points);
	}
	if(en_gravity_vector){
		printf("%6.3f %6.3f %6.3f|", (double)d.v.gravity_vector[0], (double)d.v.gravity_vector[1], (double)d.v.gravity_vector[2]);
	}
	if(en_extrinsics){
		_rotation_to_tait_bryan_xyz_intrinsic(d.v.R_cam_to_imu, &roll, &pitch, &yaw);
		printf("%6.3f %6.3f %6.3f,", (double)d.v.T_cam_wrt_imu[0], (double)d.v.T_cam_wrt_imu[1], (double)d.v.T_cam_wrt_imu[2]);
		printf("%6.1f %6.1f %6.1f|", (double)roll*RAD_TO_DEG, (double)pitch*RAD_TO_DEG, (double)yaw*RAD_TO_DEG);
	}
	if(en_quality){
		printf("%10.5f|", (double)d.v.quality);
	}
	if(en_timestamp_ns){
		printf("%15ld |", d.v.timestamp_ns);
	}
	if(en_state){
		printf(" ");
		pipe_print_vio_state(d.v.state);
		printf(" |");
	}
	if(en_error_code){
		printf(" ");
		pipe_print_vio_error(d.v.error_code);
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
	ext_vio_data_t* data_array = pipe_validate_ext_vio_data_t(data, bytes, &n_packets);
	if(data_array == NULL) return;
	for(i=0;i<n_packets;i++) _print_data(data_array[i]);
	return;
}


static int _parse_opts(int argc, char* argv[])
{
	static struct option long_options[] =
	{
		{"imu_angular_vel",		no_argument,		0, 'a'},
		{"accl_gyro_bias",		no_argument,		0, 'b'},
		{"time_shift_s",		no_argument,		0, 'c'},
		{"n_feature_points",	no_argument,		0, 'f'},
		{"gravity_vector",		no_argument,		0, 'g'},
		{"help",				no_argument,		0, 'h'},
		{"extrinsics",			no_argument,		0, 'm'},
		{"newline",				no_argument,		0, 'n'},
		{"pipe",				required_argument,	0, 'p'},
		{"quality",				no_argument,		0, 'q'},
		{"timestamp_ns",		no_argument,		0, 't'},
		{"vel_imu_wrt_vio",		no_argument,		0, 'v'},
		{"print_everything",	no_argument,		0, 'z'},
		{0, 0, 0, 0}
	};

	while(1){
		int option_index = 0;
		int c = getopt_long(argc, argv, "abcfghmnp:qtvz", long_options, &option_index);

		if(c == -1) break; // Detect the end of the options.

		switch(c){
		case 0:
			// for long args without short equivalent that just set a flag
			// nothing left to do so just break.
			if (long_options[option_index].flag != 0) break;
			break;

		case 'a':
			en_imu_angular_vel = 1;
			break;

		case 'f':
			en_n_feature_points = 1;
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

		case 'p':
			if(pipe_expand_location_string(optarg, pipe_path)<0){
				fprintf(stderr, "Invalid pipe name: %s\n", optarg);
				return -1;
			}
			break;

		case 'q':
			en_quality = 1;
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
	printf(DISABLE_WRAP);

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
		printf(ENABLE_WRAP);
		return -1;
	}

	// keep going until signal handler sets the running flag to 0
	while(main_running) usleep(200000);

	// all done, signal pipe read threads to stop
	printf("\nclosing and exiting\n");
	pipe_client_close_all();
	printf(ENABLE_WRAP);

	return 0;
}