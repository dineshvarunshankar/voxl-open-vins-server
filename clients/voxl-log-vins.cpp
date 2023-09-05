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
#include <map>

#include <modal_pipe_client.h>
#include <modal_start_stop.h>

#define CLIENT_NAME		"voxl-log-vins"

#define DEG_TO_RAD	(3.14159265358979323846/180.0)
#define RAD_TO_DEG	(180.0/3.14159265358979323846)

static char pipe_path[MODAL_PIPE_MAX_PATH_LEN] = "/run/mpa/ov_status";
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
static int report_num = 20;

#define DISABLE_WRAP		"\033[?7l"	// disables line wrap, be sure to enable before exiting
#define ENABLE_WRAP			"\033[?7h"	// default terminal behavior
#define RESET_FONT			"\x1b[0m"	// undo any font/color settings
#define FONT_BOLD			"\033[1m"	// bold font
#define CLEAR_LINE			"\033[2K"	// erases line but leaves curser in place

std::map <int, float[2]> _featDB;

typedef struct _klt_feature_t{
    uint32_t  id;               ///< unique ID for feature point
    int32_t cam_id;             ///< ID of camera which the point was seen from (typically first)
    float pix_loc[2];           ///< pixel location in the last frame
//    float depth_error_stddev;   ///< depth error in meters
} klt_feature_t;

typedef struct _ov_status_t {
    uint32_t magic_number;         ///< Unique 32-bit number used to signal the beginning of a VIO packet while parsing a data stream.
    int64_t timestamp_ns;          ///< Timestamp in clock_monotonic system time of the provided pose.
    int32_t quality;
    float p_dop;
    float r_dop;
    int num_features;
    klt_feature_t features[VIO_MAX_REPORTED_FEATURES];
} ov_status_t;
static ov_status_t ov_status;


static void _print_usage(void)
{
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
	return;
}


// called whenever we disconnect from the server
static void _disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
	fprintf(stderr, "\nvoxl-open-vins-server disconnected\n");
	if(!en_newline) printf("\r" CLEAR_LINE);
	return;
}


#define MAX_DB_SZ 2048
static void _print_data(ov_status_t d)
{

//	if (_featDB.size() > MAX_DB_SZ)
//		_featDB.clear();

	printf("*, %f, %ld, %d, %f, %f, %d,",(int) _featDB.size(), d.timestamp_ns*1e-9, d.quality, d.p_dop, d.r_dop, d.num_features);

	for (int i=0; i<VIO_MAX_REPORTED_FEATURES; i++)
	{

		_featDB[d.features[i].id][0] =  d.features[i].pix_loc[0];
		_featDB[d.features[i].id][1] =  d.features[i].pix_loc[1];
	}

	std::map <int, float[2]>::iterator it;
	int idx = 0;
	for(it=_featDB.begin(); it!=_featDB.end(); ++it){
		if (idx++ > 5)
			break;

		printf(" %d, %d, %d,", it->first, (int)it->second[0], (int)it->second[1]);
	}

	printf("\n");
	return;
}

ov_status_t* pipe_validate_ov_status_data_t(char* data, int bytes, int* n_packets){
    // cast raw data from buffer to an vio_data_t array so we can read data
    // without memcpy. Also write out packets read as 0 until we validate data.
	ov_status_t* new_ptr = (ov_status_t*) data;
    *n_packets = 0;

    // basic sanity checks
    if(bytes<0){
        fprintf(stderr, "ERROR validating VIO data received through pipe: number of bytes = %d\n", bytes);
        return NULL;
    }
    if(data==NULL){
        fprintf(stderr, "ERROR validating VIO data received through pipe: got NULL data pointer\n");
        return NULL;
    }
    if(bytes%sizeof(ov_status_t)){
        fprintf(stderr, "ERROR validating EXT VIO data received through pipe: read partial packet\n");
        fprintf(stderr, "read %d bytes, but it should be a multiple of %zu\n", bytes, sizeof(ov_status_t));
        return NULL;
    }

    // calculate number of packets locally until we validate each packet
    int n_packets_tmp = bytes/sizeof(ov_status_t);

    // check if any packets failed the magic number check
    int i, n_failed = 0;
    for(i=0;i<n_packets_tmp;i++){
        if(new_ptr[i].magic_number != VIO_MAGIC_NUMBER) n_failed++;
    }
    if(n_failed>0){
        fprintf(stderr, "ERROR validating VIO data received through pipe: %d of %d packets failed\n", n_failed, n_packets_tmp);
        return NULL;
    }

    // if we get here, all good. Write out the number of packets read and return
    // the new cast pointer. It's the same pointer the user provided but cast to
    // the right type for simplicity and easy of use.
    *n_packets = n_packets_tmp;
    return new_ptr;
}


static void _helper_cb( __attribute__((unused)) int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
	// validate that the data makes sense
	int n_packets, i;
	ov_status_t* data_array = pipe_validate_ov_status_data_t(data, bytes, &n_packets);
	if(data_array == NULL) return;
	for(i=0;i<n_packets;i++) _print_data(data_array[i]);
	return;
}


static int _parse_opts(int argc, char* argv[])
{
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
	printf("waiting for server at %s\n", pipe_path);
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
