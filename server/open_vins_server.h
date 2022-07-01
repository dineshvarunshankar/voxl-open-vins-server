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


#ifndef VOXL_QVIO_SERVER_COMMON_H
#define VOXL_QVIO_SERVER_COMMON_H



// This includes the primary VIO pipe interface. The remainder of this file is
// for things specific to the QVIO server
#include <modal_pipe_interfaces.h>
#include <modal_pipe_common.h> // for MODAL_PIPE_DEFAULT_BASE_DIR


// config file exists here if you want to access it
#define VOXL_QVIO_SERVER_CONF_FILE	"/etc/modalai/voxl-qvio-server.conf"

// command string sent to either QVIO control pipe to blank out the camera
// image temporarily to test loss of features.
#define BLANK_VIO_CAM_COMMAND	"blank"

// command string sent to either QVIO control pipe to fade the camera
// image gradually out to black
#define FADE_VIO_CAM_COMMAND	"fade"

#define QVIO_CONTROL_COMMANDS (RESET_VIO_SOFT "," RESET_VIO_HARD "," BLANK_VIO_CAM_COMMAND "," FADE_VIO_CAM_COMMAND)

// These are the paths of the named pipe interfaces
// MODAL_PIPE_DEFAULT_BASE_DIR is defined in modal_pipe_common.h
// the 'complete' interface sends data as a qvio_data_t struct
// the 'simple' interface uses the simple vio_data_t struct from modal_vio_server_interface.h

#define QVIO_EXTENDED_NAME		"qvio_extended"
#define QVIO_EXTENDED_LOCATION	MODAL_PIPE_DEFAULT_BASE_DIR QVIO_EXTENDED_NAME "/"
#define QVIO_SIMPLE_NAME		"qvio"
#define QVIO_SIMPLE_LOCATION	MODAL_PIPE_DEFAULT_BASE_DIR QVIO_SIMPLE_NAME "/"


#define QVIO_OVERLAY_NAME		"qvio_overlay"
#define QVIO_OVERLAY_LOCATION	MODAL_PIPE_DEFAULT_BASE_DIR QVIO_OVERLAY_NAME "/"

/**
 * Unique 32-bit number used to signal the beginning of a data packet
 *
 * Spells QVIO in ASCII
 */
#define QVIO_MAGIC_NUMBER (0x5156494F)


// max number of features reported by the extended data struct. More features
// may be available. TODO output a full point cloud.
#define QVIO_MAX_REPORTED_FEATURES 32

typedef enum qvio_point_quality_t{
	LOW,     ///< additional low-quality points collected for e.g. collision avoidance
	MEDIUM,  ///< Points that are not "in state"
	HIGH     ///< Points that are "in state"
} qvio_point_quality_t;

typedef struct qvio_feature_t{

	uint32_t  id;				///< unique ID for feature point
	float pixLoc[2];		///< pixel location in the last frame
	float tsf[3];			///< location of feature in vio frame (relative to init location)
	float p_tsf[3][3];		///< covarience of feature location
	float depth;			///< distance from camera to point
	float depthErrorStdDev; ///< depth error in meters
	qvio_point_quality_t pointQuality;
} qvio_feature_t;


/*
 * data structure specific to the Qualcomm MVVISLAM VIO algorithm
 * This is a superset of vio_data_t from modal_vio_server_interface.h
 *
 * total bytes = 2764
 */
typedef struct qvio_data_t{
	uint32_t magic_number;      ///< Unique 32-bit number used to signal the beginning of a VIO packet while parsing a data stream.
	float quality;              ///< Quality is be >0 in normal use with a larger number indicating higher quality. A positive quality does not guarantee the algorithm has initialized completely.
	int64_t timestamp_ns;       ///< Timestamp in clock_monotonic system time of the provided pose.
	int32_t last_cam_frame_id;
	int64_t last_cam_timestamp_ns;
	float T_imu_wrt_vio[3];     ///< Translation of the IMU with respect to VIO frame in meters.
	float R_imu_to_vio[3][3];   ///< Rotation matrix from IMU to VIO frame.
	float T_ga_vio_wrt_cam[3];
	float R_ga_vio_to_cam[3][3];
	float pose_covariance[6][6];
	float imu_cam_time_shift_s;
	float vel_imu_wrt_vio[3];   ///< Velocity of the imu with respect to the VIO frame.
	float vel_covariance[3][3];
	float imu_angular_vel[3];   ///< Angular velocity of the IMU about its X Y and Z axes respectively. Essentially filtered gyro values with internal biases applied.
	float gravity_vector[3];    ///< Estimation of the current gravity vector in VIO frame. Use this to estimate the rotation between VIO frame and a gravity-aligned VIO frame if desired.
	float gravity_covariance[3][3];
	float gyro_bias[3];
	float accl_bias[3];
	float T_cam_wrt_imu[3];     ///< Location of the optical center of the camera with respect to the IMU.
	float R_cam_to_imu[3][3];   ///< Rotation matrix from camera frame to IMU frame.
	uint32_t error_code;        ///< bitmask that can indicate multiple errors. may still contain errors if state==VIO_STATE_OK
	uint16_t n_feature_points;  ///< Number of in-state visible feature points currently being tracked.
	uint8_t state;              ///< This is used to check the overall state of the algorithm. Can be VIO_STATE_FAILED, VIO_STATE_INITIALIZING, or VIO_STATE_OK.
	uint8_t n_total_features;   ///< total features, in-state and out-of-state listed in the following array
	qvio_feature_t features[QVIO_MAX_REPORTED_FEATURES];
} __attribute__((packed)) qvio_data_t;


/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync. Here we use 10 packets which is
 * perhaps more than necessary
 *
 * Note this is NOT the size of the pipe which can hold much more. This is just
 * the read buffer size allocated on the heap into which data from the pipe is
 * read.
 */
#define QVIO_RECOMMENDED_READ_BUF_SIZE	(sizeof(qvio_data_t) * 10)


// we recommend a 512k buffer that can hold 190 qvio packets
#define QVIO_RECOMMENDED_PIPE_SIZE		(512 * 1024)


/**
 * @brief      Use this to simultaneously validate that the bytes from a pipe
 *             contains valid data, find the number of valid packets
 *             contained in a single read from the pipe, and cast the raw data
 *             buffer as a qvio_data_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate a qvio_data_t array separate from the pipe read buffer.
 *             The data can be read straight out of the pipe read buffer, much
 *             like reading data directly out of a mavlink_message_t message.
 *
 *             However, this does mean the user should finish processing this
 *             data before returning the pipe data callback which triggers a new
 *             read() of the data pipe.
 *
 * @param[in]  data       pointer to pipe read data buffer
 * @param[in]  bytes      number of bytes read into that buffer
 * @param[out] n_packets  number of valid packets received
 *
 * @return     Returns the same data pointer provided by the first argument, but
 *             cast to an qvio_data_t* struct for convenience. If there was an
 *             error then NULL is returned and n_packets is set to 0
 */
static inline qvio_data_t* voxl_qvio_validate_pipe_data(char* data, int bytes, int* n_packets)
{
	// cast raw data from buffer to an vio_data_t array so we can read data
	// without memcpy. Also write out packets read as 0 until we validate data.
	qvio_data_t* new_ptr = (qvio_data_t*) data;
	*n_packets = 0;

	// basic sanity checks
	if(bytes<0){
		fprintf(stderr, "ERROR validating QVIO data received through pipe: number of bytes = %d\n", bytes);
		return NULL;
	}
	if(data==NULL){
		fprintf(stderr, "ERROR validating QVIO data received through pipe: got NULL data pointer\n");
		return NULL;
	}
	if(bytes%sizeof(qvio_data_t)){
		fprintf(stderr, "ERROR validating QVIO data received through pipe: read partial packet\n");
		fprintf(stderr, "read %d bytes, but it should be a multiple of %ld\n", bytes, sizeof(qvio_data_t));
		return NULL;
	}

	// calculate number of packets locally until we validate each packet
	int n_packets_tmp = bytes/sizeof(qvio_data_t);

	// check if any packets failed the magic number check
	int i, n_failed = 0;
	for(i=0;i<n_packets_tmp;i++){
		if(new_ptr[i].magic_number != QVIO_MAGIC_NUMBER){
			n_failed++;
			fprintf(stderr, "ERROR got VIO magic number %d, expected %d\n", new_ptr[i].magic_number, QVIO_MAGIC_NUMBER);
		}
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



#endif // VOXL_QVIO_SERVER_COMMON_H
