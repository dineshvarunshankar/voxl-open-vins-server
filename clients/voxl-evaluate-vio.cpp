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
#include "alignment/AlignTrajectory.h"
#include "alignment/AlignUtils.h"
#include "utils/Colors.h"
#include "utils/Loader.h"
#include "utils/Math.h"
// #include "utils/print.h"

#include <stdio.h>
#include <signal.h>
#include <getopt.h>
#include <unistd.h>	// for usleep()
#include <string.h>
#include <stdlib.h> // for atoi()
#include <math.h>

#include <modal_pipe_client.h>
#include <modal_start_stop.h>
#include <modal_pipe_server.h>

#include <boost/filesystem.hpp>

#define PROCESS_NAME		"voxl-evaluate-vio"

#define GT_OUTPUT_CH   0
#define GT_OUTPUT_NAME "gt-ov-eval"
#define GT_OUTPUT_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR GT_OUTPUT_NAME "/"

#define ALIGNED_OUTPUT_CH   1
#define ALIGNED_OUTPUT_NAME "aligned-ov-eval"
#define ALIGNED_OUTPUT_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR ALIGNED_OUTPUT_NAME "/"

// this is our struct for the ov_eval expected data
typedef struct ov_eval_data {
    int64_t timestamp_ns;
    uint32_t magic_number;       ///< Unique 32-bit number used to signal the beginning of a VIO packet while parsing a data stream.
    double T_imu_wrt_vio[3];     ///< Translation of the IMU with respect to VIO frame in meters, ordered x,y,z
    double q[4];                 ///< Quaternion with orientation data, ordered qx, qy, qz, qw
} ov_eval_data;

std::vector<double> times_gt;
std::vector<Eigen::Matrix<double, 7, 1>> poses_gt;

std::string alignment_type = "posyaw";
std::string path_to_gt = "";

static char pipe_path[MODAL_PIPE_MAX_PATH_LEN] = "/run/mpa/open-vins-eval";

bool en_debug = false;
bool en_timing = false;

static int64_t log_start_ns = 133333572249;

static void _print_usage(void)
{
    printf("\n\
typical usage\n\
/# voxl-evaluate-vio\n\
\n\
This will live align a running trajectory to a ground truth file.\n\
\n\
Alignment type will default to posyaw. Required options are:\n\
-v, --vio_pipe              specify running vio pipe name\n\
-g, --ground_truth          path to ground truth file (txt or csv)\n\
-a, --aligment_type         type of alignment, can be posyaw, posyawsingle, se3, se3single\n\
Additional options are:\n\
-d, --en_debug              print debug messages\n\
-t, --en_timing             print timing messages\n\
\n");
    return;
}

static int _parse_opts(int argc, char* argv[])
{
    static struct option long_options[] =
    {
        {"vio_pipe",		    required_argument,	0, 'v'},
        {"ground_truth",		required_argument,	0, 'g'},
        {"aligment_type",	    required_argument,	0, 'a'},
        {"en_debug",	    	no_argument,		0, 'd'},
        {"en_timing",   	    no_argument,		0, 't'},
        {0, 0, 0, 0}
    };

    while(1){
        int option_index = 0;
        int c = getopt_long(argc, argv, "a:cdtv:g:", long_options, &option_index);

        if(c == -1) break; // Detect the end of the options.

        switch(c){
        case 0:
            // for long args without short equivalent that just set a flag
            // nothing left to do so just break.
            if (long_options[option_index].flag != 0) break;
            break;

        case 'a':
            alignment_type.assign(optarg);
            break;

        case 'g':
            path_to_gt.assign(optarg);
            break;

        case 'v':
            if(pipe_expand_location_string(optarg, pipe_path)<0){
                fprintf(stderr, "Invalid pipe name: %s\n", optarg);
                return -1;
            }
            break;

        case 'd':
            en_debug = 1;
            break;

        case 't':
            en_timing = 1;
            break;

        default:
            _print_usage();
            return -1;
        }
    }

    return 0;
}

static int create_server_pipes(void) {
    int flags = 0;

    pipe_info_t info =
    {
        ALIGNED_OUTPUT_NAME,            // name
        ALIGNED_OUTPUT_LOCATION,        // location
        "vio_data_t",               // type
        PROCESS_NAME,               // server_name
        VIO_RECOMMENDED_PIPE_SIZE,  // size_bytes
        0                           // server_pid
    };

    if (pipe_server_create(ALIGNED_OUTPUT_CH, info, flags)) {
        return -1;
    }

    pipe_info_t info2 =
    {
        GT_OUTPUT_NAME,            // name
        GT_OUTPUT_LOCATION,        // location
        "vio_data_t",               // type
        PROCESS_NAME,               // server_name
        VIO_RECOMMENDED_PIPE_SIZE,  // size_bytes
        0                           // server_pid
    };

    if (pipe_server_create(GT_OUTPUT_CH, info2, flags)) {
        return -1;
    }

    return 0;
}

ov_eval_data* pipe_validate_ov_eval_data_t(char* data, int bytes, int* n_packets)
{
    // cast raw data from buffer to an vio_data_t array so we can read data
    // without memcpy. Also write out packets read as 0 until we validate data.
    ov_eval_data* new_ptr = (ov_eval_data*) data;
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
    if(bytes%sizeof(ov_eval_data)){
        fprintf(stderr, "ERROR validating VIO data received through pipe: read partial packet\n");
        fprintf(stderr, "read %d bytes, but it should be a multiple of %zu\n", bytes, sizeof(ov_eval_data));
        return NULL;
    }

    // calculate number of packets locally until we validate each packet
    int n_packets_tmp = bytes/sizeof(ov_eval_data);

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
    static std::vector<ov_eval_data> queue_of_packets;

    // static int64_t log_start_ns = times_gt[0]*1e9;
    fprintf(stderr, "log start time: %ld\n", log_start_ns);

    // validate that the data makes sense
    int n_packets;
    ov_eval_data* data_array = pipe_validate_ov_eval_data_t(data, bytes, &n_packets);
    if(data_array == NULL) return;

    static int64_t first_data_ts_ns = data_array[0].timestamp_ns;
    static double offset = -(first_data_ts_ns - log_start_ns)/1e9;
    fprintf(stderr, "offset: %6.5f\n", offset);

    for (int i = 0; i < n_packets; i++){
        queue_of_packets.push_back(data_array[i]);
    }

    if (queue_of_packets.size() < 3) return;

    std::vector<double> times_temp;
    std::vector<Eigen::Matrix<double, 7, 1>> poses_temp;

    // now, instead of printing the data, we need to process it
    for (int i = 0; i < (int)queue_of_packets.size(); i++){
        times_temp.push_back(queue_of_packets[i].timestamp_ns/1000000000.0);  // convert it to seconds
        Eigen::Matrix<double, 7, 1> pose_tmp;
        pose_tmp << queue_of_packets[i].T_imu_wrt_vio[0], queue_of_packets[i].T_imu_wrt_vio[1], queue_of_packets[i].T_imu_wrt_vio[2],
                    queue_of_packets[i].q[0], queue_of_packets[i].q[1], queue_of_packets[i].q[2], queue_of_packets[i].q[4];
        poses_temp.push_back(pose_tmp);
    }

    // Intersect timestamps
    std::vector<double> gt_times_temp = times_gt;
    std::vector<Eigen::Matrix<double, 7, 1>> gt_poses_temp = poses_gt;
    double allowed_variance = 0.02;

    ov_eval::AlignUtils::perform_association(offset, allowed_variance, times_temp, gt_times_temp, poses_temp, gt_poses_temp);

    // Return failure if we didn't have any common timestamps
    // n_packets was just straight up 3 before, not really sure why
    if (poses_temp.size() < 3) {
        fprintf(stderr, "[TRAJ]: unable to get enough common timestamps between trajectories.\n");
        fprintf(stderr, "[TRAJ]: Need at least 3, got %d.\n", (int)poses_temp.size());
        return;
    }

    // Perform alignment of the trajectories
    Eigen::Matrix3d R_ESTtoGT;
    Eigen::Vector3d t_ESTinGT;
    double s_ESTtoGT;
    ov_eval::AlignTrajectory::align_trajectory(poses_temp, gt_poses_temp, R_ESTtoGT, t_ESTinGT, s_ESTtoGT, alignment_type);
    Eigen::Vector4d q_ESTtoGT = ov_eval::Math::rot_2_quat(R_ESTtoGT);
    fprintf(stderr, "[TRAJ]: q_ESTtoGT = %.3f, %.3f, %.3f, %.3f | p_ESTinGT = %.3f, %.3f, %.3f | s = %.2f\n", q_ESTtoGT(0), q_ESTtoGT(1),
                q_ESTtoGT(2), q_ESTtoGT(3), t_ESTinGT(0), t_ESTinGT(1), t_ESTinGT(2), s_ESTtoGT);

    for (size_t i = 0; i < gt_times_temp.size(); i += std::floor(gt_times_temp.size() / 16384.0) + 1) {

        // Convert into the correct frame
        int64_t timestamp = gt_times_temp.at(i) * 1e9;
        Eigen::Matrix<double, 7, 1> pose_inGT = gt_poses_temp.at(i);

        // we just need to publish the gt from above, same quat to rot function will be needed
        vio_data_t gt_packet;
        gt_packet.timestamp_ns = timestamp;
        gt_packet.magic_number = VIO_MAGIC_NUMBER;
        gt_packet.T_imu_wrt_vio[0] = pose_inGT(0);
        gt_packet.T_imu_wrt_vio[1] = pose_inGT(1);
        gt_packet.T_imu_wrt_vio[2] = pose_inGT(2);

        Eigen::Matrix<double, 3, 3> r_GT = ov_eval::Math::quat_2_Rot(pose_inGT.block(3, 0, 4, 1));

        for (int j = 0; j < 3; j++){
            for (int k = 0; k < 3; k++){
                gt_packet.R_imu_to_vio[j][k] = r_GT(j, k);
            }
        }
        
        Eigen::Vector3d pos_IinEST = R_ESTtoGT.transpose() * (pose_inGT.block(0, 0, 3, 1) - t_ESTinGT) / s_ESTtoGT;
        Eigen::Vector4d quat_ESTtoI = ov_eval::Math::quat_multiply(pose_inGT.block(3, 0, 4, 1), q_ESTtoGT);
        Eigen::Matrix<double, 3, 3> r_ESTtoGT = ov_eval::Math::quat_2_Rot(quat_ESTtoI);


        // Finally push back
        vio_data_t aligned_packet;
        aligned_packet.timestamp_ns = timestamp;
        aligned_packet.magic_number = VIO_MAGIC_NUMBER;
        aligned_packet.T_imu_wrt_vio[0] = pos_IinEST(0);
        aligned_packet.T_imu_wrt_vio[1] = pos_IinEST(1);
        aligned_packet.T_imu_wrt_vio[2] = pos_IinEST(2);

        for (int j = 0; j < 3; j++){
            for (int k = 0; k < 3; k++){
                aligned_packet.R_imu_to_vio[j][k] = r_ESTtoGT(j, k);
            }
        }
        pipe_server_write(GT_OUTPUT_CH, (char*)&gt_packet, sizeof(vio_data_t));
        pipe_server_write(ALIGNED_OUTPUT_CH, (char*)&aligned_packet, sizeof(vio_data_t));
    }

    // not sure if we want to align it bit by bit (clear every few packets), or do the whole thing
    queue_of_packets.clear();

    return;
}

// connect and disconnect callbacks
static void _connect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
	fprintf(stderr, "\nconnected to vio server\n");
	return;
}

static void _disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
	fprintf(stderr, "\nvio server disconnected\n");
	return;
}


int main(int argc, char* argv[])
{
	// check for options
	if(_parse_opts(argc, argv)) return -1;

    // check if ground truth path is empty
    if (path_to_gt == ""){
        fprintf(stderr, "User needs to provide ground truth file path, following -g when running this program\n");
        return -1;
    }

    boost::filesystem::path infolder(path_to_gt);
    if (infolder.extension() == ".csv") {
        std::vector<Eigen::Matrix3d> cov_ori_temp, cov_pos_temp;
        ov_eval::Loader::load_data_csv(path_to_gt, times_gt, poses_gt, cov_ori_temp, cov_pos_temp);
    } else {
        std::vector<Eigen::Matrix3d> cov_ori_temp, cov_pos_temp;
        ov_eval::Loader::load_data(path_to_gt, times_gt, poses_gt, cov_ori_temp, cov_pos_temp);
    }

    if (create_server_pipes()){
        fprintf(stderr, "Unable to create server pipes!\n");
        return -1;
    }

	// set some basic signal handling for safe shutdown.
	// quitting without cleanup up the pipe can result in the pipe staying
	// open and overflowing, so always cleanup properly!!!
	enable_signal_handler();
	main_running = 1;

	// set up all our MPA callbacks
	pipe_client_set_simple_helper_cb(0, _helper_cb, NULL);
	pipe_client_set_connect_cb(0, _connect_cb, NULL);
	pipe_client_set_disconnect_cb(0, _disconnect_cb, NULL);

	// request a new pipe from the server
	printf("waiting for server\n");
	int ret = pipe_client_open(0, pipe_path, PROCESS_NAME, \
				EN_PIPE_CLIENT_SIMPLE_HELPER, \
				VIO_RECOMMENDED_READ_BUF_SIZE);

	// check for MPA errors
	if(ret<0){
		pipe_print_error(ret);
		return -1;
	}

	// keep going until signal handler sets the running flag to 0
	while(main_running) usleep(200000);

	// all done, signal pipe read threads to stop
	printf("\nclosing and exiting\n");
	pipe_client_close_all();

	return 0;
}