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
#include "utils/Statistics.h"

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
    uint8_t done;
    double T_imu_wrt_vio[3];     ///< Translation of the IMU with respect to VIO frame in meters, ordered x,y,z
    double q[4];                 ///< Quaternion with orientation data, ordered qx, qy, qz, qw
} ov_eval_data;

std::vector<double> times_gt;
std::vector<Eigen::Matrix<double, 7, 1>> poses_gt;

std::string alignment_type = "posyaw";
std::string path_to_gt = "";
std::string file_to_log = "";

static char pipe_path[MODAL_PIPE_MAX_PATH_LEN] = "/run/mpa/open-vins-eval";

bool en_debug = false;
bool en_timing = false;
bool live_align = false;

static int64_t log_start_ns = -1;

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
-l, --live_align            specify whether to align the trajectory to gt as we receive data (piece by piece)\n\
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
        {"file_to_log",	        required_argument,	0, 'f'},
        {"live_align",	    	no_argument,		0, 'l'},
        {"en_debug",	    	no_argument,		0, 'd'},
        {"en_timing",   	    no_argument,		0, 't'},
        {0, 0, 0, 0}
    };

    while(1){
        int option_index = 0;
        int c = getopt_long(argc, argv, "a:cdltf:v:g:", long_options, &option_index);

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

        case 'f':
            file_to_log.assign(optarg);
            break;

        case 'v':
            if(pipe_expand_location_string(optarg, pipe_path)<0){
                fprintf(stderr, "Invalid pipe name: %s\n", optarg);
                return -1;
            }
            break;

        case 'l':
            live_align = 1;
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

int write_results_to_csv(ov_eval::Statistics error_pos, std::map<double, std::pair<ov_eval::Statistics, ov_eval::Statistics>> error_rpe){
    FILE* file = fopen(file_to_log.c_str(), "wb");
    if (!file) {
        fprintf(stderr, "Failed to open log file %s.\n", file_to_log.c_str());
        return -1;
    }

    char ch;
    if(fscanf(file,"%c",&ch)==EOF){
        // if the file is empty, lets write out a header
        fprintf(file, "rmse_pos, std_pos, rpe_median_pos_seg0, rpe_median_pos_seg1, rpe_median_pos_seg2, rpe_median_pos_seg3, rpe_median_pos_seg4\n");
    }

    // write out only relevant data here
    fprintf(file, "%6.5f,%6.5f", error_pos.rmse, error_pos.std);
    for (const auto &seg : error_rpe) {
        fprintf(file, ",%6.5f", seg.second.second.median);
    }
    fprintf(file, "\n");
    fclose(file);
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


static void calc_ate(ov_eval::Statistics &error_ori, ov_eval::Statistics &error_pos, std::vector<Eigen::Matrix<double, 7, 1>> est_poses_aignedtoGT, std::vector<Eigen::Matrix<double, 7, 1>> gt_poses, std::vector<double> est_times) {

  // Clear any old data
  error_ori.clear();
  error_pos.clear();

  // Calculate the position and orientation error at every timestep
  for (size_t i = 0; i < est_poses_aignedtoGT.size(); i++) {

    // Calculate orientation error
    Eigen::Matrix3d e_R =
        ov_eval::Math::quat_2_Rot(est_poses_aignedtoGT.at(i).block(3, 0, 4, 1)).transpose() * ov_eval::Math::quat_2_Rot(gt_poses.at(i).block(3, 0, 4, 1));
    double ori_err = 180.0 / M_PI * ov_eval::Math::log_so3(e_R).norm();

    // Calculate position error
    double pos_err = (gt_poses.at(i).block(0, 0, 3, 1) - est_poses_aignedtoGT.at(i).block(0, 0, 3, 1)).norm();

    // Append this error!
    error_ori.timestamps.push_back(est_times.at(i));
    error_ori.values.push_back(ori_err);
    error_pos.timestamps.push_back(est_times.at(i));
    error_pos.values.push_back(pos_err);
  }

  // Update stat information
  error_ori.calculate();
  error_pos.calculate();
  return;
}

std::vector<int> compute_comparison_indices_length(std::vector<double> &distances, double distance, double max_dist_diff) {

    // Vector of end ids for our pose indexes
    std::vector<int> comparisons;

    // Loop through each pose in our trajectory (i.e. our distance vector generated from the trajectory).
    for (size_t idx = 0; idx < distances.size(); idx++) {

      // Loop through and find the pose that minimized the difference between
      // The desired trajectory distance and our current trajectory distance
      double distance_startpose = distances.at(idx);
      int best_idx = -1;
      double best_error = max_dist_diff;
      for (size_t i = idx; i < distances.size(); i++) {
        if (std::abs(distances.at(i) - (distance_startpose + distance)) < best_error) {
          best_idx = i;
          best_error = std::abs(distances.at(i) - (distance_startpose + distance));
        }
      }

      // If we have an end id that reached this trajectory distance then add it!
      // Else this isn't a valid segment, thus we shouldn't add it (we will try again at the next pose)
      // NOTE: just because we searched through all poses and didn't find a close one doesn't mean we have ended
      // NOTE: this could happen if there is a gap in the groundtruth poses and we just couldn't find a pose with low error
      comparisons.push_back(best_idx);
    }

    // Finally return the ids for each starting pose that have this distance
    return comparisons;
}

static void calc_rpe(const std::vector<double> &segment_lengths, std::map<double, std::pair<ov_eval::Statistics, ov_eval::Statistics>> &error_rpe, std::vector<Eigen::Matrix<double, 7, 1>> est_poses_aignedtoGT, std::vector<Eigen::Matrix<double, 7, 1>> gt_poses, std::vector<double> est_times) {

  // Distance at each point along the trajectory
  double average_pos_difference = 0;
  std::vector<double> accum_distances(gt_poses.size());
  accum_distances[0] = 0;
  for (size_t i = 1; i < gt_poses.size(); i++) {
    double pos_diff = (gt_poses[i].block(0, 0, 3, 1) - gt_poses[i - 1].block(0, 0, 3, 1)).norm();
    accum_distances[i] = accum_distances[i - 1] + pos_diff;
    average_pos_difference += pos_diff;
  }
  average_pos_difference /= gt_poses.size();

  // Warn if large pos difference
  double max_dist_diff = 0.5;
  if (average_pos_difference > max_dist_diff) {
    printf("[COMP]: average groundtruth position difference %.2f > %.2f is too large\n", average_pos_difference, max_dist_diff);
    printf("[COMP]: this will prevent the RPE from finding valid trajectory segments!!!\n");
    printf("[COMP]: the recommendation is to use a higher frequency groundtruth, or relax this trajectory segment logic...\n");
  }

  // Loop through each segment length
  for (const double &distance : segment_lengths) {

    // Our stats for this length
    ov_eval::Statistics error_ori, error_pos;

    // Get end of subtrajectories for each possible starting point
    // NOTE: is there a better way to select which end pos is a valid segments that are of the correct lenght?
    // NOTE: right now this allows for longer segments to have bigger error in their start-end distance vs the desired segment length
    // std::vector<int> comparisons = compute_comparison_indices_length(accum_distances, distance, 0.1*distance);
    std::vector<int> comparisons = compute_comparison_indices_length(accum_distances, distance, max_dist_diff);
    assert(comparisons.size() == gt_poses.size());

    // Loop through each relative comparison
    for (size_t id_start = 0; id_start < comparisons.size(); id_start++) {

      // Get the end id (skip if we couldn't find an end)
      int id_end = comparisons[id_start];
      if (id_end == -1)
        continue;

      //===============================================================================
      // Get T I1 to world EST at beginning of subtrajectory (at state idx)
      Eigen::Matrix4d T_c1 = Eigen::Matrix4d::Identity();
      T_c1.block(0, 0, 3, 3) = ov_eval::Math::quat_2_Rot(est_poses_aignedtoGT.at(id_start).block(3, 0, 4, 1)).transpose();
      T_c1.block(0, 3, 3, 1) = est_poses_aignedtoGT.at(id_start).block(0, 0, 3, 1);

      // Get T I2 to world EST at end of subtrajectory starting (at state comparisons[idx])
      Eigen::Matrix4d T_c2 = Eigen::Matrix4d::Identity();
      T_c2.block(0, 0, 3, 3) = ov_eval::Math::quat_2_Rot(est_poses_aignedtoGT.at(id_end).block(3, 0, 4, 1)).transpose();
      T_c2.block(0, 3, 3, 1) = est_poses_aignedtoGT.at(id_end).block(0, 0, 3, 1);

      // Get T I2 to I1 EST
      Eigen::Matrix4d T_c1_c2 = ov_eval::Math::Inv_se3(T_c1) * T_c2;

      //===============================================================================
      // Get T I1 to world GT at beginning of subtrajectory (at state idx)
      Eigen::Matrix4d T_m1 = Eigen::Matrix4d::Identity();
      T_m1.block(0, 0, 3, 3) = ov_eval::Math::quat_2_Rot(gt_poses.at(id_start).block(3, 0, 4, 1)).transpose();
      T_m1.block(0, 3, 3, 1) = gt_poses.at(id_start).block(0, 0, 3, 1);

      // Get T I2 to world GT at end of subtrajectory starting (at state comparisons[idx])
      Eigen::Matrix4d T_m2 = Eigen::Matrix4d::Identity();
      T_m2.block(0, 0, 3, 3) = ov_eval::Math::quat_2_Rot(gt_poses.at(id_end).block(3, 0, 4, 1)).transpose();
      T_m2.block(0, 3, 3, 1) = gt_poses.at(id_end).block(0, 0, 3, 1);

      // Get T I2 to I1 GT
      Eigen::Matrix4d T_m1_m2 = ov_eval::Math::Inv_se3(T_m1) * T_m2;

      //===============================================================================
      // Compute error transform between EST and GT start-end transform
      Eigen::Matrix4d T_error_in_c2 = ov_eval::Math::Inv_se3(T_m1_m2) * T_c1_c2;

      Eigen::Matrix4d T_c2_rot = Eigen::Matrix4d::Identity();
      T_c2_rot.block(0, 0, 3, 3) = T_c2.block(0, 0, 3, 3);

      Eigen::Matrix4d T_c2_rot_inv = Eigen::Matrix4d::Identity();
      T_c2_rot_inv.block(0, 0, 3, 3) = T_c2.block(0, 0, 3, 3).transpose();

      // Rotate rotation so that rotation error is in the global frame (allows us to look at yaw error)
      Eigen::Matrix4d T_error_in_w = T_c2_rot * T_error_in_c2 * T_c2_rot_inv;

      //===============================================================================
      // Compute error for position
      error_pos.timestamps.push_back(est_times.at(id_start));
      error_pos.values.push_back(T_error_in_w.block(0, 3, 3, 1).norm());

      // Calculate orientation error
      double ori_err = 180.0 / M_PI * ov_eval::Math::log_so3(T_error_in_w.block(0, 0, 3, 3)).norm();
      error_ori.timestamps.push_back(est_times.at(id_start));
      error_ori.values.push_back(ori_err);
    }

    // Update stat information
    error_ori.calculate();
    error_pos.calculate();
    error_rpe.insert({distance, {error_ori, error_pos}});
  }
}

// this is the helper that will load in the entire trajectory from a running algo 
// not sure how we can detect that it has finished though...
// will take a trajectory from some running algo and align it to the ground truth while we create it
static void _load_and_align_helper_cb( __attribute__((unused)) int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
    static std::vector<ov_eval_data> queue_of_packets;
    static bool ready_to_process = false;

    // validate that the data makes sense
    int n_packets;
    ov_eval_data* data_array = pipe_validate_ov_eval_data_t(data, bytes, &n_packets);
    if(data_array == NULL) return;

    static int64_t first_data_ts_ns = data_array[0].timestamp_ns;
    static double offset = -(first_data_ts_ns - log_start_ns)/1e9;

    for (int i = 0; i < n_packets; i++){
        if (data_array[i].done){
            ready_to_process = true;
            break;
        }
        queue_of_packets.push_back(data_array[i]);
    }

    if (queue_of_packets.size() % 50 == 0) fprintf(stderr, "collected %d samples\n", (int)queue_of_packets.size());

    if (!ready_to_process || queue_of_packets.size() < 3) return;

    fprintf(stderr, "Log complete. Starting to perform calculations.\n");

    std::vector<double> times_temp;
    std::vector<Eigen::Matrix<double, 7, 1>> poses_temp;

    for (int i = 0; i < (int)queue_of_packets.size(); i++){
        times_temp.push_back(queue_of_packets[i].timestamp_ns/1000000000.0);  // convert it to seconds
        Eigen::Matrix<double, 7, 1> pose_tmp;
        pose_tmp << queue_of_packets[i].T_imu_wrt_vio[0], queue_of_packets[i].T_imu_wrt_vio[1], queue_of_packets[i].T_imu_wrt_vio[2],
                    queue_of_packets[i].q[0], queue_of_packets[i].q[1], queue_of_packets[i].q[2], queue_of_packets[i].q[4];
        poses_temp.push_back(pose_tmp);
    }
    queue_of_packets.clear();

    // Intersect timestamps
    std::vector<double> gt_times_temp = times_gt;
    std::vector<Eigen::Matrix<double, 7, 1>> gt_poses_temp = poses_gt;
    double allowed_variance = 0.02;

    ov_eval::AlignUtils::perform_association(offset, allowed_variance, times_temp, gt_times_temp, poses_temp, gt_poses_temp);

    // Return failure if we didn't have any common timestamps
    if (poses_temp.size() < MIN_PACKETS_FOR_ALIGNMENT) {
        fprintf(stderr, "[TRAJ]: unable to get enough common timestamps between trajectories.\n");
        fprintf(stderr, "[TRAJ]: Need at least %d, got %d.\n", MIN_PACKETS_FOR_ALIGNMENT, (int)poses_temp.size());
        return;
    }

    // Perform alignment of the trajectories
    Eigen::Matrix3d R_ESTtoGT, R_GTtoEST;
    Eigen::Vector3d t_ESTinGT, t_GTinEST;
    double s_ESTtoGT, s_GTtoEST;
    ov_eval::AlignTrajectory::align_trajectory(poses_temp, gt_poses_temp, R_ESTtoGT, t_ESTinGT, s_ESTtoGT, alignment_type);
    ov_eval::AlignTrajectory::align_trajectory(gt_poses_temp, poses_temp, R_GTtoEST, t_GTinEST, s_GTtoEST, alignment_type);

    // Debug print to the user
    Eigen::Vector4d q_ESTtoGT = ov_eval::Math::rot_2_quat(R_ESTtoGT);
    Eigen::Vector4d q_GTtoEST = ov_eval::Math::rot_2_quat(R_GTtoEST);
    fprintf(stderr, "[TRAJ]: q_ESTtoGT = %.3f, %.3f, %.3f, %.3f | p_ESTinGT = %.3f, %.3f, %.3f | s = %.2f\n", q_ESTtoGT(0), q_ESTtoGT(1),
            q_ESTtoGT(2), q_ESTtoGT(3), t_ESTinGT(0), t_ESTinGT(1), t_ESTinGT(2), s_ESTtoGT);

    // Aligned trajectories
    std::vector<Eigen::Matrix<double, 7, 1>> est_poses_aignedtoGT;
    std::vector<Eigen::Matrix<double, 7, 1>> gt_poses_aignedtoEST;

    // Finally lets calculate the aligned trajectories
    for (size_t i = 0; i < poses_temp.size(); i++) {
        Eigen::Matrix<double, 7, 1> pose_ESTinGT, pose_GTinEST;
        pose_ESTinGT.block(0, 0, 3, 1) = s_ESTtoGT * R_ESTtoGT * poses_temp.at(i).block(0, 0, 3, 1) + t_ESTinGT;
        pose_ESTinGT.block(3, 0, 4, 1) = ov_eval::Math::quat_multiply(poses_temp.at(i).block(3, 0, 4, 1), ov_eval::Math::Inv(q_ESTtoGT));
        pose_GTinEST.block(0, 0, 3, 1) = s_GTtoEST * R_GTtoEST * gt_poses_temp.at(i).block(0, 0, 3, 1) + t_GTinEST;
        pose_GTinEST.block(3, 0, 4, 1) = ov_eval::Math::quat_multiply(gt_poses_temp.at(i).block(3, 0, 4, 1), ov_eval::Math::Inv(q_GTtoEST));
        est_poses_aignedtoGT.push_back(pose_ESTinGT);
        gt_poses_aignedtoEST.push_back(pose_GTinEST);
    }

    ov_eval::Statistics error_ori, error_pos;
    calc_ate(error_ori, error_pos, est_poses_aignedtoGT, gt_poses_temp, times_temp);

    printf("======================================\n");
    printf("Absolute Trajectory Error\n");
    printf("======================================\n");
    printf("rmse_ori = %.3f | rmse_pos = %.3f\n", error_ori.rmse, error_pos.rmse);
    printf("mean_ori = %.3f | mean_pos = %.3f\n", error_ori.mean, error_pos.mean);
    printf("min_ori  = %.3f | min_pos  = %.3f\n", error_ori.min, error_pos.min);
    printf("max_ori  = %.3f | max_pos  = %.3f\n", error_ori.max, error_pos.max);
    printf("std_ori  = %.3f | std_pos  = %.3f\n", error_ori.std, error_pos.std);

    // distances here. defaults are much larger for large ass rosbags
    std::vector<double> segments = {3.0, 6.0, 9.0, 12.0, 15.0};
    std::map<double, std::pair<ov_eval::Statistics, ov_eval::Statistics>> error_rpe;

    calc_rpe(segments, error_rpe, est_poses_aignedtoGT, gt_poses_temp, times_temp);

    printf("======================================\n");
    printf("Relative Pose Error\n");
    printf("======================================\n");
    for (const auto &seg : error_rpe) {
        printf("seg %d - median_ori = %.3f | median_pos = %.3f (%d samples)\n", (int)seg.first, seg.second.first.median,
                    seg.second.second.median, (int)seg.second.second.values.size());
    }

    // save em
    if (write_results_to_csv(error_pos, error_rpe)<0) fprintf(stderr, "failed to save error\n");


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

        Eigen::Matrix<double, 7, 1> pose_ESTinGT = est_poses_aignedtoGT.at(i);
        Eigen::Matrix<double, 3, 3> r_ESTtoGT = ov_eval::Math::quat_2_Rot(pose_ESTinGT.block(3, 0, 4, 1));

        // Finally push back
        vio_data_t aligned_packet;
        aligned_packet.timestamp_ns = timestamp;
        aligned_packet.magic_number = VIO_MAGIC_NUMBER;
        aligned_packet.T_imu_wrt_vio[0] = pose_ESTinGT(0);
        aligned_packet.T_imu_wrt_vio[1] = pose_ESTinGT(1);
        aligned_packet.T_imu_wrt_vio[2] = pose_ESTinGT(2);

        for (int j = 0; j < 3; j++){
            for (int k = 0; k < 3; k++){
                aligned_packet.R_imu_to_vio[j][k] = r_ESTtoGT(j, k);
            }
        }
        pipe_server_write(GT_OUTPUT_CH, (char*)&gt_packet, sizeof(vio_data_t));
        usleep(100000);
        pipe_server_write(ALIGNED_OUTPUT_CH, (char*)&aligned_packet, sizeof(vio_data_t));
    }

    return;
}

// this is the "live aligner" function
// will take a trajectory from some running algo and align it to the ground truth while we create it
// will only calculate absolute error for each aligned chunk
static void _live_align_helper_cb( __attribute__((unused)) int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
    static std::vector<ov_eval_data> queue_of_packets;

    // validate that the data makes sense
    int n_packets;
    ov_eval_data* data_array = pipe_validate_ov_eval_data_t(data, bytes, &n_packets);
    if(data_array == NULL) return;

    static int64_t first_data_ts_ns = data_array[0].timestamp_ns;
    static double offset = -(first_data_ts_ns - log_start_ns)/1e9;

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
    if (poses_temp.size() < MIN_PACKETS_FOR_ALIGNMENT) {
        fprintf(stderr, "[TRAJ]: unable to get enough common timestamps between trajectories.\n");
        fprintf(stderr, "[TRAJ]: Need at least %d, got %d.\n", MIN_PACKETS_FOR_ALIGNMENT, (int)poses_temp.size());
        return;
    }

    // Perform alignment of the trajectories
    Eigen::Matrix3d R_ESTtoGT, R_GTtoEST;
    Eigen::Vector3d t_ESTinGT, t_GTinEST;
    double s_ESTtoGT, s_GTtoEST;
    ov_eval::AlignTrajectory::align_trajectory(poses_temp, gt_poses_temp, R_ESTtoGT, t_ESTinGT, s_ESTtoGT, alignment_type);
    ov_eval::AlignTrajectory::align_trajectory(gt_poses_temp, poses_temp, R_GTtoEST, t_GTinEST, s_GTtoEST, alignment_type);

    // Debug print to the user
    Eigen::Vector4d q_ESTtoGT = ov_eval::Math::rot_2_quat(R_ESTtoGT);
    Eigen::Vector4d q_GTtoEST = ov_eval::Math::rot_2_quat(R_GTtoEST);
    fprintf(stderr, "[TRAJ]: q_ESTtoGT = %.3f, %.3f, %.3f, %.3f | p_ESTinGT = %.3f, %.3f, %.3f | s = %.2f\n", q_ESTtoGT(0), q_ESTtoGT(1),
            q_ESTtoGT(2), q_ESTtoGT(3), t_ESTinGT(0), t_ESTinGT(1), t_ESTinGT(2), s_ESTtoGT);

    // Aligned trajectories
    std::vector<Eigen::Matrix<double, 7, 1>> est_poses_aignedtoGT;
    std::vector<Eigen::Matrix<double, 7, 1>> gt_poses_aignedtoEST;

    // Finally lets calculate the aligned trajectories
    for (size_t i = 0; i < poses_temp.size(); i++) {
        Eigen::Matrix<double, 7, 1> pose_ESTinGT, pose_GTinEST;
        pose_ESTinGT.block(0, 0, 3, 1) = s_ESTtoGT * R_ESTtoGT * poses_temp.at(i).block(0, 0, 3, 1) + t_ESTinGT;
        pose_ESTinGT.block(3, 0, 4, 1) = ov_eval::Math::quat_multiply(poses_temp.at(i).block(3, 0, 4, 1), ov_eval::Math::Inv(q_ESTtoGT));
        pose_GTinEST.block(0, 0, 3, 1) = s_GTtoEST * R_GTtoEST * gt_poses_temp.at(i).block(0, 0, 3, 1) + t_GTinEST;
        pose_GTinEST.block(3, 0, 4, 1) = ov_eval::Math::quat_multiply(gt_poses_temp.at(i).block(3, 0, 4, 1), ov_eval::Math::Inv(q_GTtoEST));
        est_poses_aignedtoGT.push_back(pose_ESTinGT);
        gt_poses_aignedtoEST.push_back(pose_GTinEST);
    }

    ov_eval::Statistics error_ori, error_pos;
    calc_ate(error_ori, error_pos, est_poses_aignedtoGT, gt_poses_temp, times_temp);

    printf("======================================\n");
    printf("Absolute Trajectory Error\n");
    printf("======================================\n");
    printf("rmse_ori = %.3f | rmse_pos = %.3f\n", error_ori.rmse, error_pos.rmse);
    printf("mean_ori = %.3f | mean_pos = %.3f\n", error_ori.mean, error_pos.mean);
    printf("min_ori  = %.3f | min_pos  = %.3f\n", error_ori.min, error_pos.min);
    printf("max_ori  = %.3f | max_pos  = %.3f\n", error_ori.max, error_pos.max);
    printf("std_ori  = %.3f | std_pos  = %.3f\n", error_ori.std, error_pos.std);

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

        Eigen::Matrix<double, 7, 1> pose_ESTinGT = est_poses_aignedtoGT.at(i);
        Eigen::Matrix<double, 3, 3> r_ESTtoGT = ov_eval::Math::quat_2_Rot(pose_ESTinGT.block(3, 0, 4, 1));

        // Finally push back
        vio_data_t aligned_packet;
        aligned_packet.timestamp_ns = timestamp;
        aligned_packet.magic_number = VIO_MAGIC_NUMBER;
        aligned_packet.T_imu_wrt_vio[0] = pose_ESTinGT(0);
        aligned_packet.T_imu_wrt_vio[1] = pose_ESTinGT(1);
        aligned_packet.T_imu_wrt_vio[2] = pose_ESTinGT(2);

        for (int j = 0; j < 3; j++){
            for (int k = 0; k < 3; k++){
                aligned_packet.R_imu_to_vio[j][k] = r_ESTtoGT(j, k);
            }
        }
        pipe_server_write(GT_OUTPUT_CH, (char*)&gt_packet, sizeof(vio_data_t));
        usleep(100000);
        pipe_server_write(ALIGNED_OUTPUT_CH, (char*)&aligned_packet, sizeof(vio_data_t));
    }
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
        fprintf(stderr, "ERROR: User needs to provide ground truth file path, following -g when running this program\n");
        return -1;
    }

    if (file_to_log == "" && !live_align){
        fprintf(stderr, "WARNING: User should provide log file path, following -f, otherwise info will only be dumped to console\n");
    }

    boost::filesystem::path infolder(path_to_gt);
    if (infolder.extension() == ".csv") {
        std::vector<Eigen::Matrix3d> cov_ori_temp, cov_pos_temp;
        log_start_ns = ov_eval::Loader::load_data_csv(path_to_gt, times_gt, poses_gt, cov_ori_temp, cov_pos_temp);
    } else {
        fprintf(stderr, "WARNING: This format expects times to be in seconds. Make sure your log method is correct.\n");
        std::vector<Eigen::Matrix3d> cov_ori_temp, cov_pos_temp;
        ov_eval::Loader::load_data(path_to_gt, times_gt, poses_gt, cov_ori_temp, cov_pos_temp);
    }

    if (create_server_pipes()){
        fprintf(stderr, "ERROR: Unable to create server pipes!\n");
        return -1;
    }

	// set some basic signal handling for safe shutdown.
	// quitting without cleanup up the pipe can result in the pipe staying
	// open and overflowing, so always cleanup properly!!!
	enable_signal_handler();
	main_running = 1;

	// set up all our MPA callbacks
	pipe_client_set_simple_helper_cb(0, live_align ? _live_align_helper_cb : _load_and_align_helper_cb, NULL);
	pipe_client_set_connect_cb(0, _connect_cb, NULL);
	pipe_client_set_disconnect_cb(0, _disconnect_cb, NULL);

	// request a new pipe from the server
	printf("waiting for server at %s\n", pipe_path);
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