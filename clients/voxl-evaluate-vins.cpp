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


/*
this file is going to handle three services:
voxl-replay
voxl-feature-tracker
voxl-open-vins-server

given a log path, we will create a date stamped directory for the test on a singular log

within this directory, we will create a new folder each time params are adjusted
these will just be enumerated

then within that folder, we save a folder of params, conf files etc
and a csv logging output

csv will contain:
duration of uptime for that run
end point

we will run until we receive some kind of failure code from open-vins-output

then, record this output
then we kill all of our services, starting at the top of the chain

decide to make parameter adjustment here?
brute force for now, take a list of params and pre compute a few hundred iterations for their possible combos
*/

#include <stdio.h>
#include <signal.h>
#include <getopt.h>
#include <unistd.h>	// for usleep()
#include <string>
#include <string.h>
#include <stdlib.h> // for atoi()
#include <math.h>
#include <sys/stat.h>	// for mkdir
#include <unistd.h>		// for access()
#include <limits.h>		// for access()
#include <vector>
#include <map>
#include <iostream>


#include <modal_pipe_client.h>
#include <modal_start_stop.h>
#include <modal_json.h>



#define CLIENT_NAME		"voxl-evaluate-vins"
#define OV_NAME         "ov"
#define OUTPUT_DIR      "/data/open-vins/"

#define OV_CONFIG_FILE "/etc/modalai/voxl-open-vins-server.conf"
#define FT_CONFIG_FILE "/etc/modalai/voxl-feature-tracker.conf"

static std::string log_path;

static int64_t last_timestamp_ns = 0;
static int64_t first_timestamp_ns = 0;
static bool is_first_set = false;
static float last_pose[3] = {0};

static bool failure = false;

static pid_t sub_pid_list[3] = {0};

char process_paths[3][256] = { 
    "/usr/bin/voxl-replay", 
    "/usr/bin/voxl-feature-tracker", 
    "/usr/local/bin/voxl-open-vins-server"
};

char process_args[3][256] = {
    "", 
    "",
    ""
};

std::map<std::string, std::vector<int>> param_table;
static const int n_ov_params = 13;
static const int n_ft_params = 6;

static int n_total_params = 0;

static std::string session_dir;



static void _print_usage(void)
{
	printf("\n\
typical usage\n\
/# voxl-evaluate-vins\n\
\n\
-l, --log_path              path to log we are evaluating\n\
-h, --help                  print this help message\n\
\n");
	return;
}


static int _parse_opts(int argc, char* argv[])
{
	static struct option long_options[] =
	{
		{"log_path",		    required_argument,	0, 'l'},
		{"help",				no_argument,		0, 'h'},
		{0, 0, 0, 0}
	};

	while(1){
		int option_index = 0;
		int c = getopt_long(argc, argv, "l:h", long_options, &option_index);

		if(c == -1) break; // Detect the end of the options.

		switch(c){
		case 0:
			// for long args without short equivalent that just set a flag
			// nothing left to do so just break.
			if (long_options[option_index].flag != 0) break;
			break;

		case 'l':
            log_path.assign(optarg);
            printf("using log path of %s\n", log_path.c_str());
            snprintf(process_args[0], 256, "-yp%s", log_path.c_str());
			break;

		case 'h':
			_print_usage();
            exit(-1);

		default:
            fprintf(stderr, "ERROR: unsupported arg provided\n");
			_print_usage();
			exit(-1);
		}
	}
	return 0;
}


static int _mkdir(const char *dir)
{
	char tmp[PATH_MAX];
	char* p = NULL;

	snprintf(tmp, sizeof(tmp),"%s",dir);
	for(p = tmp + 1; *p!=0; p++){
		if(*p == '/'){
			*p = 0;
			if(mkdir(tmp, S_IRWXU) && errno!=EEXIST){
				perror("ERROR calling mkdir");
				printf("tried to mkdir %s\n", tmp);
				return -1;
			}
			*p = '/';
		}
	}
	return 0;
}


static void _open_vins_helper_cb( __attribute__((unused)) int ch, char* data, int bytes, __attribute__((unused)) void* context)
{

    if (failure) return;

    // validate that the data makes sense
    int n_packets, i;
    vio_data_t* data_array = pipe_validate_vio_data_t(data, bytes, &n_packets);

    if(data_array == NULL){
        // uh oh
        fprintf(stderr, "fail\n");
    }

    if (data_array[n_packets-1].state == VIO_STATE_FAILED){
        fprintf(stderr, "FAILURE PACKET RECEIVED\n");
        failure = true;
        return;
    }

    if (!is_first_set){
        first_timestamp_ns = data_array[n_packets-1].timestamp_ns;
    }

    last_timestamp_ns = data_array[n_packets-1].timestamp_ns;
    
    memcpy(last_pose, data_array[n_packets-1].T_imu_wrt_vio, 3);

    return;
}


static void _open_vins_disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
    fprintf(stderr, "disconnected from open-vins\n");
    failure = true;
	return;
}


static int setup_ov_connection(){
    pipe_client_set_simple_helper_cb(0, _open_vins_helper_cb, NULL);
	pipe_client_set_disconnect_cb(0, _open_vins_disconnect_cb, NULL);

	int ret = pipe_client_open(0, OV_NAME, CLIENT_NAME, \
				EN_PIPE_CLIENT_SIMPLE_HELPER, \
				VIO_RECOMMENDED_READ_BUF_SIZE);

    return ret;
}


static int start_sub_processes(int processes_left){
    // need to fork 3 times
    // replay, feature tracker, and open-vins
    pid_t pid;

    if(processes_left > 0)
    {
        if ((pid = fork()) < 0)
        {
            perror("fork");
        }
        else if (pid == 0)
        {
            //Child stuff here
            // printf("Child %d, going to execute %s with args %s\n", processes_left, process_paths[3-processes_left], process_args[3-processes_left]);
            execl(process_paths[3-processes_left], process_paths[3-processes_left], process_args[3-processes_left], (char *)0);
            usleep(500000);
        }
        else if(pid > 0)
        {
            // we got our child pid here, so lets set it in the array
            sub_pid_list[3-processes_left] = pid;

            //parent
            // printf("Parent %d\n", processes_left);
            usleep(500000);
            start_sub_processes(processes_left - 1);
        }
    }
}


static void close_subprocesses(){
    for (int i = 0; i < 3; i++){
        kill(sub_pid_list[i], SIGTERM);
        // reset
        sub_pid_list[i] = 0;
    }
}


static int create_session_dir(){

    system("voxl-set-cpu-mode performance");

    time_t rawtime;
    struct tm * timeinfo;
    char buffer[128];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%d-%m-%Y/",timeinfo);

    session_dir = std::string(OUTPUT_DIR);
    session_dir.append(std::string(buffer));

    _mkdir(session_dir.c_str());
}


static int log_results(){
    // make a directory within our session dir
    char curr_dir[PATH_MAX];

    // search for existing log folders to determine the next number in the series
	for(int i=0; i<=10000; i++){

		// make the next log directory in the sequence
		sprintf(curr_dir, "%srun%04d/", session_dir.c_str(), i);

		// use basic mkdir to try making this directory
		int ret = mkdir(curr_dir, S_IRWXU);
		if(ret==0){
			break;
		}
		if(errno!=EEXIST){
			perror("error searching for next log directory in sequence");
			return -1;
		}
		// dir exists, move onto the next one
	}

    // create sub dir params
    char param_dir[PATH_MAX];
    sprintf(param_dir, "%sparams/", curr_dir);
    _mkdir(param_dir);

    // copy all relevant files over
    char cmd[512];
    sprintf(cmd, "cp /etc/modalai/voxl-feature-tracker.conf %s", param_dir);
    system(cmd);
    sprintf(cmd, "cp /etc/modalai/voxl-open-vins-server.conf %s", param_dir);
    system(cmd);

    // create csv in new dir
	char csv_path[512];
	sprintf(csv_path, "%sdata.csv", curr_dir);
    _mkdir(csv_path);
	FILE* fd = fopen(csv_path, "w+");

    // add duration and final pose to the csv
    fprintf(fd, "duration(s),final_pose_x(m),final_pose_y(m),final_pose_z(m)\n");

    double duration = (double)(last_timestamp_ns - first_timestamp_ns)/1000000000.;
    fprintf(fd, "%f,%f,%f,%f\n", duration, last_pose[0], last_pose[1], last_pose[2]);

    fclose(fd);

    return 0;
}


static void setup_param_tables(){
    // idea: each string param maps to a vector of ints that we can simply iterate through
    // the index of the parameters vector is determined by:
    // index % n_params % vector of this param size

    n_total_params += (n_ov_params * n_ov_params * 97);
    n_total_params += (n_ft_params * n_ov_params * 97);

    std::vector<int> bool_vec;
    for (int i = 0; i < 2; i++){
        bool_vec.push_back(i);
    }

    param_table.insert({"cam_to_imu_refinement", bool_vec});
    param_table.insert({"cam_intrins_refinement", bool_vec});
    param_table.insert({"cam_imu_ts_refinement", bool_vec});

    std::vector<int> win_vec;
    std::vector<int> rep_vec;
    for (int i = 0; i < 6; i++){
        rep_vec.push_back(i);
        win_vec.push_back(i*2);
    }

    param_table.insert({"feat_rep_msckf", rep_vec});
    param_table.insert({"feat_rep_slam", rep_vec});

    std::vector<int> clone_vec;
    for (int i = 2; i < 52; i++){
        clone_vec.push_back(i*2);
    }

    param_table.insert({"max_clone_size", clone_vec});
    param_table.insert({"max_slam_features", clone_vec});
    param_table.insert({"max_slam_in_update", clone_vec});
    param_table.insert({"max_msckf_in_update", clone_vec});

    std::vector<int> chi2_vec;
    for (int i = 0; i < 10; i++){
        chi2_vec.push_back(i);
    }

    param_table.insert({"msckf_chi2_multiplier", chi2_vec});
    param_table.insert({"msckf_sigma_px", chi2_vec});
    param_table.insert({"slam_chi2_multiplier", chi2_vec});
    param_table.insert({"slam_sigma_px", chi2_vec});

    std::vector<int> features_vec;
    for (int i = 0; i < 25; i++){
        features_vec.push_back(i*2);
    }

    param_table.insert({"num_features_to_track", features_vec});

    param_table.insert({"grid_x", chi2_vec});
    param_table.insert({"grid_y", chi2_vec});

    std::vector<int> dist_vec;
    for (int i = 0; i < 25; i++){
        features_vec.push_back(i);
    }

    param_table.insert({"min_pix_dist", dist_vec});

    std::vector<int> levels_vec;
    for (int i = 0; i < 4; i++){
        levels_vec.push_back(i);
    }

    param_table.insert({"pyramid_levels", levels_vec});
    param_table.insert({"window_size", win_vec});

    return;
}


static int update_open_vins_params(int index){

    cJSON *parent = json_read_file(OV_CONFIG_FILE);

    if (parent == NULL) return -1;

    // cJSON_SetValuestring
    // cJSON_SetNumberValue
    // cJSON_SetIntValue

    cJSON* cam_to_imu_refinement = cJSON_GetObjectItem(parent, "cam_to_imu_refinement");
    cJSON_SetIntValue(cam_to_imu_refinement, param_table["cam_to_imu_refinement"][index % n_ov_params % param_table["cam_to_imu_refinement"].size()]);

    cJSON* cam_intrins_refinement = cJSON_GetObjectItem(parent, "cam_intrins_refinement");
    cJSON_SetIntValue(cam_intrins_refinement, param_table["cam_intrins_refinement"][index % n_ov_params % param_table["cam_intrins_refinement"].size()]);

    cJSON* cam_imu_ts_refinement = cJSON_GetObjectItem(parent, "cam_imu_ts_refinement");
    cJSON_SetIntValue(cam_imu_ts_refinement, param_table["cam_imu_ts_refinement"][index % n_ov_params % param_table["cam_imu_ts_refinement"].size()]);

    cJSON* max_clone_size = cJSON_GetObjectItem(parent, "max_clone_size");
    cJSON_SetIntValue(max_clone_size, param_table["max_clone_size"][index % n_ov_params % param_table["max_clone_size"].size()]);

    cJSON* max_slam_features = cJSON_GetObjectItem(parent, "max_slam_features");
    cJSON_SetIntValue(max_slam_features, param_table["max_slam_features"][index % n_ov_params % param_table["max_slam_features"].size()]);

    cJSON* max_slam_in_update = cJSON_GetObjectItem(parent, "max_slam_in_update");
    cJSON_SetIntValue(max_slam_in_update, param_table["max_slam_in_update"][index % n_ov_params % param_table["max_slam_in_update"].size()]);

    cJSON* max_msckf_in_update = cJSON_GetObjectItem(parent, "max_msckf_in_update");
    cJSON_SetIntValue(max_msckf_in_update, param_table["max_msckf_in_update"][index % n_ov_params % param_table["max_msckf_in_update"].size()]);

    cJSON* feat_rep_msckf = cJSON_GetObjectItem(parent, "feat_rep_msckf");
    cJSON_SetIntValue(feat_rep_msckf, param_table["feat_rep_msckf"][index % n_ov_params % param_table["feat_rep_msckf"].size()]);

    cJSON* feat_rep_slam = cJSON_GetObjectItem(parent, "feat_rep_slam");
    cJSON_SetIntValue(feat_rep_slam, param_table["feat_rep_slam"][index % n_ov_params % param_table["feat_rep_slam"].size()]);

    cJSON* msckf_chi2_multiplier = cJSON_GetObjectItem(parent, "msckf_chi2_multiplier");
    cJSON_SetNumberValue(msckf_chi2_multiplier, param_table["msckf_chi2_multiplier"][index % n_ov_params % param_table["msckf_chi2_multiplier"].size()]);

    cJSON* msckf_sigma_px = cJSON_GetObjectItem(parent, "msckf_sigma_px");
    cJSON_SetNumberValue(msckf_sigma_px, param_table["msckf_sigma_px"][index % n_ov_params % param_table["msckf_sigma_px"].size()]);

    cJSON* slam_chi2_multiplier = cJSON_GetObjectItem(parent, "slam_chi2_multiplier");
    cJSON_SetNumberValue(slam_chi2_multiplier, param_table["slam_chi2_multiplier"][index % n_ov_params % param_table["slam_chi2_multiplier"].size()]);

    cJSON* slam_sigma_px = cJSON_GetObjectItem(parent, "slam_sigma_px");
    cJSON_SetNumberValue(slam_sigma_px, param_table["slam_sigma_px"][index % n_ov_params % param_table["slam_sigma_px"].size()]);

    if (index % n_ov_params % param_table["slam_sigma_px"].size() == param_table["slam_sigma_px"].size()-1) return -2;

    // cJSON* triangulate_1d = cJSON_GetObjectItem(parent, "triangulate_1d");
    // cJSON_SetIntValue(triangulate_1d, param_table["triangulate_1d"][index % n_ov_params % param_table["triangulate_1d"].size()]);

    // cJSON* refine_features = cJSON_GetObjectItem(parent, "refine_features");
    // cJSON_SetIntValue(refine_features, param_table["refine_features"][index % n_ov_params % param_table["refine_features"].size()]);

    // cJSON* max_runs = cJSON_GetObjectItem(parent, "max_runs");
    // cJSON_SetIntValue(max_runs, param_table["max_runs"][index % n_ov_params % param_table["max_runs"].size()]);

    // cJSON* min_dist = cJSON_GetObjectItem(parent, "min_dist");
    // cJSON_SetNumberValue(min_dist, 1.);
    // cJSON* max_dist = cJSON_GetObjectItem(parent, "max_dist");
    // cJSON_SetNumberValue(max_dist, 1.);
    // cJSON* max_baseline = cJSON_GetObjectItem(parent, "max_baseline");
    // cJSON_SetNumberValue(max_baseline, 1.);
    // cJSON* max_cond_number = cJSON_GetObjectItem(parent, "max_cond_number");
    // cJSON_SetNumberValue(max_cond_number, 1.);

    json_write_to_file(OV_CONFIG_FILE, parent);

    fprintf(stderr, "finishing ov param setter function\n");

    cJSON_Delete(parent);

    return 0;
}


static int update_feature_tracker_params(int index){
    cJSON *parent = json_read_file(FT_CONFIG_FILE);
    if (parent == NULL) return -1;

    cJSON* num_features_to_track = cJSON_GetObjectItem(parent, "num_features_to_track");
    cJSON_SetIntValue(num_features_to_track, param_table["num_features_to_track"][index % n_ft_params % param_table["num_features_to_track"].size()]);

    cJSON* grid_x = cJSON_GetObjectItem(parent, "grid_x");
    cJSON_SetIntValue(grid_x, param_table["grid_x"][index % n_ft_params % param_table["grid_x"].size()]);

    std::cout << "grid x: " << param_table["grid_x"][index % n_ft_params % param_table["grid_x"].size()] << std::endl;

    cJSON* grid_y = cJSON_GetObjectItem(parent, "grid_y");
    cJSON_SetIntValue(grid_y, param_table["grid_y"][index % n_ft_params % param_table["grid_y"].size()]);

    cJSON* min_pix_dist = cJSON_GetObjectItem(parent, "min_pix_dist");
    cJSON_SetIntValue(min_pix_dist, param_table["min_pix_dist"][index % n_ft_params % param_table["min_pix_dist"].size()]);

    std::cout << "min_pix_dist: " << param_table["min_pix_dist"][index % n_ft_params % param_table["min_pix_dist"].size()] << std::endl;


    cJSON* pyramid_levels = cJSON_GetObjectItem(parent, "pyramid_levels");
    cJSON_SetIntValue(pyramid_levels, param_table["pyramid_levels"][index % n_ft_params % param_table["pyramid_levels"].size()]);

    cJSON* window_size = cJSON_GetObjectItem(parent, "window_size");
    cJSON_SetIntValue(window_size, param_table["window_size"][index % n_ft_params % param_table["window_size"].size()]);

    json_write_to_file(FT_CONFIG_FILE, parent);
    cJSON_Delete(parent);

    return 0;
}


static int update_params(int index){
    int ret = 0;

    ret = update_open_vins_params(index);
    if (!update_feature_tracker_params(index)){
        return -3;
    }

    return ret;
}


static void reset_vars(){
    failure = false;
    memset(last_pose, 0, 3);
    last_timestamp_ns = 0;
    first_timestamp_ns = 0;
    is_first_set = false;
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

    create_session_dir();

    setup_ov_connection();

    setup_param_tables();

    // make the base dir, ensure that it exists
	_mkdir(OUTPUT_DIR);

    bool final_run = false;
    int ret = 0;
    for (int i = 0; i < 50; i++){
        start_sub_processes(3);

        // keep going until signal handler or callback sets the running flag to 0
        while(main_running) {
            usleep(200000);

            if (failure) break;

            for (int i = 0; i < 3; i++){
                if (!kill(sub_pid_list[i], 0)) {
                    // fprintf(stderr, "pid: %d\n", sub_pid_list[i]);
                    // perror("ERROR: ");
                }
                if (errno == ESRCH) {
                    failure = true;
                    break;
                }
            }
        }

        close_subprocesses();

        log_results();

        if (final_run) break;

        ret = update_params(i);
        if (ret == -2){
            final_run = true;
        }

        reset_vars();
    }
    
    main_running = 0;

    // all done, signal pipe read threads to stop
    printf("\nclosing and exiting\n");
    pipe_client_close_all();

    return 0;
}
