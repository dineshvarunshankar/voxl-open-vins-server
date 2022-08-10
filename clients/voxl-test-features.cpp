#include <modal_pipe.h>
#include <modalcv.h>
#include <stdio.h>
#include <signal.h>
#include <getopt.h>
#include <unistd.h> // for usleep()
#include <string.h>
#include <stdlib.h> // for atoi()
#include <math.h>
#include <unistd.h>		// for access()
#include <sys/stat.h>	// for mkdir
// lots of opencv includes for examples
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <vector>


#define CLIENT_NAME     "voxl-test-features"


std::vector<mcv_fpx_feature_t*> mcv_features;
FILE* fd;

static int64_t _apps_time_monotonic_ns() {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts)) {
        fprintf(stderr, "ERROR calling clock_gettime\n");
        return -1;
    }
    return (int64_t)ts.tv_sec * 1000000000 + (int64_t)ts.tv_nsec;
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


static void _new_camera_data_handler(int ch, camera_image_metadata_t meta, char* frame, __attribute__((unused)) void* context) {
    int64_t post_start = _apps_time_monotonic_ns();

    int i;
    static int frame_id = 0;
    static bool first_try = true;
    static int prev_points = 0;
    static mcv_dcm_pos_t prev_positions[MCV_MAX_POS_BUF_SIZE];
    static cv::Mat prev_img;
    
    static cv::Mat ref_image;

    // Unpack the data into opencv image Mats
    cv::Mat _img(meta.height, meta.width, CV_8UC1, frame);
    cv::Mat img;

    // cv::GaussianBlur(_img, img, cv::Size(7,7), 0, 0, cv::BORDER_DEFAULT);    
    cv::GaussianBlur(_img, img, cv::Size(7, 7), 2, 2, cv::BORDER_REFLECT_101);

    
    mcv_dcm_pos_t positions_1[MCV_MAX_POS_BUF_SIZE];
    mcv_dcm_match_t matches_0_1[MCV_MAX_MATCH_BUF_SIZE];
    mcv_dcm_match_t matches_1_0[MCV_MAX_MATCH_BUF_SIZE];
    int n_matches, n_matches_2;
    int n_points_1, n_points_2;
    uint32_t max_score;
    int64_t fpx_start = _apps_time_monotonic_ns();
    
    int ret = mcv_fpx_process(img.data, mcv_features[0], &max_score, &n_points_1);
    if(ret) return;
    
    int64_t fpx_end = _apps_time_monotonic_ns();
    
    // make sure we don't go over the descriptor limit, this likely won't happen
    if(n_points_1>MCV_DCM_MAX_DESCRIPTORS) n_points_1 = MCV_DCM_MAX_DESCRIPTORS;
    // fprintf(fd, "%d,%d,", frame_id, n_points_1);

    // populate the search positions from the features
    for(i=0;i<n_points_1;i++){
        mcv_overlay_draw_plus(img.data, 1280, 800, mcv_features[0][i].x, mcv_features[0][i].y);
        // fprintf(fd, "%d,",mcv_features[0][i].score);

        positions_1[i].x = mcv_features[0][i].x;
        positions_1[i].y = mcv_features[0][i].y;
        char buf[50];
        sprintf(buf, "%d",  mcv_features[0][i].score);
        mcv_overlay_write_string_large_white(img.data, img.cols, img.rows, buf, positions_1[i].x, positions_1[i].y);
    }
    // fprintf(fd, "\n");

    if (first_try){
        int64_t match_start = _apps_time_monotonic_ns();
        // first frame through, so there won't be any matches, just do calc
        if(mcv_dcm_calc(img.data, positions_1, n_points_1, 0)){
            return;
        }
        first_try = false;
        prev_img = img.clone();
        memset(prev_positions, 0, MCV_MAX_MATCH_BUF_SIZE * sizeof(mcv_dcm_pos_t));
        memcpy(prev_positions, positions_1, n_points_1 * sizeof(mcv_dcm_pos_t));
        prev_points = n_points_1;
        int64_t match_end = _apps_time_monotonic_ns();
        fprintf(stderr, "descriptor calc took %6.5fms total\n", (match_end - match_start)/1e6);
        frame_id++;
        return;
    }
    else {
        int64_t match_start = _apps_time_monotonic_ns();
        // since last arg is true, will update descriptor set to match the latest feature set
        if(mcv_dcm_match(img.data, positions_1, n_points_1, matches_0_1, &n_matches, 0, true)){
            fprintf(stderr, "EARLY RET\n");
            return;
        }

        int64_t FIRST_match_start = _apps_time_monotonic_ns();
        // param in match call on whether or not to update the descriptors?
        // OR just return the descriptors, and be able to set them yourself
        // if descriptors are not updated, the reference ones will be correct for the next sequential match
        if(mcv_dcm_match(prev_img.data, prev_positions, prev_points, matches_1_0, &n_matches_2, 0, false)){
            fprintf(stderr, "EARLY RET\n");
            return;
        }
        int64_t match_end = _apps_time_monotonic_ns();
        // fprintf(stderr, "matching took %6.5fms total, %6.5f first\n", (match_end - match_start)/1e6, (FIRST_match_start - match_start)/1e6);
        // fprintf(stderr, "with %d feats prev and %d feats curr\n", prev_points, n_points_1);
    }

    fprintf(stderr, "MATCHED %d out of %d\n", n_matches, n_points_1);
    std::vector<cv::KeyPoint> points_1;
    std::vector<cv::KeyPoint> points_2;
    std::vector<cv::DMatch> matches_1_to_2;
    std::vector<cv::DMatch> matches_fin;
    std::vector<cv::DMatch> matches_2_to_1;
    std::vector<cv::KeyPoint> points_1fin;
    std::vector<cv::KeyPoint> points_2fin;
    int a_ind = 0;
    int64_t test_t = _apps_time_monotonic_ns();

    std::vector<std::vector<std::pair<int, int>>> desc_matches(prev_points);

    for(i=0;i<n_matches;i++){
        uint16_t idx   = matches_0_1[i].index;
        uint16_t score = matches_0_1[i].score;

        desc_matches[idx].push_back(std::make_pair(score, i));
    }

    for (int i = 0; i < desc_matches.size(); i++){
        // fprintf(stderr, "index: %d\n", i);
        if (desc_matches[i].empty()) continue;
        std::sort(desc_matches[i].begin(), desc_matches[i].end());
        fprintf(stderr, "lowest score: %d\n", desc_matches[i][0].first);
        points_1.push_back(cv::KeyPoint(prev_positions[i].x, prev_positions[i].y, 1.));
        matches_1_to_2.push_back(cv::DMatch(0,0, 1));
        points_2.push_back(cv::KeyPoint(positions_1[desc_matches[i][0].second].x, positions_1[desc_matches[i][0].second].y, 1.));


        for (int j =0; j < desc_matches[i].size(); j++){
            fprintf(stderr, "score: %d\n", desc_matches[i][j]);
        }
        fprintf(stderr, "\n");
    }

    // for(i=0;i<n_matches;i++){
    //     uint16_t idx   = matches_0_1[i].index;
    //     uint16_t score = matches_0_1[i].score;
    //     if (score < 20){
    //         points_1.push_back(cv::KeyPoint(prev_positions[idx].x, prev_positions[idx].y, 1.));
    //         points_2.push_back(cv::KeyPoint(positions_1[i].x, positions_1[i].y, 1.));
    //         matches_1_to_2.push_back(cv::DMatch(idx, i, 1));
    //         a_ind++;
    //     }
    // }

    // for (int i = 0; i < ??)
    // for(i=0;i<n_matches_2;i++){
    //     uint16_t idx   = matches_1_0[i].index;
    //     uint16_t score = matches_1_0[i].score;
    //     if (score < 20){
    //         matches_2_to_1.push_back(cv::DMatch(i, idx, 1));
    //         a_ind++;
    //     }
    // }
    fprintf(stderr, "here\n");
    a_ind = 0;
    int discarded = 0;
    for (int i = 0; i < matches_1_to_2.size(); i++){
        int prev_ind = a_ind;
        // for (int j = 0; j < matches_2_to_1.size(); j++){
            // if (matches_2_to_1[j].trainIdx == matches_1_to_2[i].queryIdx && matches_1_to_2[i].trainIdx == matches_2_to_1[j].queryIdx){
                matches_fin.push_back(cv::DMatch(a_ind, a_ind, 1));
                a_ind++;
                points_1fin.push_back(points_1[i]);
                points_2fin.push_back(points_2[i]);
                // break;
            // }
            // else continue;
        // }
        // if (prev_ind == a_ind) discarded++;
    }
    int64_t test_e = _apps_time_monotonic_ns();
    fprintf(stderr, "filtering took %6.5fms\n", (test_e - test_t)/1e6);
    if (points_1.empty()) return;

    cv::Mat final_out, final_out1, final_out2;
    // draw the real matches
    cv::drawMatches(prev_img, points_1fin, img, points_2fin, matches_fin, final_out1, cv::Scalar(0), cv::Scalar(0));
    // fprintf(stderr, "GOOD MATCHES: %d, DISCARDS: %d\n", matches_fin.size(), discarded);
    cv::cvtColor(final_out1,final_out,cv::COLOR_BGR2GRAY);
    camera_image_metadata_t t_meta = meta;
    t_meta.format = IMAGE_FORMAT_RAW8;
    t_meta.width = final_out.cols;
    t_meta.height = final_out.rows;
    t_meta.size_bytes = t_meta.width * t_meta.height;
    pipe_server_write_camera_frame(1, t_meta, final_out.data);
    prev_img = img.clone();

    memset(prev_positions, 0, MCV_MAX_MATCH_BUF_SIZE * sizeof(mcv_dcm_pos_t));
    memcpy(prev_positions, positions_1, n_points_1 * sizeof(mcv_dcm_pos_t));
    int64_t post_end = _apps_time_monotonic_ns();
    double p_time = (post_end - post_start)/1e6;
    // fprintf(stderr, "processing took %6.5fms\n", (post_end - post_start)/1e6);
    prev_points = n_points_1;
    frame_id++;
    return;
}


static int setup_fpx(int n_cameras) {
    ////////////////////////////////////////////////////////////////////////////
    // initialize feature point extractor
    ////////////////////////////////////////////////////////////////////////////
    mcv_fpx_config_t fpx_config;
    fpx_config.width = 1280;  // TODO pull this dynamically
    fpx_config.height = 800;  // TODO pull this dynamically
    // fpx_config.mode = MCV_FPX_PEAK_8x8;
    fpx_config.mode = MCV_FPX_ZONE;
    fpx_config.nms_mode = MCV_FPX_5_TAP_NMS;
    fpx_config.score_threshold = 100;
    fpx_config.robustness = 10;  // 0-127, default 10
    int feature_buf_size;
    if (mcv_fpx_init(fpx_config, &feature_buf_size)) {
        return -1;
    }
    // malloc required memory for feature output
    printf("allocating %d bytes for features\n", feature_buf_size);
    mcv_features.resize(n_cameras);
    for (int i = 0; i <n_cameras; i++){
        mcv_features[i] = (mcv_fpx_feature_t*)malloc(feature_buf_size);
    }
    if(mcv_dcm_init(1280, 800, n_cameras)){
        return -1;
    }
    return 0;
}
int main(int argc, char* argv[])
{

    // make any required subdirs and the csv itself
	_mkdir("/data/features.csv");
	fd = fopen("/data/features.csv", "w+");
	if(fd == 0){
		fprintf(stderr, "ERROR: can't open log file for writing\n");
		return -1;
	}

	// write header
	int ret = 0;
    ret = fprintf(fd, "frame_id,feat_count,scores\n");

    // set some basic signal handling for safe shutdown.
    // quitting without cleanup up the pipe can result in the pipe staying
    // open and overflowing, so always cleanup properly!!!
    enable_signal_handler();
    main_running = 1;
    setup_fpx(1);
     // init overlay pipe
    pipe_info_t info3 = {
        "feature_out",        // name
        "/run/mpa/feature_out/",    // location
        "camera_image_metadata_t",  // type
        CLIENT_NAME,               // server_name
        1280*800*3,              // size_bytes
        0                           // server_pid
    };
    if (pipe_server_create(1, info3, 0)) {
        exit(-1);
    }
    pipe_client_set_camera_helper_cb(0, _new_camera_data_handler, NULL);
    int flags = CLIENT_FLAG_EN_CAMERA_HELPER;
    if (pipe_client_open(0, "/run/mpa/stereo_front/", CLIENT_NAME, flags, 1280 * 800 * 60) != 0) {
        fprintf(stderr, "ERROR: FAILED TO OPEN %s\n", "/run/mpa/stereo_front/");
        return -1;
    }
    // keep going until signal handler sets the running flag to 0
    while(main_running) usleep(200000);
    // all done, signal pipe read threads to stop
    printf("\nclosing and exiting\n");
    pipe_client_close_all();
    mcv_dcm_deinit();
    mcv_fpx_deinit();

    return 0;
}