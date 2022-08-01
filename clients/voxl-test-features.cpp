#include <modal_pipe.h>
#include <modalcv.h>
#include <stdio.h>
#include <signal.h>
#include <getopt.h>
#include <unistd.h>	// for usleep()
#include <string.h>
#include <stdlib.h> // for atoi()
#include <math.h>
// lots of opencv includes for examples
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include <vector>

#define CLIENT_NAME		"voxl-test-features"


std::vector<mcv_fpx_feature_t*> mcv_features;


static void _new_camera_data_handler(int ch, camera_image_metadata_t meta, char* frame, __attribute__((unused)) void* context) {
    int i;
    static bool first_try = true;
    static mcv_dcm_pos_t prev_positions[MCV_MAX_POS_BUF_SIZE];
    static cv::Mat prev_img;

    if (meta.format == IMAGE_FORMAT_STEREO_NV12 || meta.format == IMAGE_FORMAT_STEREO_NV21) {
        // Unpack the data into opencv image Mats
        cv::Mat img(meta.height, meta.width, CV_8UC1, frame);

        mcv_dcm_pos_t positions_1[MCV_MAX_POS_BUF_SIZE];
        mcv_dcm_match_t matches_0_1[MCV_MAX_MATCH_BUF_SIZE];
        mcv_dcm_match_t matches_1_0[MCV_MAX_MATCH_BUF_SIZE];

        int n_matches, n_matches_2;

        int n_points_1, n_points_2;
	    uint32_t max_score;

        int ret = mcv_fpx_process(img.data, mcv_features[0], &max_score, &n_points_1);
        if(ret) return ;

        printf("n_points_1:  %5d\n", n_points_1);
        printf("max_score: %5d\n", max_score);

        // make sure we don't go over the descriptor limit, this likely won't happen
        if(n_points_1>MCV_DCM_MAX_DESCRIPTORS) n_points_1 = MCV_DCM_MAX_DESCRIPTORS;

        // populate the search positions from the features
        for(i=0;i<n_points_1;i++){
            positions_1[i].x = mcv_features[0][i].x;
            positions_1[i].y = mcv_features[0][i].y;
            // printf("xy: %3d %3d\n", positions_1[i].x, positions_1[i].y);
        }

        if (first_try){
            // first frame through, so there won't be any matches, just do calc
            if(mcv_dcm_calc(img.data, positions_1, n_points_1, 0)){
                return;
            }
            first_try = false;
            prev_img = img.clone();
            memset(prev_positions, 0, MCV_MAX_MATCH_BUF_SIZE * sizeof(mcv_dcm_pos_t));
            memcpy(prev_positions, positions_1, n_points_1 * sizeof(mcv_dcm_pos_t));
            return;
        }
        else {
            // first frame through, so there won't be any matches
            if(mcv_dcm_match(img.data, positions_1, n_points_1, matches_0_1, &n_matches, 0)){
                return;
            }
            if(mcv_dcm_match(prev_img.data, prev_positions, MCV_DCM_MAX_DESCRIPTORS, matches_1_0, &n_matches_2, 0)){
                return;
            }
            first_try = true;
        }


        std::vector<cv::KeyPoint> points_1;
        std::vector<cv::KeyPoint> points_2;
        std::vector<cv::DMatch> matches_1_to_2;
        std::vector<cv::DMatch> matches_fin;



        std::vector<cv::KeyPoint> _points_1;
        std::vector<cv::KeyPoint> _points_2;
        std::vector<cv::DMatch> matches_2_to_1;


        int a_ind = 0;

        std::vector<int> buckets(MCV_DCM_MAX_DESCRIPTORS, 25);
        std::vector<int> buckets_2(MCV_DCM_MAX_DESCRIPTORS, 25);
        // fprintf(stderr, "SIZE N 2: %d\n", n_matches_2);


        for(i=0;i<n_matches;i++){
            uint16_t idx   = matches_0_1[i].index;
            uint16_t score = matches_0_1[i].score;

            if (score < 20 && buckets[idx] > score){
                buckets[idx] = score;
            }
        }

        for(i=0;i<n_matches_2;i++){
            uint16_t idx   = matches_1_0[i].index;
            uint16_t score = matches_1_0[i].score;

            if (score < 20 && buckets_2[idx] > score){
                buckets_2[idx] = score;
            }
        }

        for(i=0;i<n_matches;i++){
            uint16_t idx   = matches_0_1[i].index;
            uint16_t score = matches_0_1[i].score;

            if (score < 20){
                if (buckets[idx] != score){
                    // fprintf(stderr, "FAILED BUCKET CHECK: score is %d, bucket is %d\n", score, buckets[idx]);
                    continue;
                }

                // draw the features we just extracted on frame 1
                // mcv_overlay_draw_plus(prev_img.data, prev_img.cols, prev_img.rows, prev_positions[idx].x, prev_positions[idx].y);
                // mcv_overlay_draw_plus(img.data, img.cols, img.rows, positions_1[i].x, positions_1[i].y);

                points_1.push_back(cv::KeyPoint(prev_positions[idx].x, prev_positions[idx].y, 1.));
                points_2.push_back(cv::KeyPoint(positions_1[i].x, positions_1[i].y, 1.));
                matches_1_to_2.push_back(cv::DMatch(idx, i, 1));
                a_ind++;
            }
        }

        a_ind = 0;
        for(i=0;i<n_matches_2;i++){
            uint16_t idx   = matches_1_0[i].index;
            uint16_t score = matches_1_0[i].score;

            if (score < 20){
                if (buckets_2[idx] != score){
                    // fprintf(stderr, "FAILED BUCKET CHECK: score is %d, bucket is %d\n", score, buckets[idx]);
                    continue;
                }

                // draw the features we just extracted on frame 1
                // mcv_overlay_draw_plus(prev_img.data, prev_img.cols, prev_img.rows, prev_positions[idx].x, prev_positions[idx].y);
                // mcv_overlay_draw_plus(img.data, img.cols, img.rows, positions_1[i].x, positions_1[i].y);

                _points_1.push_back(cv::KeyPoint(positions_1[idx].x, positions_1[idx].y, 1.));
                _points_2.push_back(cv::KeyPoint(prev_positions[i].x, prev_positions[i].y, 1.));
                matches_2_to_1.push_back(cv::DMatch(i, idx, 1));
                a_ind++;
            }
        }
        a_ind = 0;
        int discarded = 0;
        std::vector<cv::KeyPoint> points_1fin;
        std::vector<cv::KeyPoint> points_2fin;
        for (int i = 0; i < matches_1_to_2.size(); i++){
            int prev_ind = a_ind;
            // logic required to do this is:
            // if matches_1t2.queryind is in matches_2t1, the train inds must be the same?
            for (int j = 0; j < matches_2_to_1.size(); j++){
                if (matches_2_to_1[j].trainIdx == matches_1_to_2[i].queryIdx){
                    matches_fin.push_back(cv::DMatch(a_ind, a_ind, 1));
                    a_ind++;
                    points_1fin.push_back(points_1[i]);
                    points_2fin.push_back(points_2[i]);
                    break;
                }
                else {
                    continue;
                }
            }
            if (prev_ind == a_ind)                    discarded++;


        }

        if (points_1.empty()) return;

        cv::Mat final_out, final_out1, final_out2;
        // draw the real matches
        cv::drawMatches(prev_img, points_1fin, img, points_2fin, matches_fin, final_out1);
        // fprintf(stderr, "SIZE 2: %d\n", matches_2_to_1.size());
        // cv::drawMatches(img, _points_1, prev_img, _points_2, matches_2_to_1, final_out2);

        // cv::vconcat(final_out1, final_out2, final_out);

        fprintf(stderr, "GOOD MATCHES: %d, DISCARDS: %d\n", matches_fin.size(), discarded);

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
    }

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
    fpx_config.score_threshold = 10;
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

	return 0;
}