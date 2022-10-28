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


#ifndef QUALITY_H
#define QUALITY_H

// #define DEBUG_QUALITY

#include <modal_pipe_interfaces.h>

// tunable params
#define GRID_W 20
#define GRID_H 15
#define BLOCKS_FOR_100_PERCENT 0.5f
#define STDDEV_WEIGHT 10.0f // higher number weights the stddev of features more

// non-tunable params
#define GRID_BLOCKS (GRID_W*GRID_H)
#define MAX_SCORE_PER_BLOCK (1.0f)

int grid_spacing_x;
int grid_spacing_y;



static void _populate_single_block(float score, float* grid_scores, int pos_x, int pos_y)
{
	if(pos_x>=GRID_W || pos_x<0) return;
	if(pos_y>=GRID_H || pos_y<0) return;

	int idx = (pos_y*GRID_W) + pos_x;

	if(score > grid_scores[idx]){
		grid_scores[idx] = score;
	}
	return;
}


static void _add_feature_to_grid(float score, float* grid_scores, int x, int y)
{
	// find grid square for this feature
	int pos_x = x/grid_spacing_x;
	int pos_y = y/grid_spacing_y;
	if(pos_x>=GRID_W) pos_x = GRID_W-1;
	if(pos_y>=GRID_H) pos_y = GRID_H-1;

	// now populate the region around the feature
	for(int i=-2; i<=2; i++){
		for(int j=-2; j<=2; j++){
			_populate_single_block(score, grid_scores, pos_x+i, pos_y+j);
		}
	}
	return;

}

/**
 * @brief      Calculates a vio quality based on feature points and covariance
 *
 *             From Mavlink documentation:
 *
 *             Optional odometry quality metric as a percentage. -1 = odometry
 *             has failed, 0 = unknown/unset quality, 1 = worst quality, 100 =
 *             best quality
 *
 * @param[in]  state       The state
 * @param      vel_cov     The velocity cov
 * @param[in]  img_w       The image w
 * @param[in]  img_h       The image h
 * @param[in]  n_features  The n features
 * @param      features    The features
 *
 * @return     The quality.
 */
static int calc_quality(uint8_t state, float* vel_cov, float vel_norm, int img_w, int img_h, int n_features, int n_cams, vio_feature_t* features)
{
	int i;

	if(state != VIO_STATE_OK){
		return -1;
	}

	if(vel_cov[0]<=0.0f || vel_cov[6]<=0.0f || vel_cov[11]<=0.0f){
		fprintf(stderr, "negative covariance\n");
		return -1;
	}

	if(img_w < GRID_W || img_h < GRID_H){
		fprintf(stderr, "ERROR in %s, invalid img width or height\n", __FUNCTION__);
		return -1;
	}

	grid_spacing_x = img_w/GRID_W;
	grid_spacing_y = img_h/GRID_H;

	float grid_scores[n_cams][GRID_BLOCKS];
	for (int i = 0; i < n_cams; i++){
		memset(grid_scores[i], 0, sizeof(float)*GRID_BLOCKS);
	}

	// loop through features and save the best depth covariance for each
	// block with a feature in it
	for(i=0; i<n_features; i++){

		// skip points not in state
		if(features[i].point_quality == LOW){
			#ifdef DEBUG_QUALITY
			fprintf(stderr, "skipping oos point with qual of %d\n", features[i].point_quality);
			#endif
			continue;
		}

		// skip points with negative pixel locations (I don't know why mvvislam
		// reports features with a position of -1
		int x = features[i].pix_loc[0];
		int y = features[i].pix_loc[1];
		if(x<0 || y<0) {
			#ifdef DEBUG_QUALITY
			fprintf(stderr, "skipping feat with negative loc\n");
			#endif
			continue;
		}

		// points with a higher stddev will contribute less to the total score
		float stddev = features[i].depth_error_stddev;
		if (stddev == -1.) continue;
		float score = MAX_SCORE_PER_BLOCK - STDDEV_WEIGHT*(stddev*stddev);

		if (features[i].point_quality == HIGH){
			score += MAX_SCORE_PER_BLOCK/3.;
			if (vel_norm < 0.05) score += MAX_SCORE_PER_BLOCK/3.;
		}

		_add_feature_to_grid(score, grid_scores[features[i].cam_id], x, y);
	}

#ifdef DEBUG_QUALITY
	for (int m = 0; m < n_cams; m++){
	printf("scores:\n");
		for(i=0; i<GRID_H; i++){
			for(int j=0; j<GRID_W; j++){
				printf("%3.1f ", (double)grid_scores[m][(i*GRID_W)+j]);
			}
			printf("\n");
		}
	}

#endif
	float sum[n_cams] = {0.0f};

	for (int m = 0; m < n_cams; m++){
		for(i=0; i<GRID_BLOCKS; i++) sum[m] += grid_scores[m][i];
	}

	float best_sum = 0;
	for (int m = 0; m < n_cams; m++){
		if (sum[m] > best_sum) best_sum = sum[m];	
	}
	// calculate the total score

	// theoretical max score would be a perfect feature in every block.
	// consider 100% quality to be half the blocks filled
	float perfect_sum = MAX_SCORE_PER_BLOCK * (GRID_BLOCKS) * BLOCKS_FOR_100_PERCENT;
	perfect_sum = perfect_sum*perfect_sum;

	int quality = roundf((best_sum*best_sum / perfect_sum) * 100.0f);

	// bound the output. Remember 0 is a special value meaning unknown so
	// don't report that since we calculating the quality.
	if(quality>100) quality = 100;
	if(quality<1) quality = -1;


#ifdef DEBUG_QUALITY
	printf("final quality: %d\n", quality);
#endif

	return quality;
}




/*
// this is the OLD quality metric saved here for reference
static int32_t _qvio_quality(struct mvVISLAMPose pose, int n_features)
{
	// vio quality uses covariance and a reset counter
	// the whole covarience matrix should be zero in case of blowup
	if(pose.errCovPose[3][3]<=0.0f || pose.errCovPose[4][4]<=0.0f || pose.errCovPose[5][5]<=0.0f){
		return -1.0f;
	}

	// pick the maximum (worst) of the velocity diagonal entries
	float max = pose.errCovVelocity[0][0];
	if(pose.errCovVelocity[1][1]>max) max = pose.errCovVelocity[1][1];
	if(pose.errCovVelocity[2][2]>max) max = pose.errCovVelocity[2][2];


	// covarience is very small. Scale it by 10^5 to make the output more readable
	float conf = 0.00001f / max;

	if (n_features <= 4 && conf > .15f){
		conf = .15;
	}
	else if (n_features <= 10 && conf > .40f){
		conf = .40;
	}

	// now scale up to 100
	conf *= 100;

	if (conf > 100) conf = 100;
	// some limits:
	// hard 100 limit
	// if we have <= 4 feats, quality cannot be above 15%
	// if we have <= 10 feats, quality cannot be above 40%

	return (int32_t)conf;
}

*/

#endif // end #define QUALITY_H
