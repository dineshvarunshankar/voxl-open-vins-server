

#include "shared_vars.h"
#include "vft_module.h"

#include <stdio.h>

#if defined(BUILD_QRB5165) && BUILD_QRB5165 == 1
#include <vft_interface.h>
#else
#include "common.h"
#endif


char vft_name[CHAR_BUF_SIZE] = "tracking_feats";

vft_feature_packet* feature_packet_from_vft;
vft_feature* features_from_vft;

std::deque<vft_feature_set> feature_queue;
std::mutex feature_queue_mtx;
vft_feature_set feat_set;
vft_feature_set feat_set_multi[4];
vft_feature_set feat_set_zero;



void _vft_disconnect_cb(__attribute__((unused)) int ch,
						__attribute__((unused)) void *context)
{
    destroy_vft_memory(&feature_packet_from_vft, &features_from_vft);
	return;
}

void _vft_data_default_handler(__attribute__((unused)) int ch, char* data, int bytes,
                       __attribute__((unused)) void* context)
{

	// receving feature data, reset errors
	global_error_codes &= ~ERROR_CODE_CAM_MISSING;

	if (validate_vft_data(ch, data, bytes, &feature_packet_from_vft, &features_from_vft)) {
	    printf("ERROR parsing vft data from pipe...\n");
		return;
	}

	double set_time = (double) feature_packet_from_vft->timestamp_ns[0] * 1e-9;

	static double last_set_time = set_time;
	static int n_total_features = 0;

	double dt_f = set_time-last_set_time;
	last_set_time = set_time;

	if (dt_f == 0)
		return;

	if (is_resetting)
		return;

	// testing logic
	int test_offset = 0;
	if (pause_cam_states[0] || pause_cam_states[1])
	{
		if (pause_cam_states[0])
		{
			test_offset = feature_packet_from_vft->num_feats[0];
			feature_packet_from_vft->num_feats[0] = 0;
		}
		if (pause_cam_states[1])
		{
			feature_packet_from_vft->num_feats[1] = 0;
		}
	}


	if (!vio_manager->initialized() || imu_moved || en_vio_always_on)
	{
		n_total_features = 0;

		if (use_takeoff_cam && (is_armed || en_vio_always_on) && (double) alt_z < takeoff_threshold) // turn off, we are in the air
		{
			if (!en_vio_always_on)
				printf(
					"Detected Takeoff, going to multicamera VINS normal operations\n");
			use_takeoff_cam = false;
		}

		int n_cams = feature_packet_from_vft->n_cams;

		for (int i = 0; i < n_cams; i++) {
			if (use_takeoff_cam && i == takeoff_cam)
			{
				n_total_features = 0;
				n_total_features = feature_packet_from_vft->num_feats[i];
				break;
			}
			else
			{
				n_total_features += feature_packet_from_vft->num_feats[i];
			}
		}

//		printf("n_cams (%d) ,%d ,%d ,%d ,%d  %d %d\n", n_cams, n_total_features, feature_packet_from_vft->num_feats[0], feature_packet_from_vft->num_feats[1], feature_packet_from_vft->num_feats[2], use_takeoff_cam, takeoff_cam);
//		int cam_num = 0;
//		int ctn = 0;
//		bool has_feats = 0;
		// not the best but VFT sends the same number for every camera
//		feat_set.features.resize(n_total_features);
//		feat_set_zero.features.resize(n_total_features);

		feat_set_zero.features.resize(feature_packet_from_vft->num_feats[0]);
		std::map<double, int> ts_map;

		{
			int last_feat_ctn = 0;

			// break out features in to their own feat_set
			for (int z=0;z<cameras_used;z++)
			{
				double ts_cam =  (double) feature_packet_from_vft->timestamp_ns[z] * 1e-9;
				ts_map[ts_cam] = z;

				feat_set_multi[z].timestamp = ts_cam;
				feat_set_multi[z].cam_id = 0;

				int cam_total_features = feature_packet_from_vft->num_feats[z];
				feat_set_multi[z].features.resize(cam_total_features);


				for (int x=0;x<cam_total_features;x++)
				{
					feat_set_multi[z].features[x].cam_id = z;
					feat_set_multi[z].features[x].id = features_from_vft[last_feat_ctn].id;
					feat_set_multi[z].features[x].u = features_from_vft[last_feat_ctn].x;
					feat_set_multi[z].features[x].v = features_from_vft[last_feat_ctn].y;  // TODO subtract from height?
					memcpy(feat_set_multi[z].features[x].descriptor, features_from_vft[last_feat_ctn].descriptor, 32);

					if (z == takeoff_cam)
					{
						feat_set_zero.cam_id = z;
						feat_set_zero.features[x].cam_id = z;
						feat_set_zero.features[x].id = features_from_vft[last_feat_ctn].id;
						feat_set_zero.features[x].u = features_from_vft[last_feat_ctn].x;
						feat_set_zero.features[x].v = features_from_vft[last_feat_ctn].y;  // TODO subtract from height?
						memcpy(feat_set_zero.features[x].descriptor, features_from_vft[last_feat_ctn].descriptor, 32);
					}
					last_feat_ctn++;
				}
			}

			int total_feats_ctn = 0;
			double avg_ts = 0;
			int num_cams_used = 0;
			feat_set.features.clear();
			for (const auto& pair : ts_map)
			{
				int cam_num = pair.second;
				if  (use_takeoff_cam && takeoff_cam >= 0  && cam_num != takeoff_cam)
				{
					continue;
				}

				num_cams_used++;
				avg_ts += feat_set_multi[cam_num].timestamp;
				feat_set.features.insert(feat_set.features.end(), feat_set_multi[cam_num].features.begin(), feat_set_multi[cam_num].features.end());

			}

			double cam_time =  avg_ts/num_cams_used;
//			double cam_time = ts_map.begin()->first;

//			printf("avg time %f cam0 %f cam1 %f\n", cam_time,  feat_set_multi[0].timestamp,feat_set_multi[1].timestamp);

			feat_set.timestamp = cam_time;
			feat_set_zero.timestamp = cam_time;
			std::lock_guard < std::mutex > lck(feature_queue_mtx);
			feature_queue.push_back(feat_set);
		}

//		else
//		{
//	        auto lastElement = ts_map.rbegin();
//
////			feat_set.timestamp = lastElement->first;
////			feat_set_zero.timestamp = lastElement->first;
////			std::lock_guard < std::mutex > lck(feature_queue_mtx);
////			feature_queue.push_back(feat_set);
//
//			for(const auto& pair : ts_map) {
//				if ( pair.first != NULL)
//				{
//					int idx_num = pair.second;
//					int idx_num_feats = feature_packet_from_vft->num_feats[idx_num];
//
//					feat_set_multi[idx_num].features.resize(n_total_features);
//					feat_set_zero.features.resize(n_total_features);
//
//					feat_set_multi[idx_num].timestamp =  pair.first;
//					feat_set_zero.timestamp = pair.first;
//
//					for (int i=0; i<idx_num_feats; i++)
//					{
//						int ofs = idx_num*idx_num_feats+i;
//
//						feat_set_multi[idx_num].cam_id = pair.second;
//						feat_set_multi[idx_num].features[i].cam_id = pair.second;
//						feat_set_multi[idx_num].features[i].id = features_from_vft[ofs].id;
//						feat_set_multi[idx_num].features[i].u = features_from_vft[ofs].x;
//						feat_set_multi[idx_num].features[i].v = features_from_vft[ofs].y;  // TODO subtract from height?
//						memcpy(feat_set_multi[idx_num].features[i].descriptor, features_from_vft[ofs].descriptor, 32);
//
//						if (idx_num == 0)
//						{
//							feat_set_zero.cam_id = pair.second;
//							feat_set_zero.features[i].cam_id = pair.second;
//							feat_set_zero.features[i].id = features_from_vft[ofs].id;
//							feat_set_zero.features[i].u = features_from_vft[ofs].x;
//							feat_set_zero.features[i].v = features_from_vft[ofs].y;  // TODO subtract from height?
//							memcpy(feat_set_zero.features[i].descriptor, features_from_vft[ofs].descriptor, 32);
//						}
//					}
//					if (idx_num < 2)
//					{
//						std::lock_guard < std::mutex > lck(feature_queue_mtx);
//						feature_queue.push_back(feat_set_multi[idx_num]);
//					}
//				}
//				else
//				{
//					printf("PAIR IS NULL\n");
//				}
//		    }
//		}
	}
	else
	{
		double cam_time =  (double) feature_packet_from_vft->timestamp_ns[0] * 1e-9;
		// if idle for a long time, update the zero state by forcing a reset
		if (cam_time - feat_set_zero.timestamp > 30.0)
		{
			printf("Sitting around for a long time, resetting zero state for blind takeoff\n");
			init_failure_detector_reset_flag = 1;
		}

		for (int i = 0; i < (int) feat_set_zero.features.size(); i++)
		{
			feat_set.timestamp = cam_time;
			feat_set.cam_id = feat_set_zero.cam_id;
			feat_set.features[i].cam_id = feat_set_zero.features[i].cam_id;
			feat_set.features[i].id = feat_set_zero.features[i].id;
			feat_set.features[i].u = feat_set_zero.features[i].u;
			feat_set.features[i].v = feat_set_zero.features[i].v;
			memcpy(feat_set.features[i].descriptor, feat_set_zero.features[i].descriptor, 32);
		}
		std::lock_guard < std::mutex > lck(feature_queue_mtx);

		feature_queue.push_back(feat_set);
	}

	is_cam_connected = true;
}


#ifdef PROD
static void _new_feat_data_default_handler(__attribute__((unused)) int ch, char* data, int bytes,
                       __attribute__((unused)) void* context) 
{
	if (validate_vft_data(ch, data, bytes, &feature_packet_from_vft, &features_from_vft)) {
	    printf("ERROR parsing vft data from pipe...\n");
		return;
	}
	

	// reorganize


	double set_time = (double) feature_packet_from_vft->timestamp_ns[0] * 1e-9;
			
	static double last_set_time = set_time;
	static int n_total_features = 0;

	double dt_f = set_time-last_set_time;
	last_set_time = set_time;

	if (dt_f == 0)
		return;

	if (is_resetting)
		return;
	
	// testing logic
	int test_offset = 0;
	if (pause_cam_states[0] || pause_cam_states[1])
	{
		if (pause_cam_states[0])
		{
			test_offset = feature_packet_from_vft->num_feats[0];
			feature_packet_from_vft->num_feats[0] = 0;
		}
		if (pause_cam_states[1])
		{
			feature_packet_from_vft->num_feats[1] = 0;
		}
	}

	
	if (!vio_manager->initialized() || imu_moved || en_vio_always_on)
	{
		n_total_features = 0;
		
		if (use_takeoff_cam && (is_armed || en_vio_always_on) && (double) alt_z < takeoff_threshold) // turn off, we are in the air
		{
			if (!en_vio_always_on)
				printf(
					"Detected Takeoff, going to multicamera VINS normal operations\n");
			use_takeoff_cam = false;
		}			
		
		int n_cams = feature_packet_from_vft->n_cams;
				
		for (int i = 0; i < n_cams; i++) {
			if (use_takeoff_cam && i == takeoff_cam)
			{
				n_total_features = 0;
				n_total_features = feature_packet_from_vft->num_feats[i];
				break;
			}
			else
			{
				n_total_features += feature_packet_from_vft->num_feats[i];
			}
		}
  
		int cam_num = 0;
		int ctn = 0;
		bool has_feats = 0;
		
		feat_set.features.resize(n_total_features);
		feat_set_zero.features.resize(n_total_features);
		
		std::map<double, int> ts_map;

		for (int i = 0; i < n_total_features; i++)
		{
			if (use_takeoff_cam && cam_num != takeoff_cam)
			{
				continue;
			}

			if (ctn >= feature_packet_from_vft->num_feats[cam_num])  
			{
				ctn = 0;
				cam_num++;
			}

			double ts_cam =  (double) feature_packet_from_vft->timestamp_ns[cam_num] * 1e-9;
            ts_map[ts_cam] = cam_num;

			int tmp_offset = test_offset + i;
//			feat_set.timestamp = cam_time;
			feat_set.cam_id = cam_num;
			feat_set.features[i].cam_id = cam_num;
			feat_set.features[i].id = features_from_vft[tmp_offset].id;
			feat_set.features[i].u = features_from_vft[tmp_offset].x;
			feat_set.features[i].v = features_from_vft[tmp_offset].y;  // TODO subtract from height?
			memcpy(feat_set.features[i].descriptor, features_from_vft[tmp_offset].descriptor, 32);
			
//			feat_set_zero.timestamp = cam_time;
			feat_set_zero.cam_id = cam_num;
			feat_set_zero.features[i].cam_id = cam_num;
			feat_set_zero.features[i].id = features_from_vft[tmp_offset].id;
			feat_set_zero.features[i].u = features_from_vft[tmp_offset].x;
			feat_set_zero.features[i].v = features_from_vft[tmp_offset].y;  // TODO subtract from height?
			memcpy(feat_set_zero.features[i].descriptor, features_from_vft[tmp_offset].descriptor, 32);
			
			ctn++;
		}        

		double cam_time =  (double) feature_packet_from_vft->timestamp_ns[0] * 1e-9;
		if (cam_num < 1)
		{
			feat_set.timestamp = cam_time;
			feat_set_zero.timestamp = cam_time;
			std::lock_guard < std::mutex > lck(feature_queue_mtx);
			feature_queue.push_back(feat_set);
		}
		else
		{

	        	auto lastElement = ts_map.rbegin();
//	       		  std::cout << "Last Key: " << lastElement->first << ", Last Value: " << lastElement->second << std::endl;

			feat_set.timestamp = lastElement->first;
			feat_set_zero.timestamp = lastElement->first;
			std::lock_guard < std::mutex > lck(feature_queue_mtx);
			feature_queue.push_back(feat_set);

			printf("ts %f  \t", last_imu_time);
			for(const auto& pair : ts_map) {
		        std::cout << "Key: " << pair.first << "(" << pair.first-last_imu_time << ")" << ", Value: " << pair.second << "\t";
//				feat_set.timestamp = pair.first;
//				feat_set_zero.timestamp = pair.first;
//				feature_queue.push_back(feat_set);
		    }

			std::cout << "\n^^^" << std::endl;

		}


	}
	else
	{
		double cam_time =  (double) feature_packet_from_vft->timestamp_ns[0] * 1e-9;

		// if idle for a long time, update the zero state by forcing a reset
		if (cam_time - feat_set_zero.timestamp > 30.0)
		{
			printf("Sitting around for a long time, resetting zero state for blind takeoff\n");
			init_failure_detector_reset_flag = 1;
		}
		
		for (int i = 0; i < (int) feat_set_zero.features.size(); i++)
		{
			
			feat_set.timestamp = cam_time;							
			feat_set.cam_id = feat_set_zero.cam_id;
			feat_set.features[i].cam_id = feat_set_zero.features[i].cam_id;
			feat_set.features[i].id = feat_set_zero.features[i].id;
			feat_set.features[i].u = feat_set_zero.features[i].u;
			feat_set.features[i].v = feat_set_zero.features[i].v;  
			memcpy(feat_set.features[i].descriptor, feat_set_zero.features[i].descriptor, 32);
		}
		std::lock_guard < std::mutex > lck(feature_queue_mtx);	

		feature_queue.push_back(feat_set);
	}

	is_cam_connected = true;
}
#endif


int connect_vft_service(void)
{
	// connect to our feature tracker
	pipe_client_set_disconnect_cb(FEATURE_CH, _vft_disconnect_cb, NULL);
	pipe_client_set_simple_helper_cb(FEATURE_CH, _vft_data_default_handler, NULL);

	// allocate memory for vft data
	create_vft_memory(&feature_packet_from_vft, &features_from_vft);
	
	// open client pipe
	if (pipe_client_open(FEATURE_CH, vft_name, PROCESS_NAME, EN_PIPE_CLIENT_SIMPLE_HELPER /*CLIENT_FLAG_EN_SIMPLE_HELPER*/, 
			sizeof(vft_feature_packet)) != 0)
	{
		printf("failed to open vft client pipe\n");
		return -1;
	}

	printf("VFT connected\n");

	return 0;
}