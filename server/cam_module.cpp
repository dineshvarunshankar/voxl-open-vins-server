#include "shared_vars.h"
#include "cam_config_file.h"
#include "cam_module.h"
#include "img_ringbuffer.h"

#include <stdio.h>

int64_t last_cam_time;



// helper callback for cams we are using in the system
static void _cam_helper_cb(__attribute__((unused)) int ch,
		camera_image_metadata_t meta, char *frame, void *context)
{

	// MPA sends in async mode when connected to multiple camera pipes
	// this will hold state per image until it is added to
	// the vio stack.
	std::lock_guard < std::mutex > cam_lg(cam_lock_mutex);

	if (en_ext_feature_tracker && !overlay_client_connected) {
		return;
	}

	static int idler_ctn = 0;

	if (is_resetting)
	{
		return;
	}

	if (!en_ext_feature_tracker)
		is_cam_connected = true;

	// camera working, reset errors
	global_error_codes &= ~ERROR_CODE_CAM_MISSING;

	if (!is_imu_connected || !main_running || image_update_running
			|| resume_processing >= 1)
	{
		if (image_update_running)
		{
			is_initialized = true;
		}
		else
		{
			is_initialized = false;
		}

		std::shared_ptr < ov_msckf::State > current_state =
				vio_manager->get_state(); // contains a few extra pieces we need

		s.timestamp_ns = static_cast<int64_t>(current_state->_timestamp * 1e9);
		s.error_code |= ERROR_CODE_DROPPED_IMU;

 		memcpy(&d.v, &s, sizeof(vio_data_t));
		// send to both pipes
 		if (pipe_server_get_num_clients(EXTENDED_CH) > 0) // publish
 			pipe_server_write(EXTENDED_CH, (char*) &d, sizeof(ext_vio_data_t));

 		if (pipe_server_get_num_clients(SIMPLE_CH) > 0) // publish
 			pipe_server_write(SIMPLE_CH, (char*) &s, sizeof(vio_data_t));
		
		// TODO replace with histogram analysis
		if (resume_processing++ > 12)
		{
			if(en_debug)
				printf("[THERMAL] Resume processing done\n");
			
			resume_processing = 0;
		}

		return;
	}


	if (!en_ext_feature_tracker && meta.format == IMAGE_FORMAT_STEREO_RAW8)
	{
		if (idler_limit > 0 && idler_ctn++ < idler_limit)
		{
			last_cam_time = _apps_time_monotonic_ns();
			return;
		}
		else
		{
			idler_ctn = 0;
		}
	}


	camera_mode *cm = (camera_mode*) context;

	static img_ringbuf_packet curr_message_static;
	img_ringbuf_packet *curr_message = &curr_message_static;
//	img_ringbuf_packet *curr_message = new img_ringbuf_packet;
	curr_message->camid = ch;
	curr_message->metadata = meta;
	
	if (en_debug_timing_cam)
	{		
		if (cameras_used > 1)
		{
			static double cams_ts[2];
			static int ctn_cams_ts[2];
			
			printf("Cam %d ts: %f", ch, curr_message->metadata.timestamp_ns * 1e-09);

			if ((ctn_cams_ts[0] + ctn_cams_ts[1]) >= 2)  // todo add 3rd cam
			{
				printf(" Ch1 vs Ch2 delta: %f (ms)\n", (cams_ts[1] - cams_ts[0])* 1e3);
				ctn_cams_ts[0] = 0;
				ctn_cams_ts[1] = 0;
				cams_ts[0] = 0;
				cams_ts[1] = 0;
			}
			else
				printf("\n");
			
			ctn_cams_ts[ch-1] = 1;
			cams_ts[ch-1] = curr_message->metadata.timestamp_ns * 1e-09;
		}

	}
	
	try
	{
		if (meta.format == IMAGE_FORMAT_RAW8)
		{
			if (cameras_used == 1)
			{
				memcpy(curr_message->image_pixels, (uint8_t*) frame,
						meta.size_bytes);			
				img_ringbuf->insert_data(curr_message);

				if (en_ext_feature_tracker) {
					return;
				}
				
				cv::Mat internal_img(meta.height, meta.width, CV_8UC1,
					(uint8_t*) curr_message->image_pixels);
	
				ov_core::CameraData message;
				message.timestamp = curr_message->metadata.timestamp_ns * 1e-09;
				message.sensor_ids.push_back(0);

				//TODO handling exception!
				static cv::Mat zero_image = imread("/data/modalai/ov/zero_ref.jpg", cv::IMREAD_GRAYSCALE);
				static cv::Mat ignore_mask(internal_img.rows,
						internal_img.cols, CV_8UC1, cv::Scalar(255));
				static cv::Mat use_mask(internal_img.rows,
						internal_img.cols, CV_8UC1, cv::Scalar(0));
	
				if (zero_image.rows != internal_img.rows)
					resize(zero_image, zero_image, cv::Size(internal_img.cols, internal_img.rows), cv::INTER_NEAREST);
	
				if (!vio_manager->initialized())
				{
					if (en_debug)
						printf("Initializing\n");
					
					message.images.push_back(zero_image);
				}
				else
				{
					if (!imu_moved)
					{
						if (en_debug)
							printf("Idling\n");
	
						if (!has_idle_images)
						{
							if (en_debug)
								printf("Capture new idle images\n");											
							idle_image1 = internal_img.clone();
							has_idle_images = true;
						}
						
						message.images.push_back(idle_image1);
					}
					else
					{
						message.images.push_back(internal_img.clone());
					}
				}
				
				if (use_takeoff_cam)
				{
					message.masks.push_back(use_mask);
					if (is_armed && (double) alt_z < takeoff_threshold) // turn off, we are in the air
					{
						//printf("Detected Takeoff, going to VINS normal operations\n");
						use_takeoff_cam = false;
	
						// if we're disarmed, and running load  throttling, turn off and go fullt throttle
						idler_limit = -1;
					}
				}
				else
				{
					message.masks.push_back(use_mask);
				}
	
				std::lock_guard < std::mutex > lck(camera_queue_mtx);
				camera_queue.push_back(message);
				std::sort(camera_queue.begin(), camera_queue.end());
			}
			else  // MULTICAM     //TBD refactor to conver single cam
			{

				static std::vector<double> ts_cam_vec(cameras_used);

				// combine and sim multi camera
				if (curr_message->camid == 1)
				{
					memcpy(slot_image_pixels, (uint8_t*) frame,
							meta.size_bytes);			
				    update_slots |= (1 << 7);
				    ts_cam_vec[0] = curr_message->metadata.timestamp_ns * 1e-09;
				}
				else if (curr_message->camid == 2)
				{
					memcpy(slot_image_pixels + meta.size_bytes, 
							(uint8_t*) frame,
							meta.size_bytes);			
				    update_slots |= (1 << 6);
				    ts_cam_vec[1] = curr_message->metadata.timestamp_ns * 1e-09;
				}
				else if (curr_message->camid == 3)
				{
					memcpy(slot_image_pixels + (2 * meta.size_bytes),
							(uint8_t*) frame,
							meta.size_bytes);
				    update_slots |= (1 << 5);
				    ts_cam_vec[2] = curr_message->metadata.timestamp_ns * 1e-09;
				}


//				if (((update_slots >> 7) & 1) && ((update_slots >> 6) & 1))
				// cam 2 combos
				bool two_cam = (update_slots == 192) || (update_slots == 160) ||  (update_slots == 96);

				if ((cameras_used == 2 && two_cam) || (cameras_used == 3 && update_slots == 224))
				{

					if (!en_ext_feature_tracker)
					{
						if (idler_limit > 0 && idler_ctn++ < idler_limit)
						{
							last_cam_time = _apps_time_monotonic_ns();
							return;
						}
						else
						{
							idler_ctn = 0;
						}
					}

					curr_message->camid = ch;
					curr_message->metadata = meta;
					memcpy(curr_message->image_pixels, (uint8_t*) slot_image_pixels,
							meta.size_bytes*cameras_used);			
					update_slots = 0;
					img_ringbuf->insert_data(curr_message);

					double avg_time = 0;
					for (const auto& _ts : ts_cam_vec)
					{
						avg_time += _ts;
					}
					avg_time /= cameras_used;

//					double avg_time = 0;
//					for (const auto& _ts : ts_cam_vec)
//					{
//						if (_ts > avg_time)
//						avg_time = _ts;
//					}

					ov_core::CameraData message;
					message.timestamp = avg_time;
//					message.timestamp = curr_message->metadata.timestamp_ns * 1e-09;
					message.sensor_ids.push_back(0);

					cv::Mat internal_img_1(meta.height, meta.width, CV_8UC1,
							(uint8_t*) slot_image_pixels);
					cv::Mat internal_img_2;
					cv::Mat internal_img_3;

					if (cameras_used >= 2)
					{
						message.sensor_ids.push_back(1);
						cv::Mat internal_img_tmp(meta.height, meta.width, CV_8UC1,
							(uint8_t*) slot_image_pixels + meta.size_bytes);
						internal_img_tmp.copyTo(internal_img_2);

					}

					if (cameras_used == 3)
					{
						message.sensor_ids.push_back(2);
						cv::Mat internal_img_tmp(meta.height, meta.width, CV_8UC1,
								(uint8_t*) slot_image_pixels + (2 * meta.size_bytes));
						internal_img_tmp.copyTo(internal_img_3);
					}


					// TODO dynamic aperture (AP)
	#ifdef DYN_AP_EXPERIMENTAL
					static cv::Mat idle_mask1(internal_img_1.rows,
							internal_img_1.cols, CV_8UC1, cv::Scalar(0));
	//				static int rectX1 = (internal_img_1.cols - 640) / 2;
	//				static int rectY1 = (internal_img_1.rows - 620) / 2;
	//				cv::rectangle(idle_mask1, cv::Rect(rectX1, rectY1, 640, 180), cv::Scalar(255), -1);
					cv::rectangle(idle_mask1, cv::Rect(426, 266, internal_img_1.cols-426, internal_img_1.rows-266), cv::Scalar(255), -1);

					static cv::Mat idle_mask2(internal_img_2.rows,
							internal_img_2.cols, CV_8UC1, cv::Scalar(0));
	//				static int rectX2 = (internal_img_2.cols -640 ) / 2;
	//				static int rectY2 = (internal_img_2.rows - 620) / 2;
	//				cv::rectangle(idle_mask2, cv::Rect(rectX2, rectY2, 640, 180), cv::Scalar(255), -1);
					cv::rectangle(idle_mask2, cv::Rect(426, 266, internal_img_2.cols-426, internal_img_2.rows-266), cv::Scalar(255), -1);
	#endif

					//TODO handling exception!
					static cv::Mat zero_image = imread("/data/modalai/ov/zero_ref.jpg", cv::IMREAD_GRAYSCALE);
					if (zero_image.rows != internal_img_1.rows)
						resize(zero_image, zero_image, cv::Size(internal_img_1.cols, internal_img_1.rows), cv::INTER_NEAREST);


					// these static things are bad cause they are typically the same, but just in care we start using completely different
					// cameras and mixing and matching, this will set up the framework.
					static cv::Mat ignore_mask1(internal_img_1.rows,
							internal_img_1.cols, CV_8UC1, cv::Scalar(255));
					static cv::Mat ignore_mask2(internal_img_2.rows,
							internal_img_2.cols, CV_8UC1, cv::Scalar(255));
					static cv::Mat ignore_mask3(internal_img_2.rows,
							internal_img_2.cols, CV_8UC1, cv::Scalar(255));

					static cv::Mat use_mask1(internal_img_1.rows,
							internal_img_1.cols, CV_8UC1, cv::Scalar(0));
					static cv::Mat use_mask2(internal_img_2.rows,
							internal_img_2.cols, CV_8UC1, cv::Scalar(0));
					static cv::Mat use_mask3(internal_img_2.rows,
							internal_img_2.cols, CV_8UC1, cv::Scalar(0));

					if (!vio_manager->initialized())
					{
						if (en_debug)
							printf("Initializing\n");
						message.images.push_back(zero_image);
						message.images.push_back(zero_image);
						if (cameras_used == 3)
							message.images.push_back(zero_image);
					}
					else
					{
						if (!imu_moved)
						{
							if (en_debug)
								printf("Idling\n");

							if (!has_idle_images)
							{
								if (en_debug)
									printf("Capture new idle images\n");
								idle_image1 = internal_img_1.clone();
								idle_image2 = internal_img_2.clone();
								if (cameras_used ==  3)
									idle_image3 = internal_img_3.clone();

								has_idle_images = true;
							}

							message.images.push_back(idle_image1);
							message.images.push_back(idle_image2);
							if (cameras_used == 3)
								message.images.push_back(idle_image3);
						}
						else
						{
							// use real time raw imagery
							message.images.push_back(internal_img_1.clone());
							message.images.push_back(internal_img_2.clone());
							if (cameras_used == 3)
								message.images.push_back(internal_img_3.clone());
						}
					}

					if (use_takeoff_cam)
					{
						if (takeoff_cam == 0)
						{
							if (en_debug)
								printf("in takeoff %f\n", (double)alt_z);
							message.masks.push_back(use_mask1);
							message.masks.push_back(ignore_mask1);
							if (cameras_used == 3)
								message.masks.push_back(ignore_mask1);

						}
						else if (takeoff_cam == 1)
						{
							if (en_debug)
								printf("in takeoff %f\n", (double)alt_z);
							message.masks.push_back(ignore_mask1);
							message.masks.push_back(use_mask1);
							if (cameras_used == 3)
								message.masks.push_back(ignore_mask1);

						}
						else
						{
							message.masks.push_back(use_mask1);
							message.masks.push_back(use_mask1);
							if (cameras_used == 3)
								message.masks.push_back(use_mask1);

						}

						if ((is_armed || en_vio_always_on) && (double) alt_z < takeoff_threshold) // turn off, we are in the air
						{
							//printf("Detected Takeoff, going to multicamera VINS normal operations\n");
							use_takeoff_cam = false;

							// if we're disarmed, and running load  throttling, turn off and go fullt throttle
							idler_limit = -1;
						}
					}
					else
					{
						if (pause_cam_states[0])
							message.masks.push_back(ignore_mask1);
						else
							message.masks.push_back(use_mask1);

						if (pause_cam_states[1])
							message.masks.push_back(ignore_mask1);
						else
							message.masks.push_back(use_mask1);

						if (cameras_used == 3)
						{
							if (pause_cam_states[2])
								message.masks.push_back(ignore_mask1);
							else
								message.masks.push_back(use_mask1);

						}
					}

					std::lock_guard < std::mutex > lck(camera_queue_mtx);
					camera_queue.push_back(message);
					std::sort(camera_queue.begin(), camera_queue.end());

				}

			}
			is_thermal = false;
			
		}
		else if (meta.format == IMAGE_FORMAT_STEREO_RAW8)
		{
			if (*cm == STEREO)
			{
				// NOTE: the stereo image comes in as a concatenated single image via vertical tiled (width = width, height = img_ctn * height)
//			printf("STEREO CH %d %f\n", ch, curr_message->metadata.timestamp_ns * 1e-09);
				memcpy(curr_message->image_pixels, (uint8_t*) frame,
						meta.size_bytes);

				img_ringbuf->insert_data(curr_message);
				
				cv::Mat internal_img_1(meta.height, meta.width, CV_8UC1,
						(uint8_t*) curr_message->image_pixels);
				cv::Mat internal_img_2(meta.height, meta.width, CV_8UC1,
						(uint8_t*) curr_message->image_pixels
								+ meta.size_bytes / 2);

				ov_core::CameraData message;
				message.timestamp = curr_message->metadata.timestamp_ns * 1e-09;
				message.sensor_ids.push_back(0);
				message.sensor_ids.push_back(1);

				// TODO dynamic aperture
#ifdef DYN_AP_EXPERIMENTAL
				static cv::Mat idle_mask1(internal_img_1.rows,
						internal_img_1.cols, CV_8UC1, cv::Scalar(0));
//				static int rectX1 = (internal_img_1.cols - 640) / 2;
//				static int rectY1 = (internal_img_1.rows - 620) / 2;
//				cv::rectangle(idle_mask1, cv::Rect(rectX1, rectY1, 640, 180), cv::Scalar(255), -1);  	
				cv::rectangle(idle_mask1, cv::Rect(426, 266, internal_img_1.cols-426, internal_img_1.rows-266), cv::Scalar(255), -1);  	
				
				static cv::Mat idle_mask2(internal_img_2.rows,
						internal_img_2.cols, CV_8UC1, cv::Scalar(0));
//				static int rectX2 = (internal_img_2.cols -640 ) / 2;
//				static int rectY2 = (internal_img_2.rows - 620) / 2;
//				cv::rectangle(idle_mask2, cv::Rect(rectX2, rectY2, 640, 180), cv::Scalar(255), -1);  	
				cv::rectangle(idle_mask2, cv::Rect(426, 266, internal_img_2.cols-426, internal_img_2.rows-266), cv::Scalar(255), -1);  	
#endif
				//TODO handling exception!
				static cv::Mat zero_image = imread("/data/modalai/ov/zero_ref.jpg", cv::IMREAD_GRAYSCALE);
				if (zero_image.rows != internal_img_1.rows)
					resize(zero_image, zero_image, cv::Size(internal_img_1.cols, internal_img_1.rows), cv::INTER_NEAREST);

				static cv::Mat ignore_mask1(internal_img_1.rows,
						internal_img_1.cols, CV_8UC1, cv::Scalar(255));
				static cv::Mat ignore_mask2(internal_img_2.rows,
						internal_img_2.cols, CV_8UC1, cv::Scalar(255));
				
				static cv::Mat use_mask1(internal_img_1.rows,
						internal_img_1.cols, CV_8UC1, cv::Scalar(0));
				static cv::Mat use_mask2(internal_img_2.rows,
						internal_img_2.cols, CV_8UC1, cv::Scalar(0));

				if (!vio_manager->initialized())
				{
					if (en_debug)
						printf("Initializing\n");
					
					message.images.push_back(zero_image);
					message.images.push_back(zero_image);
				}
				else 
				{
					if (!imu_moved)
					{
						if (en_debug)
							printf("Idling\n");

						if (!has_idle_images)
						{
							if (en_debug)
								printf("Capture new idle images\n");											
							idle_image1 = internal_img_1.clone();
							idle_image2 = internal_img_2.clone();													
							has_idle_images = true;
						}
						
						message.images.push_back(idle_image1);
						message.images.push_back(idle_image2);	
					}
					else
					{
						// use real time raw imagery
						message.images.push_back(internal_img_1.clone());
						message.images.push_back(internal_img_2.clone());
					}
				}
				
				if (use_takeoff_cam)
				{
					if (takeoff_cam == 0)
					{
						if (en_debug)
							printf("in takeoff %f\n", (double)alt_z);
						message.masks.push_back(use_mask1);
						message.masks.push_back(ignore_mask1);
					}
					else if (takeoff_cam == 1)
					{
						if (en_debug)
							printf("in takeoff %f\n", (double)alt_z);
						message.masks.push_back(ignore_mask2);
						message.masks.push_back(use_mask2);
					}
					else
					{
						message.masks.push_back(use_mask1);
						message.masks.push_back(use_mask2);
					}

					if ((is_armed || en_vio_always_on) && (double) alt_z < takeoff_threshold) // turn off, we are in the air
					{
						//printf("Detected Takeoff, going to multicamera VINS normal operations\n");
						use_takeoff_cam = false;

						// if we're disarmed, and running load  throttling, turn off and go fullt throttle
						idler_limit = -1;
					}
				}
				else
				{
					message.masks.push_back(use_mask1);
					message.masks.push_back(use_mask2);
				}

				std::lock_guard < std::mutex > lck(camera_queue_mtx);
				camera_queue.push_back(message);
				std::sort(camera_queue.begin(), camera_queue.end());

				is_thermal = false;

			}
			else //MONO
			{
				memcpy(curr_message->image_pixels, (uint8_t*) frame,
						meta.size_bytes);

				img_ringbuf->insert_data(curr_message);
				
				int offset = 0;
				// if (single_cam_in_use == 1)
				// {
				// 	offset = meta.size_bytes / 2;
				// }
				cv::Mat internal_img(meta.height, meta.width, CV_8UC1,
						(uint8_t*) curr_message->image_pixels + offset);
				
				ov_core::CameraData message;
				message.timestamp = curr_message->metadata.timestamp_ns * 1e-09;
				message.sensor_ids.push_back(0);

				//TODO handling exception!
				static cv::Mat zero_image = imread("/data/modalai/ov/zero_ref.jpg", cv::IMREAD_GRAYSCALE);
				if (zero_image.rows != internal_img.rows)
					resize(zero_image, zero_image, cv::Size(internal_img.cols, internal_img.rows), cv::INTER_NEAREST);

				static cv::Mat ignore_mask(internal_img.rows,
						internal_img.cols, CV_8UC1, cv::Scalar(255));
				
				static cv::Mat use_mask(internal_img.rows,
						internal_img.cols, CV_8UC1, cv::Scalar(0));

				if (!vio_manager->initialized())
				{
					if (en_debug)
						printf("Initializing\n");
					
					message.images.push_back(zero_image);
				}
				else
				{
					if (!imu_moved)
					{
						if (en_debug)
							printf("Idling\n");

						if (!has_idle_images)
						{
							if (en_debug)
								printf("Capture new idle images\n");											
							idle_image1 = internal_img.clone();
							has_idle_images = true;
						}
						
						message.images.push_back(idle_image1);
					}
					else
					{
						message.images.push_back(internal_img.clone());
					}
				}
				
				if (use_takeoff_cam)
				{
					message.masks.push_back(use_mask);
					if ((is_armed || en_vio_always_on) && (double) alt_z < takeoff_threshold) // turn off, we are in the air
					{
						//printf("Detected Takeoff, going to multicamera VINS normal operations\n");
						use_takeoff_cam = false;

						// if we're disarmed, and running load  throttling, turn off and go fullt throttle
						idler_limit = -1;
					}
				}
				else
				{
					message.masks.push_back(use_mask);
				}

				std::lock_guard < std::mutex > lck(camera_queue_mtx);
				camera_queue.push_back(message);
				std::sort(camera_queue.begin(), camera_queue.end());

				is_thermal = false;
				
			}
		}


		///////////////////////////////////////////////////////////////////////////////////
		// SPECIAL MODE
		// SPECIAL MODE  SEEK + BOSON
		// SPECIAL MODE  SEEK + BOSON
		// SPECIAL MODE
        else if (cameras_used == 1 && meta.format == IMAGE_FORMAT_NV12)  // boson
		{
			static int zz = 0;
			if (zz++ % 2 == 0)
				return;

			cv::Mat internal_img(meta.height, meta.width, CV_8UC1, (uchar*)frame);

		//			static double minVal, maxVal;
		//            if (!is_armed)
		//            {
		//            	if (zz < INIT_TIMEOUT_FRAMES)
		//            		minMaxLoc(internal_img, &minVal, &maxVal);
		//            }
		//
		//            if (en_debug)
		//            {
		//				printf("Max: %f Min: %f\n", maxVal, minVal);
		//            }
		//
		//			internal_img.setTo(0, internal_img < minVal/2);

			if (thermal_brightness_bos != 1.00)
			{
				internal_img.convertTo(internal_img, CV_16UC1, thermal_brightness_bos);
				cv::threshold(internal_img, internal_img, 255, 255, cv::THRESH_TRUNC);
			}

			if (en_thermal_enhance)
			{

				static cv::Mat kernel = (cv::Mat_<float>(3, 3) <<
						-1,  -0,  1,
						-0,   0,  0,
						 1,   0,  1);

				cv::Mat embossed;
				cv::filter2D(internal_img, embossed, -1, kernel);
		//				cv::filter2D(internal_img, embossed, CV_32F, kernel);

				// Normalize the result to 8-bit for display
				cv::normalize(embossed, embossed, 0, 255, cv::NORM_MINMAX, CV_8UC1);
				//embossed.convertTo(internal_img, CV_8UC1);
				internal_img = embossed;
			}


			memcpy(curr_message->image_pixels, internal_img.data,
					internal_img.total() * internal_img.elemSize());

			img_ringbuf->insert_data(curr_message);

			ov_core::CameraData message;
			message.timestamp = curr_message->metadata.timestamp_ns * 1e-09;
			message.sensor_ids.push_back(0);


			//TODO handling exception!
			static cv::Mat zero_image = imread("/data/modalai/ov/zero_ref.jpg", cv::IMREAD_GRAYSCALE);
			static cv::Mat ignore_mask(internal_img.rows,
					internal_img.cols, CV_8UC1, cv::Scalar(255));
			static cv::Mat use_mask(internal_img.rows,
					internal_img.cols, CV_8UC1, cv::Scalar(0));

			if (zero_image.rows != internal_img.rows)
				resize(zero_image, zero_image, cv::Size(internal_img.cols, internal_img.rows), cv::INTER_NEAREST);

			if (!vio_manager->initialized())
			{
				if (en_debug)
					printf("Initializing\n");

				message.images.push_back(zero_image);
			}
			else
			{
				if (!imu_moved)
				{
					if (en_debug)
						printf("Idling\n");

					if (!has_idle_images)
					{
						if (en_debug)
							printf("Capture new idle images\n");
						idle_image1 = internal_img.clone();
						has_idle_images = true;
					}

					message.images.push_back(idle_image1);
				}
				else
				{
					message.images.push_back(internal_img.clone());
				}
			}

			if (use_takeoff_cam)
			{
				message.masks.push_back(use_mask);
				if (is_armed && (double) alt_z < takeoff_threshold) // turn off, we are in the air
				{
					printf(
							"Detected Takeoff, going to VINS normal operations\n");
					use_takeoff_cam = false;

					// if we're disarmed, and running load  throttling, turn off and go fullt throttle
					idler_limit = -1;
				}
			}
			else
			{
				message.masks.push_back(use_mask);
			}

			std::lock_guard < std::mutex > lck(camera_queue_mtx);
			camera_queue.push_back(message);
			std::sort(camera_queue.begin(), camera_queue.end());
			is_thermal = false;

		}
        else if (cameras_used == 1 && meta.format == IMAGE_FORMAT_RAW16)  // seek
		{
			cv::Mat internal_img;

			cv::Mat raw16Image(meta.height, meta.width, CV_16UC1, (uint8_t*) frame);

			if (thermal_brightness != 1.00)
			{
				raw16Image.convertTo(raw16Image, CV_16UC1, thermal_brightness);
				cv::threshold(raw16Image, raw16Image, 65535, 65535, cv::THRESH_TRUNC);
			}

			if (en_thermal_enhance)
			{

		//            	static cv::Mat kernel = (cv::Mat_<float>(3, 3) <<
		//                      -4, -2, 1,
		//                      -2,  1, 3,
		//                       1,  3, 4);

		//            	static cv::Mat kernel = (cv::Mat_<float>(3, 3) <<
		//            			-3,  -1,  0,
		//            			-1,   1,  1,
		//            			 0,   1,  3);

				static cv::Mat kernel = (cv::Mat_<float>(3, 3) <<
						-3,  -1,  3,
						-1,   1,  1,
						 3,   1,  3);


				// Apply the filter
				cv::Mat embossed;
				cv::filter2D(raw16Image, embossed, CV_32F, kernel);

				// Normalize the result to 8-bit for display
				cv::normalize(embossed, embossed, 0, 255, cv::NORM_MINMAX);
				embossed.convertTo(internal_img, CV_8UC1);

		//				internal_img = ~internal_img;
		//				static double minVal, maxVal;
		//	            if (!is_armed)
		//	            {
		//	            	if (zz++ < INIT_TIMEOUT_FRAMES)
		//	            		minMaxLoc(internal_img, &minVal, &maxVal);
		//	            }
		//
		//	            if (en_debug)
		//	            {
		//					printf("Max: %f Min: %f\n", maxVal, minVal);
		//	            }
		//
		//
		//	            internal_img.setTo(0, internal_img < 8);
			}
			else
			{
				raw16Image.convertTo(internal_img, CV_8UC1, 1.0 / 256.0); // Scaling to fit into 8-bit
			}


			memcpy(curr_message->image_pixels, internal_img.data,
									internal_img.total() * internal_img.elemSize());
			img_ringbuf->insert_data(curr_message);

			ov_core::CameraData message;
			message.timestamp = curr_message->metadata.timestamp_ns * 1e-09;
			message.sensor_ids.push_back(0);

			//TODO handling exception!
			static cv::Mat zero_image = imread("/data/modalai/ov/zero_ref.jpg", cv::IMREAD_GRAYSCALE);
			static cv::Mat ignore_mask(internal_img.rows,
					internal_img.cols, CV_8UC1, cv::Scalar(255));
			static cv::Mat use_mask(internal_img.rows,
					internal_img.cols, CV_8UC1, cv::Scalar(0));

			if (zero_image.rows != internal_img.rows)
				resize(zero_image, zero_image, cv::Size(internal_img.cols, internal_img.rows), cv::INTER_NEAREST);

			if (!vio_manager->initialized())
			{
				if (en_debug)
					printf("Initializing\n");

				message.images.push_back(zero_image);
			}
			else
			{
				if (!imu_moved)
				{
					if (en_debug)
						printf("Idling\n");

					if (!has_idle_images)
					{
						if (en_debug)
							printf("Capture new idle images\n");
						idle_image1 = internal_img.clone();
						has_idle_images = true;
					}

					message.images.push_back(idle_image1);
				}
				else
				{
					message.images.push_back(internal_img.clone());
				}
			}

			if (use_takeoff_cam)
			{
				message.masks.push_back(use_mask);
				if (is_armed && (double) alt_z < takeoff_threshold) // turn off, we are in the air
				{
					printf(
							"Detected Takeoff, going to VINS normal operations\n");
					use_takeoff_cam = false;

					// if we're disarmed, and running load  throttling, turn off and go fullt throttle
					idler_limit = -1;
				}
			}
			else
			{
				message.masks.push_back(use_mask);
			}

			std::lock_guard < std::mutex > lck(camera_queue_mtx);
			camera_queue.push_back(message);
			std::sort(camera_queue.begin(), camera_queue.end());
			is_thermal = false;

		}
		else if (cameras_used == 2 && (meta.format == IMAGE_FORMAT_NV12 || meta.format == IMAGE_FORMAT_RAW16))  // boson+seek
		{

			static std::vector<double> ts_cam_vec(cameras_used);
			cv::Mat internal_img;

			if (meta.format == IMAGE_FORMAT_NV12)
			{
				static int zz = 0;
				if (zz++ % 2 == 0)
					return;

				cv::Mat internal_img_i(meta.height, meta.width, CV_8UC1, (uchar*)frame);

				if (thermal_brightness_bos != 1.00)
				{
					internal_img_i.convertTo(internal_img_i, CV_16UC1, thermal_brightness_bos);
					cv::threshold(internal_img_i, internal_img_i, 255, 255, cv::THRESH_TRUNC);
				}

				if (en_thermal_enhance)
				{
					static cv::Mat kernel = (cv::Mat_<float>(3, 3) <<
							-1,  -0,  1,
							-0,   0,  0,
							 1,   0,  1);

					cv::Mat embossed;
					cv::filter2D(internal_img_i, embossed, -1, kernel);
					cv::normalize(embossed, embossed, 0, 255, cv::NORM_MINMAX, CV_8UC1);
					internal_img_i = embossed;
				}

				// 640 rez
				//internal_img = internal_img_i.clone();

			    // Resize the image to 320x240
			    cv::resize(internal_img_i, internal_img, cv::Size(320, 240), cv::INTER_NEAREST);

	            size_t byteSize = internal_img.total() * internal_img.elemSize();
	            meta.height = 240;
	            meta.width = 320;
	            meta.size_bytes = byteSize;
	        	curr_message->metadata = meta;

			}
			else if (meta.format == IMAGE_FORMAT_RAW16)
			{
				cv::Mat internal_img_i;
				cv::Mat raw16Image(meta.height, meta.width, CV_16UC1, (uint8_t*) frame);

				if (thermal_brightness != 1.00)
				{
					raw16Image.convertTo(raw16Image, CV_16UC1, thermal_brightness);
					cv::threshold(raw16Image, raw16Image, 65535, 65535, cv::THRESH_TRUNC);
				}

				if (en_thermal_enhance)
				{
					static cv::Mat kernel = (cv::Mat_<float>(3, 3) <<
							-3,  -1,  3,
							-1,   1,  1,
							 3,   1,  3);

					cv::Mat embossed;
					cv::filter2D(raw16Image, embossed, CV_32F, kernel);
					cv::normalize(embossed, embossed, 0, 255, cv::NORM_MINMAX);
					embossed.convertTo(internal_img_i, CV_8UC1);
				}
				else
				{
					raw16Image.convertTo(internal_img_i, CV_8UC1, 1.0 / 255.0); // Scaling to fit into 8-bit
				}

				internal_img = internal_img_i.clone();
			}


			size_t byteSize = internal_img.total() * internal_img.elemSize();
			meta.size_bytes = byteSize;

			// combine and sim multi camera
			if (curr_message->camid == 1)
			{
				memcpy(slot_image_pixels, (uint8_t*) internal_img.data,
						meta.size_bytes);
			    update_slots |= (1 << 7);
			    ts_cam_vec[0] = curr_message->metadata.timestamp_ns * 1e-09;
			}
			else if (curr_message->camid == 2)
			{
				memcpy(slot_image_pixels + meta.size_bytes,
						(uint8_t*) internal_img.data,
						meta.size_bytes);
			    update_slots |= (1 << 6);
			    ts_cam_vec[1] = curr_message->metadata.timestamp_ns * 1e-09;
			}
			else if (curr_message->camid == 3)
			{
				memcpy(slot_image_pixels + (2 * meta.size_bytes),
						(uint8_t*) internal_img.data,
						meta.size_bytes);
			    update_slots |= (1 << 5);
			    ts_cam_vec[2] = curr_message->metadata.timestamp_ns * 1e-09;
			}


//				if (((update_slots >> 7) & 1) && ((update_slots >> 6) & 1))
			// cam 2 combos
			bool two_cam = (update_slots == 192) || (update_slots == 160) ||  (update_slots == 96);

			if ((cameras_used == 2 && two_cam) || (cameras_used == 3 && update_slots == 224))
			{

				if (!en_ext_feature_tracker)
				{
					if (idler_limit > 0 && idler_ctn++ < idler_limit)
					{
						last_cam_time = _apps_time_monotonic_ns();
						return;
					}
					else
					{
						idler_ctn = 0;
					}
				}

				curr_message->camid = ch;
				curr_message->metadata = meta;
				memcpy(curr_message->image_pixels, (uint8_t*) slot_image_pixels,
						meta.size_bytes*cameras_used);
				update_slots = 0;
				img_ringbuf->insert_data(curr_message);

				double avg_time = 0;
				for (const auto& _ts : ts_cam_vec)
				{
					avg_time += _ts;
				}
				avg_time /= cameras_used;

//					double avg_time = 0;
//					for (const auto& _ts : ts_cam_vec)
//					{
//						if (_ts > avg_time)
//						avg_time = _ts;
//					}

				ov_core::CameraData message;
				message.timestamp = avg_time;
//					message.timestamp = curr_message->metadata.timestamp_ns * 1e-09;
				message.sensor_ids.push_back(0);

				cv::Mat internal_img_1(meta.height, meta.width, CV_8UC1,
						(uint8_t*) slot_image_pixels);
				cv::Mat internal_img_2;
				cv::Mat internal_img_3;

				if (cameras_used >= 2)
				{
					message.sensor_ids.push_back(1);
					cv::Mat internal_img_tmp(meta.height, meta.width, CV_8UC1,
						(uint8_t*) slot_image_pixels + meta.size_bytes);
					internal_img_tmp.copyTo(internal_img_2);

				}

				if (cameras_used == 3)
				{
					message.sensor_ids.push_back(2);
					cv::Mat internal_img_tmp(meta.height, meta.width, CV_8UC1,
							(uint8_t*) slot_image_pixels + (2 * meta.size_bytes));
					internal_img_tmp.copyTo(internal_img_3);
				}


				//TODO handling exception!
				static cv::Mat zero_image = imread("/data/modalai/ov/zero_ref.jpg", cv::IMREAD_GRAYSCALE);
				if (zero_image.rows != internal_img_1.rows)
					resize(zero_image, zero_image, cv::Size(internal_img_1.cols, internal_img_1.rows), cv::INTER_NEAREST);


				// these static things are bad cause they are typically the same, but just in care we start using completely different
				// cameras and mixing and matching, this will set up the framework.
				static cv::Mat ignore_mask1(internal_img_1.rows,
						internal_img_1.cols, CV_8UC1, cv::Scalar(255));
				static cv::Mat ignore_mask2(internal_img_2.rows,
						internal_img_2.cols, CV_8UC1, cv::Scalar(255));
				static cv::Mat ignore_mask3(internal_img_2.rows,
						internal_img_2.cols, CV_8UC1, cv::Scalar(255));

				static cv::Mat use_mask1(internal_img_1.rows,
						internal_img_1.cols, CV_8UC1, cv::Scalar(0));
				static cv::Mat use_mask2(internal_img_2.rows,
						internal_img_2.cols, CV_8UC1, cv::Scalar(0));
				static cv::Mat use_mask3(internal_img_2.rows,
						internal_img_2.cols, CV_8UC1, cv::Scalar(0));

				if (!vio_manager->initialized())
				{
					if (en_debug)
						printf("Initializing\n");
					message.images.push_back(zero_image);
					message.images.push_back(zero_image);
					if (cameras_used == 3)
						message.images.push_back(zero_image);
				}
				else
				{
					if (!imu_moved)
					{
						if (en_debug)
							printf("Idling\n");

						if (!has_idle_images)
						{
							if (en_debug)
								printf("Capture new idle images\n");
							idle_image1 = internal_img_1.clone();
							idle_image2 = internal_img_2.clone();
							if (cameras_used ==  3)
								idle_image3 = internal_img_3.clone();

							has_idle_images = true;
						}

						message.images.push_back(idle_image1);
						message.images.push_back(idle_image2);
						if (cameras_used == 3)
							message.images.push_back(idle_image3);
					}
					else
					{
						// use real time raw imagery
						message.images.push_back(internal_img_1.clone());
						message.images.push_back(internal_img_2.clone());
						if (cameras_used == 3)
							message.images.push_back(internal_img_3.clone());
					}
				}

				if (use_takeoff_cam)
				{
					if (takeoff_cam == 0)
					{
						if (en_debug)
							printf("in takeoff %f\n", (double)alt_z);
						message.masks.push_back(use_mask1);
						message.masks.push_back(ignore_mask1);
						if (cameras_used == 3)
							message.masks.push_back(ignore_mask1);

					}
					else if (takeoff_cam == 1)
					{
						if (en_debug)
							printf("in takeoff %f\n", (double)alt_z);
						message.masks.push_back(ignore_mask1);
						message.masks.push_back(use_mask1);
						if (cameras_used == 3)
							message.masks.push_back(ignore_mask1);

					}
					else
					{
						message.masks.push_back(use_mask1);
						message.masks.push_back(use_mask1);
						if (cameras_used == 3)
							message.masks.push_back(use_mask1);

					}

					if ((is_armed || en_vio_always_on) && (double) alt_z < takeoff_threshold) // turn off, we are in the air
					{
						//printf("Detected Takeoff, going to multicamera VINS normal operations\n");
						use_takeoff_cam = false;

						// if we're disarmed, and running load  throttling, turn off and go fullt throttle
						idler_limit = -1;
					}
				}
				else
				{
					if (pause_cam_states[0])
						message.masks.push_back(ignore_mask1);
					else
						message.masks.push_back(use_mask1);

					if (pause_cam_states[1])
						message.masks.push_back(ignore_mask1);
					else
						message.masks.push_back(use_mask1);

					if (cameras_used == 3)
					{
						if (pause_cam_states[2])
							message.masks.push_back(ignore_mask1);
						else
							message.masks.push_back(use_mask1);

					}
				}

				std::lock_guard < std::mutex > lck(camera_queue_mtx);
				camera_queue.push_back(message);
				std::sort(camera_queue.begin(), camera_queue.end());

			}

			is_thermal = false;
		}


		// SPECIAL MODE
		// SPECIAL MODE  SEEK + BOSON
		// SPECIAL MODE  SEEK + BOSON
		// SPECIAL MODE
///////////////////////////////////////////////////////////////////////////////////


		last_cam_time = _apps_time_monotonic_ns();

	}
	catch (const std::out_of_range &e)
	{
		fprintf(stderr, "CAM Process error!\n");
	}

	thread_update_running = false;


	current_height = meta.height;
	current_width = meta.width;

//	delete curr_message;
}


int connect_cam_service(std::vector<camera_info> &cam_info_vec) {

	// connect to all configured cameras
	char t_cam_nam[256];

	fprintf(stderr, "Number of Cameras active: %d\n", cameras_used);
	std::vector < std::string > tmp_camera_pipe_names;

	for (int i = 0; i < cameras_used; i++)
	{
		int ch = pipe_client_get_next_available_channel();
		camera_pipe_channels[i] = ch;

		fprintf(stderr, "Camera merge --- > ch: %d to cam id: %d\n", ch, i);

		sprintf(t_cam_nam, "%s", cam_info_vec[i].tracking_name);

		if (std::find(tmp_camera_pipe_names.begin(),
				tmp_camera_pipe_names.end(), t_cam_nam)
				== tmp_camera_pipe_names.end())
		{
			// pipe_client_set_disconnect_cb(FEAT_OVERLAY_CH, _imu_disconnect_cb, NULL);
			pipe_client_set_camera_helper_cb(ch, _cam_helper_cb, &cam_info_vec[i].mode);
			int flags = CLIENT_FLAG_EN_CAMERA_HELPER;
			int ret = pipe_client_open(ch, t_cam_nam, PROCESS_NAME, flags,
					1280 * 800 * 15);
			if (ret)
			{
				fprintf(stderr, "failed to open %s\n", cam_info_vec[i].tracking_name);
				return -1;
			}
			else
			{
				fprintf(stderr, "Opening camera pipe: %s\n",
						cam_info_vec[i].tracking_name);
			}

			fprintf(stderr, "tmp_camera_pipe_names.push_back(): %s\n",
					t_cam_nam);

			tmp_camera_pipe_names.push_back(t_cam_nam);
		}
		else
		{
			fprintf(stderr,
					"Note: found camera pipe callback already exists, likely a stereo camera setup\n");
		}
		pipe_client_flush(ch);

	}


    return 0;
}


