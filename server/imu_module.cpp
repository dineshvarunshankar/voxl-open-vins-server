#include "shared_vars.h"
#include "imu_module.h"
#include "config_file.h"
#include "cam_config_file.h"
#include "common.h"
#include <state/StateHelper.h>

#include <stdio.h>


std::mutex imu_lock_mutex;

void _imu_disconnect_cb(__attribute__((unused)) int ch,
		__attribute__((unused)) void *context)
{
	std::lock_guard < std::mutex > lg(imu_lock_mutex);
	return;
}

// imu callback registered to the imu server
void _imu_data_handler_cb(__attribute__((unused)) int ch,
		char *data, int bytes, __attribute__((unused)) void *context)
{	

	double cam_imu_time_delta = (double) (_apps_time_monotonic_ns()
			- last_cam_time) * 1e-9;

	if (is_initialized && idler_limit <= 0)
	{
		if (!bypass_reset_checks && cam_imu_time_delta > 0.3)
		{
			printf("-----------> UNSTABLE [THERMAL?] %1.2f <----------------\n", cam_imu_time_delta);
			bypass_reset_checks = true;
		}
		else if (bypass_reset_checks && cam_imu_time_delta < 0.25)
		{
			printf("-----------> STABLE [THERMAL?] %1.2f  <----------------\n", cam_imu_time_delta);
			bypass_reset_checks = false;
			resume_processing = 1;
		}
	}

	std::lock_guard < std::mutex > imu_lg(imu_lock_mutex);

//	if (en_debug)
//			printf("_new_imu_data_default_handler entrance: %f\n",  _apps_time_monotonic_ns() * 1e-9);

	int n_packets;
	imu_data_t *data_array = pipe_validate_imu_data_t(data, bytes, &n_packets);
	if (data_array == NULL)
	{
		printf("ERROR: no IMU data\n");
		return;
	}

	if (n_packets <= 0)
	{
		printf("ERROR: no IMU n_packets extracted from imu data\n");
		return;
	}

	// flag that imu data is active, remove the error, skip data if camera is disconnected
	is_imu_connected = 1;
	global_error_codes &= ~ERROR_CODE_IMU_MISSING;

	if (!is_cam_connected || !main_running)
	{
		s.magic_number = VIO_MAGIC_NUMBER;
		d.v.magic_number = VIO_MAGIC_NUMBER;

		std::shared_ptr < ov_msckf::State > current_state =
				vio_manager->get_state(); // contains a few extra pieces we need
		
		s.timestamp_ns = static_cast<int64_t>(current_state->_timestamp * 1e9);
		s.error_code |= ERROR_CODE_CAM_MISSING;
		d.v.error_code |= ERROR_CODE_CAM_MISSING;

		memcpy(&d.v, &s, sizeof(vio_data_t));

		is_initialized = false;
		// send to both pipes
 		if (pipe_server_get_num_clients(EXTENDED_CH) > 0) // publish
 			pipe_server_write(EXTENDED_CH, (char*) &d, sizeof(ext_vio_data_t));

 		if (pipe_server_get_num_clients(SIMPLE_CH) > 0) // publish
 			pipe_server_write(SIMPLE_CH, (char*) &s, sizeof(vio_data_t));

		return;
	}

		is_init = vio_manager->initialized();

		// time this in debug mode
		int64_t process_time;

		static double last_accel_mag = 0, last_gyro_mag = 0;
		static int last_perf_limit = 0;

		// TODO current IMU is setup to batch send imu values. This has a conflict with OV's internal processing system
		// by pausing caluclation while the publishing loop runs with new timestamps.
		// So on average use every other packet.
		if (!vio_manager->is_moving())
		{
		perf_limit = 5; //6
		}
		else
		{

			if (n_packets > 100)
			perf_limit = 10;  //3
			else if (n_packets > 15)
			perf_limit = 3;
			else
				perf_limit = 1;

			if (!changed_motion_state)
			{
				// ARM CHECK?
				fprintf(stderr,
						"[WARN] Motion detected, going to full processing\n");
				changed_motion_state = true;
			}
		}

		if (en_debug && perf_limit != last_perf_limit)
		{
			printf("WARNING: IMU processing rate changed: %d\n", perf_limit);
		}

		last_perf_limit = perf_limit;

	for (int i = 0; i < n_packets; i += perf_limit) {

			int64_t  dt_long =  (data_array[i].timestamp_ns-last_imu_timestamp_ns);
			double dt = (double) dt_long * 1e-09;

			// check if we somehow got an out-of-order imu sample and reject it
		if ((int64_t) data_array[i].timestamp_ns <= last_imu_timestamp_ns) {
			fprintf(stderr, "WARNING out-of-order imu %fms before previous\n", dt);
				continue;
			}
		else {

				// Create the data struct that we will use for ingesting data into the vio manager
				ov_core::ImuData vio_manager_data;
				vio_manager_data.timestamp = data_array[i].timestamp_ns * 1e-09; // (seconds)

				Eigen::Matrix<double, 3, 1> t_wm;
				Eigen::Matrix<double, 3, 1> t_am;

				t_wm(2, 0) = data_array[i].gyro_rad[2];
				t_wm(1, 0) = data_array[i].gyro_rad[1];
				t_wm(0, 0) = data_array[i].gyro_rad[0];
				t_am(2, 0) = data_array[i].accl_ms2[2];
				t_am(1, 0) = data_array[i].accl_ms2[1];
				t_am(0, 0) = data_array[i].accl_ms2[0];

				if (!imu_moved && dt <0.01)
				{
					static int ramp = 0;
					static double prev_accel = 0;
					double accel_mag = t_am.norm();
					static double base_accel = accel_mag;

					if (ramp++ < 256)
					{
						base_accel = accel_mag;
					}
					else
					{
						double d_accel = accel_mag-base_accel;
						d_accel  = (0.8 * d_accel) - (0.2 * prev_accel);
						prev_accel = d_accel;
						if (d_accel > init_imu_thresh_accel && (throttle_state > 1 || en_vio_always_on))
						{

							if (ramp < 258)
								printf("[INFO] IMU has moved, JERK detected\n");


							imu_moved = true;
							vio_manager->force_moving();
						}
					}

//////////////////////////////////////////////////////////////////
// TODO -- why was this added
//					if (is_initialized)
//					{
//////////////////////////////////////////////////////////////////

					for (int k = 0; k<3; ++k)
					{
						if (fabs(data_array[i].accl_ms2[k]) > fabs(data_array[i].accl_ms2[gravity_axis]))
						{
							gravity_axis = k;
						}
					}

//////////////////////////////////////////////////////////////////
// TODO -- why was this added
//					}
//					else
//					{
//						gravity_axis = 2;
//					}
//////////////////////////////////////////////////////////////////

			}

								
				// NED to FLU systems as per VINS.
				t_wm = flu_ned_correction_mat * t_wm;
				t_am = flu_ned_correction_mat * t_am;

				// FLU set data for VINS
				vio_manager_data.wm(0, 0) = t_wm(0, 0); // roll
				vio_manager_data.wm(1, 0) = t_wm(1, 0);  // pitch
				vio_manager_data.wm(2, 0) = t_wm(2, 0);  //yaw
				vio_manager_data.am(0, 0) = t_am(0, 0);  // X axis
				vio_manager_data.am(1, 0) = t_am(1, 0);  // Y axis
				vio_manager_data.am(2, 0) = t_am(2, 0);  // Z axis

				vio_manager->feed_measurement_imu(vio_manager_data);

				last_imu_timestamp_ns = data_array[i].timestamp_ns;
				last_imu_time = vio_manager_data.timestamp;

				if (thread_update_running)
					return;

				thread_update_running = true;

				double timestamp_imu_inC = vio_manager_data.timestamp
						- vio_manager->get_state()->_calib_dt_CAMtoIMU->value()(
								0);

				if (feature_queue.size() > 10)
				{
					printf("[WARN]: VFT data overflow (>1 sec) \n");
				}

				std::thread thread ([&]
				{
					std::lock_guard < std::mutex > feat_lck(feature_queue_mtx);
					std::lock_guard < std::mutex > pub_lg(publish_lock_mutex);
					std::lock_guard < std::mutex > lck(camera_queue_mtx);

					try
					{
						if (en_ext_feature_tracker)
						{
							camera_queue.clear();
							while (!feature_queue.empty() && feature_queue.at(0).timestamp< timestamp_imu_inC)
							{
								vft_feature_set fst = feature_queue.at(0);
								vio_manager->feed_measurement_feature(fst.timestamp,fst.features);
								_publish_default(last_imu_time);
								feature_queue.pop_front();

							Eigen::MatrixXd covariance_mat = ov_msckf::StateHelper::get_full_covariance(vio_manager->get_state());
							// std::cout << "Matrix rows: " << covariance_mat.rows() << ", cols: " << covariance_mat.cols() << "\n";

							}
						}
						else
						{
							while (!camera_queue.empty() && camera_queue.at(0).timestamp< timestamp_imu_inC)
							{
								vio_manager->feed_measurement_camera(camera_queue.at(0));
								_publish_default(last_imu_time);
								camera_queue.pop_front();
							}
						}
					}
					catch (...)
					{
						fprintf(stderr, "IMU Process error!\n");
					}

					thread_update_running = false;
			});
				thread.join();

			}
		}
}



int connect_imu_service(void) {

	pipe_client_set_disconnect_cb(IMU_CH, _imu_disconnect_cb, NULL);
	pipe_client_set_simple_helper_cb(IMU_CH, _imu_data_handler_cb, NULL);
    
	int flags = CLIENT_FLAG_EN_SIMPLE_HELPER;

	if (pipe_client_open(IMU_CH, imu_name, PROCESS_NAME, flags,
			IMU_RECOMMENDED_READ_BUF_SIZE) != 0)
	{
		fprintf(stderr, "failed to open imu client pipe\n");
		return -1;
	}

    return 0;
}
