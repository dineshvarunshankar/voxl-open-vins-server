#include "shared_vars.h"
#include "config_file.h"
#include "quality.h"
#include "common.h"
#include "cam_config_file.h"


int grid_spacing_x;
int grid_spacing_y;


static double map_double(double x, double in_min, double in_max, double out_min,
		double out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// Looks at quality over time
static bool stable_quality(int cur_qual, bool bypass=false)
{

	static double last_good_qual_ts = _apps_time_monotonic_ns();
	bool is_bad = false;
	double ts_threshold = auto_reset_max_v_cov_timeout_s;

	if (bypass)
		ts_threshold = auto_reset_max_v_cov_timeout_s - 1;

	if (cur_qual >= 1)
	{
		last_good_qual_ts = _apps_time_monotonic_ns();
	}

	// if quality is less than acceptable for more than 2 sec
	double ts = (_apps_time_monotonic_ns() - last_good_qual_ts) * 1e-9;

	if (ts > ts_threshold)
	{
		printf("[ERROR] quality was bad for a long time!\n");
		is_bad = true;
	}

	return is_bad;
}


// looks at state during initialization to avoid false positive restarts
static bool stable_state(int the_state)
{
	static double last_good_state_ts = _apps_time_monotonic_ns();
	static bool is_ready = false;

	if (is_ready)
		return is_ready;

	double ts = (_apps_time_monotonic_ns() - last_good_state_ts) * 1e-9;

	// if quality is less than acceptable for more than 2 sec
	if (the_state != VIO_STATE_OK)
	{
		last_good_state_ts = _apps_time_monotonic_ns();
	}
	else if (ts > 3)
	{
		is_ready = true;
	}

	return is_ready;
}


// Looks at quality over time
static bool stable_features(int cur_feats, bool bypass = false)
{
	static bool wait_for_features = true;
	double ts_threshold = auto_reset_min_feature_timeout_s;

	if (wait_for_features)
	{
		if (cur_feats > 3)
		{
			last_good_feat_ts = _apps_time_monotonic_ns();
			wait_for_features = false;
		}
		return false;
	}

	bool is_bad = false;
	// if quality is less than acceptable for more than 2 sec

	int min_good_feat_thresh = auto_reset_min_features;
	
	if (!offline && ext_blind_take_off_force && !is_armed)
	{
		min_good_feat_thresh = max_slam_in_update;
		cur_feats = min_good_feat_thresh - 1;
	}	
	
	if (bypass)
		ts_threshold = auto_reset_min_feature_timeout_s -1;

	if (cur_feats > min_good_feat_thresh)
	{
		last_good_feat_ts = _apps_time_monotonic_ns();
	}

	double ts = (_apps_time_monotonic_ns() - last_good_feat_ts) * 1e-9;

	if (ts > ts_threshold)
	{
		printf("ERROR: features were 0 for a long time! cur: %d, min_req: %d (are you bench testing?) \n", cur_feats, min_good_feat_thresh);
		is_bad = true;
		wait_for_features = true;
		ext_blind_take_off_force = false;
	}

	return is_bad;
}

static Eigen::Matrix<double, 4, 1> euler_to_quaternion(double roll, double pitch, double yaw) {
    double cy = cos(yaw / 2);
    double sy = sin(yaw / 2);
    double cp = cos(pitch / 2);
    double sp = sin(pitch / 2);
    double cr = cos(roll / 2);
    double sr = sin(roll  / 2);

    Eigen::Matrix<double, 4, 1> q;
    q(3) = cy * cp * cr + sy * sp * sr;
    q(0) = cy * cp * sr - sy * sp * cr;
    q(1) = cy * sp * cr + sy * cp * sr;
    q(2) = sy * cp * cr - cy * sp * sr;

    return q;
}



void _publish_default(double pose_timestamp)
{
    static Eigen::Matrix3d rot_global_zero_horizon = Eigen::Matrix3d::Identity();
	static double nullpoint = 1;
	static double flipframe = 1;

	int nPoints;
	int n_good_points = 0;
	int n_oos_points = 0;
	int i, j;

	// make sure we start with clean data structs and apply any global error codes
	// full extended vio packet
//	memset(&d, 0, sizeof(d));
	d.v.magic_number = VIO_MAGIC_NUMBER;
	d.v.error_code = global_error_codes;

	std::shared_ptr < ov_msckf::State > current_state =
			vio_manager->get_state(); // contains a few extra pieces we need

	// simple lib modal pipe standard vio packet
//	memset(&s, 0, sizeof(s));
	s.magic_number = VIO_MAGIC_NUMBER;
	s.error_code = global_error_codes;
	s.timestamp_ns = static_cast<int64_t>(current_state->_timestamp * 1e9);

	// simple lib modal pipe standard vio packet
//	memset(&ov_status, 0, sizeof(ov_status));
	ov_status.magic_number = VIO_MAGIC_NUMBER;

	// record that we just got a successful pose and point cloud
	last_real_pose_timestamp_ns = static_cast<int64_t>(pose_timestamp * 1e9);

	if (!vio_manager->initialized())
	{
		s.timestamp_ns = last_real_pose_timestamp_ns;
		s.quality  = -1;
		s.state = VIO_STATE_INITIALIZING;
		s.error_code |= ERROR_CODE_STALLED;
		memcpy(&d.v, &s, sizeof(vio_data_t));
		is_initialized = false;
		// send to both pipes

		if (pipe_server_get_num_clients(EXTENDED_CH) > 0) // publish
			pipe_server_write(EXTENDED_CH, (char*) &d, sizeof(ext_vio_data_t));
		if (pipe_server_get_num_clients(SIMPLE_CH) > 0) // publish
			pipe_server_write(SIMPLE_CH, (char*) &s, sizeof(vio_data_t));

		return;
	}
	else
	{
		s.state = VIO_STATE_OK;
		is_initialized = true;
	}
	

#ifdef EXPERIMENTAL // replaces above
	// check if its initialized or not
	if (!en_force_init)
	{
		if (!vio_manager->initialized())
		{
			s.state = VIO_STATE_INITIALIZING;
			memcpy(&d.v, &s, sizeof(vio_data_t));
			is_initialized = false;
			// send to both pipes
			pipe_server_write(EXTENDED_CH, (char*) &d, sizeof(ext_vio_data_t));
			pipe_server_write(SIMPLE_CH, (char*) &s, sizeof(vio_data_t));
			
			return;
		}
		else
		{
			vio_manager_options.init_options.init_dyn_use = true;
			s.state = VIO_STATE_OK;
			is_initialized = true;
		}
	}
	else  // FORCE INITIALIZATION -- only for non-mission, non-autonomy use, for pilot use
	{
		 if (imu_moved)
		 {
			s.state = VIO_STATE_OK;
			is_initialized = true;
		 }
		 else
		 {
				s.state = VIO_STATE_OK;
				memcpy(&d.v, &s, sizeof(vio_data_t));
				is_initialized = false;
				// send to both pipes
				pipe_server_write(EXTENDED_CH, (char*) &d, sizeof(ext_vio_data_t));
				pipe_server_write(SIMPLE_CH, (char*) &s, sizeof(vio_data_t));
				return;
		 } 
	}
#endif
	
	std::vector < std::shared_ptr < ov_type::Type >> statevars;
	statevars.push_back(current_state->_imu->p());
	statevars.push_back(current_state->_imu->q());
	statevars.push_back(current_state->_imu->v());

	Eigen::Matrix<double, 9, 9> covariance_posori =
			ov_msckf::StateHelper::get_marginal_covariance(current_state,
					statevars);
	// Row-major representation of the 6x6 covariance matrix
	// The orientation parameters use a fixed-axis representation.
	// In order, the parameters are:
	// (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
	// float64[36] covariance

	Eigen::VectorXd cov_varis = covariance_posori.diagonal();

	double T_uncertainty = 0.0;
	T_uncertainty += cov_varis(0, 0) * cov_varis(0, 0);
	T_uncertainty += cov_varis(1, 1) * cov_varis(1, 1);
	T_uncertainty += cov_varis(2, 2) * cov_varis(2, 2);
	T_uncertainty = sqrt(T_uncertainty);

	double R_uncertainty = 0.0;
	R_uncertainty += cov_varis(3, 3) * cov_varis(3, 3);
	R_uncertainty += cov_varis(4, 4) * cov_varis(4, 4);
	R_uncertainty += cov_varis(5, 5) * cov_varis(5, 5);
	R_uncertainty = sqrt(R_uncertainty);

	double V_uncertainty = 0.0;
	V_uncertainty += cov_varis(6, 6) * cov_varis(6, 6);
	V_uncertainty += cov_varis(7, 7) * cov_varis(7, 7);
	V_uncertainty += cov_varis(8, 8) * cov_varis(8, 8);
	V_uncertainty = sqrt(V_uncertainty);

//	if (en_debug)
//		printf("Uncertainty in the robot's pose: xyz: %f R:%f V: %f\n", T_uncertainty, R_uncertainty, V_uncertainty);

#ifdef MONTE
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covariance_posori);
	Eigen::MatrixXd transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
	static std::mt19937 gen{ std::random_device{}() };
	static std::normal_distribution<> dist;
	Eigen::VectorXd uncertain = transform * Eigen::VectorXd{ 6 }.unaryExpr([&](auto x) { return dist(gen); });
	Eigen::Vector3d  T_sigma = uncertain.segment(0,2);
	Eigen::Vector3d  R_sigma = uncertain.segment(3,5);
	T_uncertainty = (T_uncertainty * 0.6) + (0.4 * (sqrt((T_sigma.array() - T_sigma.mean()).square().sum() / (T_sigma.size() - 1))));
	R_uncertainty = (R_uncertainty * 0.6) + (0.4* (sqrt((R_sigma.array() - R_sigma.mean()).square().sum() / (R_sigma.size() - 1))));

	if (std::isnan(T_uncertainty))
	{
		T_uncertainty = 0;
	}

	if (std::isnan(R_uncertainty))
	{
		R_uncertainty = 0;
	}

	printf("Uncertainty in the robot's pose: xyz: %f R:%f\n", T_uncertainty, R_uncertainty);
#endif

//	printf("Get the uncertainty in the robot's pose: (%dx%d) %f\n", uncertain.rows(), uncertain.cols(), std_dev);
//	printf("(%d): ", (int)cov_varis.size());
//	for (int i=0; i<cov_varis.size(); i++)
//	{
//		printf("%f ", cov_varis(i));
//	}
//	printf("\n");

	// get features
	// this function will give us back as much info as available for features in various stages of the overall state
	std::vector < output_feature > curr_pixel_locs;
	int sz = vio_manager->get_pixel_loc_features(curr_pixel_locs);
	
	for (size_t d = 0; d < curr_pixel_locs.size(); d++)
	{
		if (curr_pixel_locs[d].pix_loc[0] > 0.0f
				&& curr_pixel_locs[d].pix_loc[1] > 0.0f)
		{
			if (curr_pixel_locs[d].point_quality == OV_HIGH)
			{
				n_good_points++;
			}
			else if (curr_pixel_locs[d].point_quality == OV_MEDIUM)
			{
				// UNUSED points
				n_oos_points++;
			}
		}
	}

	// sometimes qvio will report covariance as invalid but state is still OKAY
	// this is NOT alright, in this case manually set the state to failed.
	//
	// Rotation
	if (cov_varis(3, 3) < 0.0f || cov_varis(4, 4) < 0.0f
			|| cov_varis(5, 5) < 0.0f)
	{
		fprintf(stderr, "ERROR: diagonal went negative\n");
		s.state = VIO_STATE_FAILED;
		s.error_code |= ERROR_CODE_COVARIANCE;
	}

	// don't send packets from the past, this can happen when qvio stalls
	// during a reset
	if (s.timestamp_ns < last_sent_timestamp_ns)
	{
		fprintf(stderr, "WARNING: skipping pose data from the past %f %ld\n",
				current_state->_timestamp * 1e9, last_sent_timestamp_ns);
		s.error_code |= ERROR_CODE_BAD_TIMESTAMP;

		return;
	}

	double run_rate_s = (s.timestamp_ns - last_sent_timestamp_ns)*1e-9;

	// All checks passed, after this point this function should not return
	// until the end
	last_sent_timestamp_ns = static_cast<int64_t>(current_state->_timestamp
			* 1e9);

	// populate some other data
	d.imu_cam_time_shift_s = current_state->_calib_dt_CAMtoIMU->value()(0);
	last_time_alignment_ns = current_state->_calib_dt_CAMtoIMU->value()(0)
			* 1e9;
	s.n_feature_points = n_good_points;
	d.last_cam_frame_id = last_frame_frame_id;
	d.last_cam_timestamp_ns = last_frame_timestamp_ns;

	// gather all the VINS output
	Eigen::Matrix<double, 3, 1> imu_wrt_wio_holder = current_state->_imu->pos();
	Eigen::Matrix<double, 3, 1> vel_imu_wrt_vio_holder = current_state->_imu->vel();
	// Not used anymore!
	//	Eigen::Matrix3d rot_fej = current_state->_imu->Rot_fej();
	Eigen::Matrix<double, 4, 1> ned_q = current_state->_imu->quat_fej();

	// Rotations

	// TODO make more efficient
	// TODO maybe use body extrinsics to set a flag if VOXL2 is mounted irregularly
	// TODO which is what this is for
	static Eigen::Matrix<double, 4, 1> rot_corrected(default_level_horizon);

#ifdef EXPERIMENTAL
	if (!imu_moved && !is_armed)
	{
		   double angle_q = std::acos(default_level_horizon.dot(ned_q));
		   if (std::abs(angle_q * 180.0 / M_PI) > 30.0)
		   {
			   printf("[INFO] dynamic horizon alignment\n");
			   rot_corrected = ov_core::Inv(ned_q);
		   }
		   else
		   {
			   printf("[INFO] static IMU horizon alignment\n");
			   rot_corrected = default_level_horizon;
		   }
    }
#endif

	Eigen::Matrix<double, 4, 1> new_rot = ov_core::quat_multiply(ned_q, rot_corrected);

   // Convert to NED of quat
    new_rot.y() *= -1;
    new_rot.z() *= -1;

	static double yaw_offset = 0;

    if (gravity_axis == 0 && gravity_vector_direction < 0) // TODO maynot need gravity_vector_direction eval
	{
    	new_rot = ov_core::quat_multiply(world_correction_q, new_rot);

    	Eigen::Matrix<double, 3, 3> rot_new_rot = ov_core::quat_2_Rot(new_rot);
		Eigen::Matrix<double, 3, 1> rpy  =ov_core:: rot2rpy(rot_new_rot);

    	if (zero_horizon)
    	{
 		   double angle_q = std::acos(default_level_horizon.dot(ned_q));
		   if (std::abs(angle_q * 180.0 / M_PI) > 30.0)
		   {
			   yaw_offset = rpy(2);
		   }

	    	// one shot
			zero_horizon = false;
	    	printf("NULL FACTOR from yaw->roll: %f\n", yaw_offset*180.0 / M_PI);
    	}

    	Eigen::Matrix<double, 3, 3> yaw_offset_rot = ov_core::rot_z(-yaw_offset);
    	new_rot = ov_core::quat_multiply(new_rot, ov_core::rot_2_quat(yaw_offset_rot));
		rpy  =ov_core:: rot2rpy(ov_core::quat_2_Rot(new_rot));

		// WARNING!!!
    	// TODO FIX THIS AS A ROTATION AND NOT SWAPPING
    	// THIS NEEDS TO SUPPORT  INVERSE OF WORLD COORINATES
    	// ONLY A HACK FOR VERTICALLY MOUNTED VOXLs
		new_rot = euler_to_quaternion(gravity_vector_direction*rpy(2), rpy(0), rpy(1));

	}
    else  if (gravity_axis == 2 && gravity_vector_direction  < 0)
	{
    	Eigen::Matrix<double, 3, 3> roll_offset_rot = ov_core::rot_x(M_PI);
    	new_rot = ov_core::quat_multiply(new_rot, ov_core::rot_2_quat(roll_offset_rot));
	}

    // support other rotation here!

	Eigen::Matrix3d final_out_ned = ov_core::quat_2_Rot(new_rot);
	Eigen::MatrixXf::Map(reinterpret_cast<float*>(s.R_imu_to_vio), 3, 3) =
			final_out_ned.cast<float>();

    if (en_debug)
    {
        Eigen::Matrix<double, 3, 1>  rpy =  ov_core::rot2rpy(final_out_ned);
    	printf("[INFO]  %f, %f, %f --- %f, %f, %f\n",
                    rpy(0)/M_PI*180,
                    rpy(1)/M_PI*180,
                    rpy(2)/M_PI*180,
                    imu_wrt_wio_holder(0),
                    imu_wrt_wio_holder(1),
                    imu_wrt_wio_holder(2));
    }

	// camera position here is a bit funky, since open vins outputs imu to cam and we want cam to imu
	Eigen::Matrix3d cam_out = ov_core::quat_2_Rot(
			current_state->_calib_IMUtoCAM[0]->quat()).transpose();

	Eigen::MatrixXf::Map(reinterpret_cast<float*>(s.R_cam_to_imu), 3, 3) =
			cam_out.cast<float>();


    if (gravity_axis == 0 && gravity_vector_direction < 0)
    {
		rot_global_zero_horizon = Eigen::AngleAxisd(yaw_offset, Eigen::Vector3d::UnitZ());
    }

    //
	// Translation x y z
	//
	imu_wrt_wio_holder = flu_ned_correction_mat * imu_wrt_wio_holder;
	imu_wrt_wio_holder = rot_global_zero_horizon * imu_wrt_wio_holder;
	//imu_wrt_wio_holder = world_correction_eigen.inverse() * imu_wrt_wio_holder;
	Eigen::MatrixXf::Map(s.T_imu_wrt_vio, 3, 1) = imu_wrt_wio_holder.cast<float>();

	//
	// Translation velocity vx vy vz
	//
	vel_imu_wrt_vio_holder = flu_ned_correction_mat * vel_imu_wrt_vio_holder;
	vel_imu_wrt_vio_holder =rot_global_zero_horizon * vel_imu_wrt_vio_holder;
	//vel_imu_wrt_vio_holder = world_correction_eigen.inverse()  * vel_imu_wrt_vio_holder;
	Eigen::MatrixXf::Map(s.vel_imu_wrt_vio, 3, 1) = vel_imu_wrt_vio_holder.cast<float>();


	Eigen::MatrixXf::Map(s.T_cam_wrt_imu, 3, 1) = ((ov_core::quat_2_Rot(
			current_state->_calib_IMUtoCAM[0]->quat().transpose())
			* current_state->_calib_IMUtoCAM[0]->pos()) * -1).cast<float>();
	Eigen::MatrixXf::Map(reinterpret_cast<float*>(d.gyro_bias), 3, 1) =
			current_state->_imu->bias_g_fej().cast<float>();
	Eigen::MatrixXf::Map(reinterpret_cast<float*>(d.accl_bias), 3, 1) =
			current_state->_imu->bias_a_fej().cast<float>();

	// pose covariance diagonals, 6 entries
	s.pose_covariance[0] = (float) cov_varis(0, 0);
	s.pose_covariance[6] = (float) cov_varis(1, 1);
	s.pose_covariance[11] = (float) cov_varis(2, 2);
	s.pose_covariance[15] = (float) cov_varis(3, 3);
	s.pose_covariance[18] = (float) cov_varis(4, 4);
	s.pose_covariance[20] = (float) cov_varis(5, 5);

	// velocity covariance diagonals, 3 entries
	s.velocity_covariance[0] = (float) cov_varis(6, 6);
	s.velocity_covariance[6] = (float) cov_varis(7, 7);
	s.velocity_covariance[11] = (float) cov_varis(8, 8);

	// open vins does not estimate this, but reports it
//	Eigen::Matrix<double, 3, 1> imu_vel = current_state->_imu->vel();
//	double imu_angular_vel[3];
//	imu_angular_vel[0] = imu_vel(0);
//	imu_angular_vel[1] = imu_vel(1);
//	imu_angular_vel[2] = imu_vel(2);
//
//	Eigen::Matrix<double, 3, 1> imu_angular_vel_holder(imu_angular_vel);
//	imu_angular_vel_holder = flu_ned_correction_mat * imu_angular_vel_holder;

// NO 	//imu_angular_vel_holder = world_correction_mat * imu_angular_vel_holder;
//	s.imu_angular_vel[0] = imu_angular_vel_holder(0);
//	s.imu_angular_vel[1] = imu_angular_vel_holder(1);
//	s.imu_angular_vel[2] = imu_angular_vel_holder(2);

	static Eigen::Matrix<double, 3, 1> last_rpy;

    Eigen::Matrix<double, 3, 1>  rpy =  ov_core::rot2rpy(final_out_ned);
    double delta_yaw =  last_rpy(2) - rpy(2);
    double yawrate =  delta_yaw / run_rate_s;
    double delta_roll =  last_rpy(0) - rpy(0);
    double rollrate =  delta_roll / run_rate_s;
    double delta_pitch =  last_rpy(1) - rpy(1);
    double pitchrate =  delta_pitch / run_rate_s;
    last_rpy = rpy;

	s.imu_angular_vel[0] = rollrate;
	s.imu_angular_vel[1] = pitchrate;
	s.imu_angular_vel[2] = yawrate;

	// gravtiy vector direction should be negative if the VOXL is upside down.
	// TODO figure this out by rotating the gravtiiy vector properly with the
	// world_frame matrix
	alt_z = gravity_vector_direction * imu_wrt_wio_holder(gravity_axis);

	// since open vins does the gravity alignment internally, gravity vec is always 0,0,1 and cov is 0'd out BUT
	// voxl flips it to actual
	if (gravity_axis == 0)
	{
		// NOTE -1 to flip gravity from body coordindates!
		float grav_vec[3] =
			{ (float) -1 * gravity_vector_direction, 0, 0 };
		memcpy(s.gravity_vector, grav_vec, sizeof(float) * 3);
	}
	else if (gravity_axis == 1)
	{
		// NOTE -1 to flip gravity from body coordindates!
		float grav_vec[3] =
			{ 0, (float) -1 * gravity_vector_direction,  0 };
		memcpy(s.gravity_vector, grav_vec, sizeof(float) * 3);
	}
	else if (gravity_axis == 2)
	{
		float grav_vec[3] =
			{ 0, 0, (float) gravity_vector_direction };
		memcpy(s.gravity_vector, grav_vec, sizeof(float) * 3);
	}

	// limit the number of features to what fits in our pipe packet
	d.n_total_features = (int) curr_pixel_locs.size();
	if (d.n_total_features > VIO_MAX_REPORTED_FEATURES)
	{
		d.n_total_features = VIO_MAX_REPORTED_FEATURES;
	}

	memcpy(d.features, curr_pixel_locs.data(),
			d.n_total_features * sizeof(vio_feature_t));


	if (!is_armed || !is_initialized)
	{
		s.quality = map_double(T_uncertainty, 0.004375, max_allowable_cep, 100, 0);
	}
	else
	{		// calc quality separately for each camera
		int qualities[cameras_used];

		for (int z=0; z<cameras_used;z++)
		{
			int t_cam_feats = 0;
			vio_feature_t cam_feats[VIO_MAX_REPORTED_FEATURES];

			for (size_t i = 0; i < display_snapshot.d.n_total_features; i++)
			{
				if (display_snapshot.d.features[i].cam_id == z)
				{
					cam_feats[t_cam_feats] = display_snapshot.d.features[i];
					t_cam_feats++;
				}
			}
			qualities[z] = calc_quality(s.state, s.velocity_covariance, current_width,	current_height, t_cam_feats, cam_feats);
		}

		// pick quality from best camera. If one camera is obscured that's fine
		// TODO something fancier
		int max_q = 0;
		for (int z=0; z<cameras_used;z++)
		{
			if (qualities[z] > max_q)
				max_q = qualities[z];
		}
		s.quality = max_q;

	}

	if (s.quality > 100)
		s.quality = 100;

	if (pause_qmin)
		s.quality = 0;

	if (check_for_stable_vins)
	{
		s.quality = -1;
	}


	if (!check_for_stable_vins && en_auto_reset && !init_failure_detector_reset_flag
			&& stable_state(s.state))
	{
	
		if (fabs(baro_alt) < 1.0 && fabs(d_baro) < 1.0 && fabs(s.T_imu_wrt_vio[2]) > 5.0)
			printf("INFO: MAYBE ON GROUND, POSITION LOCK? -- INVOKE ESTOP! (This is just a warning) %f %f %f\n",
					baro_alt, 	d_baro, s.T_imu_wrt_vio[2]);
		

		// check for agressive yaw
	    bool  spinning_in_place = false;
	    bool too_much_spinning = false;
	    if ( !en_cont_yaw_checks)
	    {
	    	static double start_spin_time = s.timestamp_ns;

	    	spinning_in_place = (fabs(yawrate) > fast_yaw_thresh && fabs(vel_imu_wrt_vio_holder(0)) <= 1.0 && fabs(vel_imu_wrt_vio_holder(1))<= 1.0);
	    	if (!spinning_in_place)
	    		start_spin_time = s.timestamp_ns;

	    	too_much_spinning = (s.timestamp_ns - start_spin_time) * 1e-9 > fast_yaw_timeout_s;
	    }


		bool vio_manager_bad = current_state->error_flag == VIO_STATE_FAILED;
		bool quality_bad = imu_moved && s.quality < 1;
		bool stable_quality_bad = imu_moved && stable_quality(s.quality);
		bool stable_features_bad = imu_moved && stable_features(n_good_points);
		bool too_fast = current_state->_imu->vel().norm() > current_reset_max_velocity;
		bool too_uncertain = is_armed && V_uncertainty > auto_reset_max_v_cov_instant;
		
		if (vio_manager_bad
				|| quality_bad
				|| stable_quality_bad 
				|| stable_features_bad
				|| too_fast
				|| too_uncertain
				|| too_much_spinning)
		{
			if (vio_manager_bad)
			{
				fprintf(stderr,"[VIO_BAD_STATE] State flag was set to FAIL.\n");
				s.error_code |= ERROR_CODE_STALLED;
			}
			else if (quality_bad)
			{
				fprintf(stderr,"[VIO_BAD_STATE] Quality less than 1.\n");
				s.error_code |= ERROR_CODE_CONSTRAINT;
			}
			else if (stable_quality_bad)
			{
				fprintf(stderr,"[VIO_BAD_STATE] Quality was not STABLE.\n");
				s.error_code |= ERROR_CODE_NOT_STATIONARY;
			}
			else if (stable_features_bad)
			{
				fprintf(stderr,"[VIO_BAD_STATE] No Stable feature.\n");
				s.error_code |= ERROR_CODE_NO_FEATURES;
			}
			else if (too_fast)
			{
				fprintf(stderr,"[VIO_BAD_STATE] Exceeded MAX IMU Velocity %f vs %f\n", current_state->_imu->vel().norm(),
						current_reset_max_velocity);
				s.error_code |= ERROR_CODE_VEL_WINDOW_CERT;

			}
			else if (too_much_spinning)
			{
				fprintf(stderr,"[VIO_BAD_STATE] Exceeded spin rate over time threshold %f!\n", fast_yaw_timeout_s);
				s.error_code |= ERROR_CODE_IMU_OOB;
			}
			else if (too_uncertain)
			{
				fprintf(stderr,"[VIO_BAD_STATE] Exceeded V_uncertainty. %f vs %f\n", V_uncertainty,
						auto_reset_max_v_cov_instant);
				s.error_code |= ERROR_CODE_VEL_INST_CERT;
			}
			else
			{
				fprintf(stderr,"[CRITICAL] UNKNOWN RESET TRIGGERED\n");
				s.error_code |= ERROR_CODE_UNKNOWN;
			}

			s.quality = -1;
			s.state = VIO_STATE_FAILED;

			if (is_armed || offline)
				init_failure_detector_reset_flag = 1;
			else
				init_failure_detector_reset_flag = 4;

			fprintf(stderr,
					"WARNING auto-resetting, Bad VIO, State: %s   Q: %d   Vel: %f Uncert: %f State: %d(%d)\n",
							(current_state->error_flag == VIO_STATE_FAILED) ? "false" : " true", 
							s.quality, 
							current_state->_imu->vel().norm(),
							V_uncertainty,
							s.state,
							stable_state(s.state));
			
		}
	}
	
	// fill in simplified struct inside the extended packet
	memcpy(&d.v, &s, sizeof(vio_data_t));
	
	if ((pipe_server_get_num_clients(OVSTATUS_CH) > 0) && en_ov_stats)
	{
		ov_status.timestamp_ns = s.timestamp_ns;
		ov_status.quality = s.quality;
		ov_status.p_dop = T_uncertainty;
		ov_status.r_dop = R_uncertainty;
		ov_status.num_features = curr_pixel_locs.size();

		unsigned int max_sz = curr_pixel_locs.size();
		if (max_sz >= VIO_MAX_REPORTED_FEATURES)
			max_sz = VIO_MAX_REPORTED_FEATURES - 1;

		for (size_t b = 0; b < max_sz; b++)
		{
			ov_status.features[b].id = curr_pixel_locs[b].id;
			ov_status.features[b].cam_id = curr_pixel_locs[b].cam_id;
			ov_status.features[b].pix_loc[0] = curr_pixel_locs[b].pix_loc[0];
			ov_status.features[b].pix_loc[1] = curr_pixel_locs[b].pix_loc[1];
		}

		if (pipe_server_get_num_clients(OVSTATUS_CH) > 0)
			pipe_server_write(OVSTATUS_CH, (char*) &ov_status, sizeof(ov_status_t));
	}

	if (en_debug_pos)
	{
		printf("%6.3f %6.3f %6.3f ", (double) s.T_imu_wrt_vio[0],
				(double) s.T_imu_wrt_vio[1], (double) s.T_imu_wrt_vio[2]);
		printf("\n");
	}

	// turn off dropped frame code now we have informed everyone.
	global_error_codes &= ~ERROR_CODE_DROPPED_CAM;

	display_snapshot.d = d;
	display_snapshot.timestamp = last_real_pose_timestamp_ns;
	display_snapshot.used_pts = n_good_points;
	display_snapshot.not_used_pts = n_oos_points;
	display_snapshot.q = s.quality;
	display_snapshot.cep = T_uncertainty;
	display_snapshot.rerr = R_uncertainty;

	//publish
	if (pipe_server_get_num_clients(EXTENDED_CH) > 0)
		pipe_server_write(EXTENDED_CH, (char*) &d, sizeof(ext_vio_data_t));
	if (pipe_server_get_num_clients(SIMPLE_CH) > 0) // publish
		pipe_server_write(SIMPLE_CH, (char*) &s, sizeof(vio_data_t));

	return;
}
