/**
 * @file ImuMinimal.cpp
 * @brief IMU interface and data handling implementation for VOXL OpenVINS
 * @author Zauberflote
 * @date 2025
 * @version 1.0
 *
 * This file implements the IMU interface and callback functions for handling
 * inertial measurement unit data in the VOXL OpenVINS system. It provides
 * the connection to the IMU service and data processing capabilities.
 *
 * The implementation provides:
 * - IMU data reception and validation
 * - Frame transformation for different IMU orientations
 * - Batch processing for efficient VIO integration
 * - Camera-IMU synchronization
 * - VIO state management and publishing
 * - Thread-safe operations with mutex protection
 */

#include "ImuMinimal.h"
#include <state/State.h>

namespace
{
    // THIS IS NOT CURRENTLY USED -- BUT IT IS HERE FOR FUTURE USE WITH MULTI-IMU SUPPORT
    ////////////////////////////////////////////////////////////////////////////////////
    constexpr int kIMUQueueSize = 512;
    boost::lockfree::spsc_queue<imu_data_t, boost::lockfree::capacity<kIMUQueueSize>> imu_queue;
    std::atomic<bool> imu_thread_running{false};
    std::thread imu_thread;
    ////////////////////////////////////////////////////////////////////////////////////

    // CACHE SHARED PTR TO AVOID REPEATED ALLOCATION AT HIGH FREQUENCY
    std::shared_ptr<ov_msckf::State> cached_state;
    std::map<double, std::vector<std::shared_ptr<ov_core::Feature>>> cached_features_map;

    // THIS IS HERE TO ACCOMODATE THE FACT THAT THE IMU IS NOT ALWAYS MOUNTED IN THE SAME ORIENTATION
    // VISION HUB ALSO EXPECTS DATA TO BE IN FRD AND IMU FRAME-ORIENTATION
    //  -- FOR SPECIAL SETUPS THIS IS YOUR BREAD AND BUTTER, BUT MAKE SURE YOUR EXTRINSICS ARE CORRECT FOR YOUR SETUP
    struct FrameTransform
    {
        // WE NEED TO FIND THE AXIS WHERE GRAVITY IS MOST PREDOMINANT AND ITS DIRECTION
        // THEN WE CAN COMPUTE THE CORRECTION MATRIX
        enum class Axis
        {
            X,
            Y,
            Z
        };
        enum class Direction
        {
            POSITIVE,
            NEGATIVE
        };

        // ASSUME IMU IS MOUNTED ALIGNED WITH THE BODY FRAME
        Axis gravity_axis{Axis::Z};
        Direction gravity_direction{Direction::NEGATIVE};
        bool is_initialized{false};
        Eigen::Matrix3d correction_matrix{Eigen::Matrix3d::Identity()};

        // PROVE THE ASSUMPTION
        void update(const imu_data_t &data)
        {
            if (is_initialized)
                return; // CHECK IF WE HAVE ALREADY DONE THIS

            // FOR NOW WE ARE CHECKING WITH ONE SAMPLE -- PARTIALLY ASSUMING STATIC INITIALIZATION
            // MAX ELEMENT NORM IS THE AXIS WHERE GRAVITY IS MOST PREDOMINANT
            std::array<double, 3> accel{data.accl_ms2[0], data.accl_ms2[1], data.accl_ms2[2]};
            auto max_it = std::max_element(accel.begin(), accel.end(),
                                           [](double a, double b)
                                           { return std::abs(a) < std::abs(b); });

            gravity_axis = static_cast<Axis>(std::distance(accel.begin(), max_it));
            gravity_direction = *max_it > 0 ? Direction::POSITIVE : Direction::NEGATIVE;

            // NOW COMPUTE CORRECTION MATRIX BASED ON GRAVITY AXIS AND DIRECTION
            // CASES ARE HARD-CODED BUT YOU CAN EDIT AND ADD MORE CASES AS NEEDED
            switch (gravity_axis)
            {
            case Axis::X:
                if (data.accl_ms2[2] < 0)
                {
                    correction_matrix = Eigen::AngleAxisd(
                                            (gravity_direction == Direction::POSITIVE ? 1 : -1) * M_PI / 2,
                                            Eigen::Vector3d::UnitY())
                                            .toRotationMatrix();
                }
                else
                {
                    correction_matrix = Eigen::AngleAxisd(
                                            -(gravity_direction == Direction::POSITIVE ? 1 : -1) * M_PI / 2,
                                            Eigen::Vector3d::UnitY())
                                            .toRotationMatrix() *
                                        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
                }
                break;
            case Axis::Y:
                correction_matrix = Eigen::AngleAxisd(
                                        (gravity_direction == Direction::POSITIVE ? 1 : -1) * M_PI / 2,
                                        Eigen::Vector3d::UnitX())
                                        .toRotationMatrix();
                break;
            case Axis::Z:
                if (gravity_direction == Direction::POSITIVE)
                {
                    correction_matrix = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
                }
                break;
            }

            is_initialized = true;
            printf("[INFO] Frame transform initialized - Gravity axis: %d, Direction: %d\n",
                   static_cast<int>(gravity_axis),
                   static_cast<int>(gravity_direction));
        }

        Eigen::Vector3d transform(const Eigen::Vector3d &v) const
        {
            return correction_matrix * v;
        }
    };

    FrameTransform frame_transform;
}

/** @brief Mutex for IMU data access synchronization */
std::mutex imu_lock_mutex;

/**
 * @brief Handler for incoming IMU data
 *
 * This callback processes incoming IMU data, updates the system state,
 * and triggers appropriate processing based on motion state. It serves
 * as the primary entry point for all IMU data processing in the system.
 *
 * The function performs the following operations:
 * - Validates and parses IMU data packets
 * - Updates frame transformation based on gravity direction
 * - Converts IMU data to OpenVINS format
 * - Performs batch processing to reduce mutex operations
 * - Feeds IMU data to the VIO manager
 * - Synchronizes camera data with IMU timestamps
 * - Triggers VIO processing and data publishing
 *
 * The function uses batch processing to minimize mutex contention
 * at high IMU frequencies and ensures proper temporal synchronization
 * between IMU and camera data.
 *
 * @param ch Channel number (unused)
 * @param data Pointer to IMU data buffer
 * @param bytes Size of data buffer in bytes
 * @param context Context pointer (unused)
 */
void _imu_data_handler_cb(int ch, char *data, int bytes, void *context)
{
    // MPA CALLBACK FOR IMU
    // if (is_resetting.load()) return; // if we are resetting, just return

    // MODALAI IMU IS READ BASED ON A FIFO QUEUE TRIGGERED BY THE CAMERAS
    int n_packets = 0;
    imu_data_t *arr = pipe_validate_imu_data_t(data, bytes, &n_packets);

    
    // indicate that we are processing IMU data
    active_callbacks.fetch_add(1, std::memory_order_acquire);
    if (is_resetting.load(std::memory_order_relaxed))
    {
        active_callbacks.fetch_sub(1, std::memory_order_release);
        return;
    }
    if (!arr || n_packets <= 0)
    {
        vio_error_codes |= ERROR_CODE_IMU_MISSING;
        // Decrement callback counter before returning to keep active_callbacks balanced
        active_callbacks.fetch_sub(1, std::memory_order_release);
        return;
    }

    // CUSTOM IMU BATCH PROCESSING FUNCTION TO HALT THE MANY MUTEX LOCK OPS AT HIGH FREQUENCY INSIDE OVINS
    // THIS PROCESSES THE IMU DATA BATCH WITH A SINGLE MUTEX LOCK
    std::vector<ov_core::ImuData> imu_batch;
    // PRE-ALLOCATE
    imu_batch.reserve(n_packets);

    // STILL SCAN THROUGH THE FIFO QUEUE -- TO CONVERT THEM TO THE OVINS FORMAT
    // CAN CHANGE THIS INSIDE THE IMU-SERVER LATER ON IF NEEDED
    for (int i = 0; i < n_packets; ++i)
    {
        frame_transform.update(arr[i]);

        ov_core::ImuData sample;
        sample.timestamp = arr[i].timestamp_ns * 1e-9;
        sample.wm = Eigen::Vector3d(arr[i].gyro_rad[0], arr[i].gyro_rad[1], arr[i].gyro_rad[2]);
        sample.am = Eigen::Vector3d(arr[i].accl_ms2[0], arr[i].accl_ms2[1], arr[i].accl_ms2[2]);
        // NO NEED TO ROTATE THE IMU DATA -- WE GOT IT COVERED IN THE CALIBRATION
        if (last_imu_timestamp_ns == 0)
        {
        }
        else if (sample.timestamp * 1e9 <= last_imu_timestamp_ns)
        {
            // Only flag error if timestamp is significantly in the past (more than 1ms)
            int64_t time_diff = last_imu_timestamp_ns - (sample.timestamp * 1e9);
            if (time_diff > 1000000)
            { // 1ms in nanoseconds
                printf("[DEBUG] Setting ERROR_CODE_BAD_TIMESTAMP in IMU\n");
                vio_error_codes |= ERROR_CODE_BAD_TIMESTAMP;
            }
        }
        imu_batch.push_back(std::move(sample)); // MOVE THE SAMPLE TO THE BATCH
    }
    if (imu_batch.empty())
    {
        vio_error_codes |= ERROR_CODE_DROPPED_IMU;
        // Decrement callback counter before returning to keep active_callbacks balanced
        if (active_callbacks.fetch_sub(1, std::memory_order_release) == 1)
        {
            std::lock_guard<std::mutex> lk(reset_mtx);
            reset_cv.notify_one();
        }
        return; // CHECK IF THE BATCH IS EMPTY
    }
    // FEED BATCH TO OVINS
    vio_manager->feed_measurement_batch_imu(imu_batch, 333); // SAMPLE FREQS: 200, 250, 333, 500, 1000

    // SYNC CAMERA USING TIMESTAMP OF LAST IMU SAMPLE
    last_imu_timestamp_ns = imu_batch.back().timestamp * 1e9;
    // TODO: CLEAR IMPORTANT CONSIDERARTION: OVINS DOES NOT SUPPORT MULTI-CAM TIME OFFSET CALIBRATION --> That's mainly because of the central image queue design
    // Independently calibrating camera offset can be supported by either modfiying the feed function in OVINS + state vars
    // or manually keeping tabs on relative camera offsets and correcting at the CameraQueueFusion level, either solution is kosher but for now we will just use the average offset
    double ts_cutoff = (last_imu_timestamp_ns * 1e-9) -
                       vio_manager->get_state()->_calib_dt_CAMtoIMU->value()(0);

    // GET THE CAMERA BATCH FROM OUR CENTRAL QUEUE
    std::vector<ov_core::CameraData> batch;
    if (CameraQueueFusion::getInstance().getSortedBatch(ts_cutoff, batch))
    {
        for (const auto &frame : batch)
        {
            vio_manager->feed_measurement_camera(frame);
            // Only update cached state if needed (could add additional logic here to determine when to update)
            // if (!cached_state || vio_manager->initialized()) {
            cached_state = vio_manager->get_state();
            // }

            cached_features_map = vio_manager->get_used_features_map();

            voxl::Publisher::getInstance().publish(cached_state, frame_transform.correction_matrix, cached_features_map);
        }
    }

    // check if in flight processing count reaches zero
    if (active_callbacks.fetch_sub(1, std::memory_order_release) == 1)
    {
        // If we are resetting, notify the reset condition variable
        // This will wake up the reset thread to continue processing
        std::lock_guard<std::mutex> lk(reset_mtx);
        reset_cv.notify_one();
    }
}

/**
 * @brief Callback for IMU disconnect events
 *
 * This function is called when the IMU service disconnects. It handles
 * the cleanup and state management required when IMU data becomes unavailable.
 *
 * The function uses thread-safe mutex locking to ensure proper
 * state management during disconnection events.
 *
 * @param ch Channel number (unused)
 * @param context Context pointer (unused)
 */
void _imu_disconnect_cb(__attribute__((unused)) int ch,
                        __attribute__((unused)) void *context)
// THREAD SAFE DISCONNECT CALLBACK
{
    std::lock_guard<std::mutex> lg(imu_lock_mutex);
    return;
}

/**
 * @brief Creates IMU pipe client and associated callbacks
 *
 * This function sets up the disconnect and data handler callbacks,
 * and opens the client pipe connection to the IMU service. It initializes
 * the complete IMU data pipeline for the VIO system.
 *
 * The function performs the following operations:
 * - Sets up disconnect callback for graceful handling of service disconnection
 * - Sets up data handler callback for processing incoming IMU measurements
 * - Configures thread priority for high-frequency IMU processing
 * - Opens the client pipe connection to the IMU service
 * - Configures the pipe for optimal data flow
 * - Sets the IMU connection status flag
 *
 * @return 0 on success, -1 on failure
 */
int connect_imu_service(void)
{
    // MPA REGULAR CLIENT SETUP

    pipe_client_set_disconnect_cb(IMU_CH, _imu_disconnect_cb, NULL);
    pipe_client_set_simple_helper_cb(IMU_CH, _imu_data_handler_cb, NULL);
    pipe_client_set_helper_thread_priority(IMU_CH, THREAD_PRIORITY_RT_HIGH);

    int flags = CLIENT_FLAG_EN_SIMPLE_HELPER;

    if (pipe_client_open(IMU_CH, imu_name, PROCESS_NAME, flags,
                         IMU_RECOMMENDED_READ_BUF_SIZE) != 0)
    {
        fprintf(stderr, "failed to open imu client pipe\n");
        vio_error_codes |= ERROR_CODE_IMU_MISSING;
        return -1;
    }
    is_imu_connected = true;

    return 0;
}
