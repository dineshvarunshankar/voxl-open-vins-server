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

    // FrameTransform is now defined in VoxlVars.h and instantiated globally
}

/** @brief Mutex for IMU data access synchronization */
std::mutex imu_lock_mutex;


#define TS_PRINT(msg) \
    do { \
        int64_t __ts = _apps_time_monotonic_ns(); \
        printf("[TS %ld ns] %s\n", __ts, msg); \
    } while(0)

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
    int64_t t_start = _apps_time_monotonic_ns();
    int64_t t_prev  = t_start;

    // printf("[TS %ld ns] IMU callback start, bytes=%d\n", t_start, bytes);

    // ---- validate data ----
    int n_packets = 0;
    imu_data_t *arr = pipe_validate_imu_data_t(data, bytes, &n_packets);
    int64_t t_validate = _apps_time_monotonic_ns();
    // printf("[DT %8.3f ms] validate done, n_packets=%d\n",
    //        (t_validate - t_prev)/1e6, n_packets);
    // t_prev = t_validate;

    active_callbacks.fetch_add(1, std::memory_order_acquire);
    if (is_resetting.load(std::memory_order_relaxed))
    {
        active_callbacks.fetch_sub(1, std::memory_order_release);
        return;
    }
    if (!arr || n_packets <= 0)
    {
        vio_error_codes |= ERROR_CODE_IMU_MISSING;
        active_callbacks.fetch_sub(1, std::memory_order_release);
        return;
    }

    // ---- build batch ----
    std::vector<ov_core::ImuData> imu_batch;
    imu_batch.reserve(n_packets);

    for (int i = 0; i < n_packets; ++i)
    {
        frame_transform.update(arr[i]);

        ov_core::ImuData sample;
        sample.timestamp = arr[i].timestamp_ns * 1e-9;
        sample.wm = Eigen::Vector3d(arr[i].gyro_rad[0], arr[i].gyro_rad[1], arr[i].gyro_rad[2]);
        sample.am = Eigen::Vector3d(arr[i].accl_ms2[0], arr[i].accl_ms2[1], arr[i].accl_ms2[2]);

        if (last_imu_timestamp_ns != 0 &&
            (sample.timestamp * 1e9 <= last_imu_timestamp_ns))
        {
            int64_t time_diff = last_imu_timestamp_ns - (sample.timestamp * 1e9);
            if (time_diff > 1000000) // 1 ms
            {
                printf("[DEBUG] IMU timestamp regression %ld ns\n", time_diff);
                vio_error_codes |= ERROR_CODE_BAD_TIMESTAMP;
            }
        }
        imu_batch.push_back(std::move(sample));
    }
    int64_t t_batch = _apps_time_monotonic_ns();
    // printf("[DT %8.3f ms] batch conversion done, count=%zu\n",
    //        (t_batch - t_prev)/1e6, imu_batch.size());
    // t_prev = t_batch;

    if (imu_batch.empty())
    {
        vio_error_codes |= ERROR_CODE_DROPPED_IMU;
        if (active_callbacks.fetch_sub(1, std::memory_order_release) == 1 &&
            reset_requested.load(std::memory_order_relaxed))
        {
            std::lock_guard<std::mutex> lk(reset_mtx);
            reset_cv.notify_one();
        }
        return;
    }

    // ---- feed IMU ----
    vio_manager->feed_measurement_batch_imu(imu_batch, 333);
    // int64_t t_feed_imu = _apps_time_monotonic_ns();
    // printf("[DT %8.3f ms] fed IMU batch into VIO manager\n",
    //        (t_feed_imu - t_prev)/1e6);
    // t_prev = t_feed_imu;

    // ---- sync camera ----
    last_imu_timestamp_ns = imu_batch.back().timestamp * 1e9;
    double ts_cutoff = (last_imu_timestamp_ns * 1e-9) -
                       vio_manager->get_state()->_calib_dt_CAMtoIMU->value()(0);

    std::vector<ov_core::CameraData> batch;
    if (CameraQueueFusion::getInstance().getSortedBatch(ts_cutoff, batch))
    {
        for (const auto &msg : batch)
        {
            vio_manager->feed_measurement_camera(msg);
            cached_state = vio_manager->get_state();

            // release cl_mem objects
            for (auto &frame : msg.img_frames) {
                if (frame.img.handle_type == modal_flow::ExternalType::ClMem &&
                    frame.img.external_handle != 0) {
                    cl_mem handle = reinterpret_cast<cl_mem>(
                        static_cast<uintptr_t>(frame.img.external_handle));

                    cl_int err = clReleaseMemObject(handle);
                    if (err != CL_SUCCESS) {
                        fprintf(stderr, "Failed to release Frame cl_mem, err=%d\n", err);
                    }
                    // reset so no double free
                    const_cast<modal_flow::ImageView&>(frame.img).external_handle = 0;
                }
            }

            cached_features_map = vio_manager->get_used_features_map();
            voxl::Publisher::getInstance().publish(cached_state, cached_features_map);
        }
    }
    int64_t t_cam = _apps_time_monotonic_ns();
    // printf("[DT %8.3f ms] processed %zu camera frames\n",
    //        (t_cam - t_prev)/1e6, batch.size());
    // t_prev = t_cam;

    // ---- finish ----
    int64_t t_end = _apps_time_monotonic_ns();
    printf("[DT %8.3f ms] callback finished for %d msgs, total=%8.3f ms\n",
           (t_end - t_prev)/1e6, batch.size(), (t_end - t_start)/1e6);

    if (active_callbacks.fetch_sub(1, std::memory_order_release) == 1 &&
        reset_requested.load(std::memory_order_relaxed))
    {
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
