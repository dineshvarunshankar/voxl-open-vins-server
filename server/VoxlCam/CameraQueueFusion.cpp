/**
 * @file CameraQueueFusion.cpp
 * @brief Camera queue fusion system implementation for VOXL OpenVINS
 * @author Zauberflote
 * @date 2025
 * @version 1.0
 *
 * This file implements the camera queue fusion system that synchronizes
 * and combines data from multiple cameras. It provides temporal alignment
 * and batch processing capabilities for multi-camera VIO systems.
 *
 * The implementation provides:
 * - Multi-camera synchronization using bit masks
 * - Temporal alignment of camera frames
 * - Thread-safe queue management
 * - Event-driven processing with condition variables
 * - Batch processing capabilities
 * - Background fusion thread management
 */

#include "CameraQueueFusion.h"
#include "CameraManager.h"

/**
 * @brief Get singleton instance
 *
 * Returns the single instance of the CameraQueueFusion, creating it
 * if it doesn't exist (lazy initialization). This ensures that only
 * one fusion system exists throughout the application lifecycle.
 *
 * @return Reference to the singleton CameraQueueFusion instance
 */
CameraQueueFusion &CameraQueueFusion::getInstance()
{
    static CameraQueueFusion instance;
    return instance;
}

/**
 * @brief Start the fusion system
 *
 * Initializes the fusion system with the specified number of cameras
 * and starts the background fusion thread.
 *
 * The initialization process includes:
 * - Validating the number of cameras (must be > 0 and <= MAX_CAMERA_COUNT)
 * - Setting up the expected mask for camera readiness tracking
 * - Starting the background fusion thread
 * - Setting the running flag
 *
 * The expected mask is calculated as (1 << num_cams) - 1, which creates
 * a bit mask with the lowest num_cams bits set to 1.
 *
 * @param num_cams Number of cameras to synchronize
 */
void CameraQueueFusion::start(size_t num_cams)
{
    if (running_.exchange(true))
        return;

    // Validate the number of cameras passed in
    if (num_cams == 0 || num_cams > MAX_CAMERA_COUNT)
    {
        std::cerr << "Invalid number of cameras: " << num_cams << " (max: " << MAX_CAMERA_COUNT << ")" << std::endl;
        running_.store(false, std::memory_order_release);
        return;
    }

    num_cams_ = num_cams;
    // Expect contiguous camera IDs 0..num_cams-1
    expected_mask_ = (1u << num_cams_) -1;

    fusion_thread_ = std::thread(&CameraQueueFusion::fusionLoop, this);
    fusion_thread_.detach();
}

/**
 * @brief Mark a camera as ready with new data
 *
 * This method is called when a camera has new data available.
 * It updates the camera ready mask and may trigger fusion processing.
 *
 * The method performs the following operations:
 * - Validates the camera ID to prevent buffer overflow
 * - Sets the corresponding bit in the camera ready mask
 * - Notifies the fusion thread that data is available
 *
 * The camera ready mask uses a bit field where each bit represents
 * whether a specific camera has new data available.
 *
 * @param cam_id Camera identifier (0-based)
 */
void CameraQueueFusion::markCameraReady(size_t cam_id)
{
    // Bounds check to avoid overflow in the ready mask
    if (cam_id >= MAX_CAMERA_COUNT)
    {
        std::cerr << "Camera ID out of range: " << cam_id
                  << " (max: " << MAX_CAMERA_COUNT - 1 << ")" << std::endl;
        return;
    }

    camera_ready_mask_.fetch_or(1u << cam_id);

    // NOW NOTIFYING THE FUSION THREAD
    cv_.notify_one();
}

/**
 * @brief Get sorted batch of camera data
 *
 * Retrieves a batch of synchronized camera data that is sorted
 * by timestamp and filtered by the specified cutoff time.
 *
 * The method performs the following operations:
 * - Locks the fusion mutex for thread-safe access
 * - Checks if fused frames are available
 * - Finds frames with timestamps greater than the cutoff
 * - Moves qualifying frames to the output vector
 * - Removes processed frames from the internal queue
 *
 * @param timestamp_cutoff Timestamp cutoff for data inclusion
 * @param out Output vector to store the sorted camera data
 * @return true if data was retrieved, false if no data available
 */
bool CameraQueueFusion::getSortedBatch(double timestamp_cutoff, std::vector<ov_core::CameraData> &out)
{
    std::lock_guard<std::mutex> lock(fusion_mutex_);

    if (fused_frames_.empty())
    {
        return false;
    }

    // Find the first frame that has a timestamp greater than the cutoff
    auto it = std::find_if(fused_frames_.begin(), fused_frames_.end(),
                           [timestamp_cutoff](const auto &frame)
                           {
                               return frame.timestamp > timestamp_cutoff;
                           });

    // If all frames are before the cutoff, return false
    if (it == fused_frames_.begin())
    {
        return false;
    }

    // Move all frames before the cutoff to the output vector
    out.insert(out.end(),
               std::make_move_iterator(fused_frames_.begin()),
               std::make_move_iterator(it));

    // Remove the moved frames from the queue
    fused_frames_.erase(fused_frames_.begin(), it);

    return !out.empty();
}

/**
 * @brief Main fusion loop
 *
 * Background thread function that continuously processes camera data
 * and performs temporal synchronization and fusion.
 *
 * The fusion loop performs the following operations:
 * - Waits for CameraManager initialization
 * - Waits for all cameras to have data available (with timeout)
 * - Collects data from all cameras
 * - Sorts data by timestamp
 * - Merges data with identical timestamps
 * - Stores fused data in the internal queue
 * - Resets the camera ready mask
 *
 * The loop runs at approximately 50Hz (20ms period) and uses
 * condition variables for efficient waiting and notification.
 */
void CameraQueueFusion::fusionLoop()
{
    using namespace std::chrono;
    const auto timeout = 1us;
    const auto init_check_interval = 50us;
    const auto kMinLoopPeriod = std::chrono::duration<double, std::milli>(fusion_rate_dt_ms); // try to run at 50Hz could be 34ms for 30Hz

    do
    {
        // First check if CameraManager is initialized
        if (!voxl::CameraManager::getInstance().isInitialized())
        {
            if (en_debug)
            {
                std::cerr << "Camera fusion waiting for CameraManager initialization..." << std::endl;
            }
            std::this_thread::sleep_for(init_check_interval);
            continue;
        }
        // auto start = steady_clock::now(); // Unused variable - commented out

        // while ((camera_ready_mask_.load() & expected_mask_) != expected_mask_) {
        //     if (steady_clock::now() - start >= timeout) break;
        //     std::this_thread::yield();
        // }

        // WAIT UNTIL ALL CAMERAS HAVE PRODUCED DATA OR WE HIT THE TIMEOUT RATE BASED ON THE LOOP PERIOD
        std::unique_lock<std::mutex> lk(cv_mtx_);
        bool all_cameras_ready = cv_.wait_for(lk, kMinLoopPeriod, [this]
                                              { return ((camera_ready_mask_.load() & expected_mask_) == expected_mask_) || !main_running; });
        lk.unlock();

        // If duration has elapsed, run the thread regardless of expected_mask
        // The wait_for returns false if timeout expired, true if predicate became true
        if (!all_cameras_ready && main_running)
        {
            // Timeout occurred - process whatever cameras are ready
            if (en_debug)
            {
                std::cerr << "Camera fusion timeout - processing available cameras (ready mask: 0x"
                          << std::hex << camera_ready_mask_.load()
                          << ", expected: 0x" << expected_mask_ << std::dec << ")" << std::endl;
            }
        }

        std::vector<ov_core::CameraData> batch;

        for (const auto &cam : voxl::CameraManager::getInstance().getAllCameras())
        {
            ov_core::CameraData msg;
            while (cam->popCameraData(msg))
            {
                batch.push_back(std::move(msg));
            }
        }

        if (!batch.empty())
        {
            // Sort by timestamp first  --> could use OPENVINS operator< instead
            std::sort(batch.begin(), batch.end(),
                      [](const auto &a, const auto &b)
                      {
                          return a.timestamp < b.timestamp;
                          //   return a < b; // uses CameraData::operator<
                      });

            // Merge entries that share the same timestamp
            std::vector<ov_core::CameraData> merged;
            merged.reserve(batch.size());

            for (auto &msg : batch)
            {
                if (!merged.empty() && msg.timestamp == merged.back().timestamp)
                {
                    // Append sensor ids, images, and masks
                    merged.back().sensor_ids.insert(merged.back().sensor_ids.end(),
                                                    msg.sensor_ids.begin(), msg.sensor_ids.end());
                    merged.back().images.insert(merged.back().images.end(),
                                                msg.images.begin(), msg.images.end());
                    merged.back().masks.insert(merged.back().masks.end(),
                                               msg.masks.begin(), msg.masks.end());
                    merged.back().img_frames.insert(merged.back().img_frames.end(),
                                                    msg.img_frames.begin(), msg.img_frames.end());
                }
                else
                {
                    merged.push_back(std::move(msg));
                }
            }

            std::lock_guard<std::mutex> lock(fusion_mutex_);
            fused_frames_.insert(fused_frames_.end(),
                                 std::make_move_iterator(merged.begin()),
                                 std::make_move_iterator(merged.end()));
        }

        camera_ready_mask_.store(0, std::memory_order_release);
    } while (main_running);
}