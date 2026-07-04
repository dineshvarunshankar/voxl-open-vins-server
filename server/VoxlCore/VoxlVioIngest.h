#pragma once
/**
 * VoxlVioIngest.h -- wires the server's per-frame post-processing into VioManager's async ingest.
 *
 * Since the lock-free camera ingest lives inside ov_msckf (AsyncCameraBuffer), the server no
 * longer loops over camera batches: cameras push frames straight into the VioManager from their
 * pipe threads, and the IMU feed releases them in global timestamp order on the VIO thread.
 * This hook runs after each released frame (processed=true) or for every frame the ingest drops
 * (processed=false): it releases external cl_mem image handles exactly once, and publishes the
 * updated state. Returning false pauses draining while a reset is pending.
 *
 * Register on EVERY VioManager instance right after construction (boot + hard-reset swap).
 *
 * Author: Joao Leonardo Silva Cotta (@zauberflote1)
 */

#include <atomic>

#include "VoxlHK.h"   // voxl::Publisher
#include "VoxlVars.h" // vio_manager, is_resetting, vio_error_codes (+ vio pipe error codes)

#if HAVE_OPENCL
#include <CL/cl.h>
#endif

namespace voxl {

/// Release any external (cl_mem) image handles carried by this frame (idempotent per frame:
/// handles are zeroed after release so a double call cannot double-free)
inline void release_frame_handles(const ov_core::CameraData &msg) {
#if HAVE_OPENCL
    for (auto &frame : msg.img_frames) {
        if (frame.img.handle_type == modal_flow::ExternalType::ClMem && frame.img.external_handle != 0) {
            cl_mem handle = reinterpret_cast<cl_mem>(static_cast<uintptr_t>(frame.img.external_handle));
            cl_int err = clReleaseMemObject(handle);
            if (err != CL_SUCCESS) {
                fprintf(stderr, "Failed to release Frame cl_mem, err=%d\n", err);
            }
            const_cast<modal_flow::ImageView &>(frame.img).external_handle = 0;
        }
    }
#else
    (void)msg;
#endif
}

/// Install the per-frame hook on a VioManager (call once per instance, right after construction)
inline void register_vio_camera_callback(ov_msckf::VioManager &vm) {
    vm.set_camera_processed_callback([](const ov_core::CameraData &msg, bool processed) -> bool {
        // Handles are released for EVERY frame that entered the ingest, dropped or processed
        release_frame_handles(msg);
        if (!processed) {
            vio_error_codes |= ERROR_CODE_DROPPED_CAM;
            return true;
        }
        if (is_resetting.load(std::memory_order_acquire)) {
            return false; // pause draining; the reset path owns the estimator now
        }
        // Publish the updated state (same VIO thread that ran the update)
        auto st = vio_manager->get_state();
        auto feats = vio_manager->get_used_features_map();
        if (is_resetting.load(std::memory_order_relaxed)) {
            return false;
        }
        voxl::Publisher::getInstance().publish(st, feats);
        return true;
    });
}

} // namespace voxl
