/**
 * @file CameraBase.cpp
 * @brief Base camera implementation for VOXL OpenVINS
 * @author Zauberflote
 * @date 2025
 * @version 1.0
 *
 * This file implements the base camera functionality for the VOXL OpenVINS system.
 * It provides the foundation for camera handling, pipe communication, image
 * processing, and VIO integration.
 *
 * The implementation handles:
 * - Camera pipe connection and management
 * - Image data reception and processing
 * - OpenCL context setup for GPU acceleration
 * - Thread-safe operations with mutex protection
 * - ION buffer handling for efficient memory management
 * - Callback management for image data reception
 */

#include "CameraBase.h"

#include "VoxlVars.h"

#include <unistd.h>
#include <modal_flow/ocl/OclDevice.hpp>
#include <CL/cl_ext.h>
#include <sys/mman.h> // mmap

namespace voxl
{

    /**
     * @brief Constructor for CameraBase
     *
     * Initializes a camera instance with the provided configuration information.
     * Sets up the camera with default values and prepares it for connection.
     *
     * @param camera_info Camera configuration and calibration information
     */
    CameraBase::CameraBase(const cam_info &camera_info)
        : camera_info_(camera_info)
    {
    }

    /**
     * @brief Connect to the camera pipe service
     *
     * This method establishes the connection to the camera pipe service
     * and sets up the necessary callbacks for image data reception.
     *
     * The connection process includes:
     * - Obtaining an available pipe channel
     * - Configuring appropriate callbacks
     * - Opening the pipe connection with proper flags and buffer size
     * - Flushing the pipe to clear stale data
     *
     * @return true if connection was successful, false otherwise
     */
    bool CameraBase::connect()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (is_connected_)
        {
            std::cerr << "Camera " << camera_info_.name << " is already connected" << std::endl;
            return true;
        }

        // Get the next available channel
        channel_ = pipe_client_get_next_available_channel();
        if (channel_ < 0)
        {
            std::cerr << "Failed to get available channel for camera " << camera_info_.name << std::endl;
            vio_error_codes |= ERROR_CODE_CAM_MISSING;
            return false;
        }

        static const char suffix[] = "_ion";
        size_t len = strnlen(camera_info_.tracking_name, 128); // safe length
        size_t sfx = sizeof(suffix) - 1;                       // 4

        int flags, pipe_size;

        if (len >= sfx && memcmp(camera_info_.tracking_name + (len - sfx), suffix, sfx) == 0)
        {
            pipe_client_set_ion_buf_helper_cb(channel_, CameraBase::camera_device_buffer_callback, this);

            flags = CLIENT_FLAG_EN_ION_BUF_HELPER;
            pipe_size = 0;
        }
        else
        {
            // Set the camera callback
            pipe_client_set_camera_helper_cb(channel_, CameraBase::camera_callback, this);

            flags = CLIENT_FLAG_EN_CAMERA_HELPER;
            pipe_size = 1280 * 800 * 15;
        }

        int ret = pipe_client_open(channel_, camera_info_.tracking_name, PROCESS_NAME, flags, pipe_size);

        if (ret != 0)
        {
            std::cerr << "Failed to open camera pipe: " << camera_info_.tracking_name << std::endl;
            vio_error_codes |= ERROR_CODE_CAM_MISSING;
            return false;
        }

        if (en_debug)
        {
            std::cout << "Successfully connected to camera: " << camera_info_.name
                      << " (channel: " << channel_ << ")" << std::endl;
        }

        // Flush the pipe to clear any stale data
        pipe_client_flush(channel_);

        is_connected_ = true;
        return true;
    }

    /**
     * @brief Disconnect and clean up resources
     *
     * This method properly closes the pipe connection and cleans up
     * any allocated resources. It ensures a clean shutdown of the
     * camera connection.
     *
     * The disconnection process includes:
     * - Flushing the pipe to clear pending data
     * - Waiting for pending operations to complete
     * - Closing the pipe connection
     * - Updating connection state
     *
     * The method is thread-safe and handles cases where the camera
     * is already disconnected.
     */
    void CameraBase::disconnect()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!is_connected_)
        {
            return;
        }

        // Flush pipe to clear pending data
        if (en_debug)
        {
            printf("Flushing pipe for primary channel: %d\n", channel_);
        }
        pipe_client_flush(channel_);

        // Small delay to ensure pending operations complete
        usleep(50000);

        // Close the pipe
        pipe_client_close(channel_);
        is_connected_ = false;

        if (en_debug)
        {
            std::cout << "Disconnected camera: " << camera_info_.name << std::endl;
        }
    }

    /**
     * @brief Common callback function for pipe client
     *
     * This function receives raw image data from the pipe and dispatches
     * it to the appropriate processing method. It serves as the entry
     * point for all camera data processing.
     *
     * The callback performs the following operations:
     * - Enables thread cancellation for proper cleanup
     * - Validates the camera context pointer
     * - Checks for global shutdown conditions
     * - Dispatches to the derived class process_image method
     *
     * @param ch Channel number (unused)
     * @param meta Image metadata containing timestamp and format information
     * @param frame Pointer to image data buffer
     * @param context Context pointer (the CameraBase instance)
     */
    void CameraBase::camera_callback(int ch, camera_image_metadata_t meta, char *frame, void *context)
    {
        // Enable thread cancellation
        pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
        pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

        // Cast the context pointer back to a CameraBase pointer
        CameraBase *camera = static_cast<CameraBase *>(context);

        // Make sure we have a valid camera instance
        if (!camera)
        {
            std::cerr << "Invalid camera context in callback" << std::endl;
            vio_error_codes |= ERROR_CODE_CAM_MISSING;
            return;
        }

        // Early check for global shutdown flag
        if (!main_running)
        {
            return;
        }

        // Process the image in the derived class implementation
        camera->process_image(meta, voxl::ImageType::CV_MAT, frame);
    }

    void CameraBase::camera_device_buffer_callback(int ch, mpa_ion_buf_t *data, void *context)
    {
        // Enable thread cancellation
        pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
        pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

        // Cast the context pointer back to a CameraBase pointer
        CameraBase *camera = static_cast<CameraBase *>(context);

        // Make sure we have a valid camera instance
        if (!camera)
        {
            std::cerr << "Invalid camera context in callback" << std::endl;
            vio_error_codes |= ERROR_CODE_CAM_MISSING;
            return;
        }

        // Early check for global shutdown flag
        if (!main_running)
            return;

        auto meta = data->img_meta;

        cl_int err = CL_SUCCESS;
        if (!camera->ctx_)
            camera->ctx_ = modal_flow::ocl::OclDevice::Instance().context();
        if (!camera->q_)
            camera->q_ = clCreateCommandQueue(camera->ctx_, modal_flow::ocl::OclDevice::Instance().device(), 0, &err);
        if (err != CL_SUCCESS)
        {
            fprintf(stderr, "clCreateCommandQueue err=%d\n", err);
            return;
        }

        void *frame = mmap(NULL, data->size, PROT_READ | PROT_WRITE, MAP_SHARED, data->fd, 0);
        if (frame == MAP_FAILED)
        {
            perror("mmap");
            return;
        }
        cl_mem_ion_host_ptr cl_mem_ptr;
        memset(&cl_mem_ptr, 0, sizeof(cl_mem_ion_host_ptr));
        cl_mem_ptr.ext_host_ptr.allocation_type = CL_MEM_ION_HOST_PTR_QCOM;
        cl_mem_ptr.ext_host_ptr.host_cache_policy = CL_MEM_HOST_UNCACHED_QCOM; // CL_MEM_HOST_WRITEBACK_QCOM; //CL_MEM_HOST_UNCACHED_QCOM;
        cl_mem_ptr.ion_filedesc = data->fd;
        cl_mem_ptr.ion_hostptr = frame;

        cl_int err_code;
        cl_mem cl_mem_out = clCreateBuffer(camera->ctx_,
                                           CL_MEM_HOST_NO_ACCESS | CL_MEM_USE_HOST_PTR | CL_MEM_EXT_HOST_PTR_QCOM,
                                           data->size,
                                           &cl_mem_ptr,
                                           &err_code);

        if (err_code != CL_SUCCESS)
        {
            fprintf(stderr, "Error: clCreateBuffer failed with code %d\n", err_code);
        }

        cl_mem cl_mem_dest = clCreateBuffer(camera->ctx_,
                                            CL_MEM_HOST_NO_ACCESS | CL_MEM_READ_ONLY,
                                            data->size,
                                            NULL,
                                            &err_code);
        if (err_code != CL_SUCCESS)
        {
            fprintf(stderr, "Error: clCreateBuffer failed for destination buffer with code %d\n", err_code);
        }

        err_code = clEnqueueCopyBuffer(camera->q_,
                                       cl_mem_out,                                 // Source buffer (ION-backed memory)
                                       cl_mem_dest,                                // Destination buffer (GPU memory)
                                       0, 0,                                       // Source and destination offsets
                                       meta.height * meta.width * sizeof(uint8_t), // Size
                                       0, nullptr, nullptr);
        if (err_code != CL_SUCCESS)
        {
            fprintf(stderr, "Error: clEnqueueCopyBuffer failed with code %d\n", err_code);
        }
        clFinish(camera->q_);
        clReleaseMemObject(cl_mem_out);

        // Process the image in the derived class implementation
        camera->process_image(data->img_meta, voxl::ImageType::CL_MEM, cl_mem_dest);
    }

} // namespace voxl