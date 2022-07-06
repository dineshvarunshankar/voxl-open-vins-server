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


#ifndef OPEN_VINS_SERVER_COMMON_H
#define OPEN_VINS_SERVER_COMMON_H



// This includes the primary VIO pipe interface. The remainder of this file is
// for things specific to this VIO server
#include <modal_pipe_interfaces.h>
#include <modal_pipe_common.h> // for MODAL_PIPE_DEFAULT_BASE_DIR


// command string sent to either VIO control pipe to blank out the camera
// image temporarily to test loss of features.
#define BLANK_VIO_CAM_COMMAND	"blank"

// command string sent to either VIO control pipe to fade the camera
// image gradually out to black
#define FADE_VIO_CAM_COMMAND	"fade"

#define OV_VIO_CONTROL_COMMANDS (RESET_VIO_SOFT "," RESET_VIO_HARD "," BLANK_VIO_CAM_COMMAND "," FADE_VIO_CAM_COMMAND)

// These are the paths of the named pipe interfaces
// MODAL_PIPE_DEFAULT_BASE_DIR is defined in modal_pipe_common.h
// the 'complete' interface sends data as a qvio_data_t struct
// the 'simple' interface uses the simple vio_data_t struct from modal_vio_server_interface.h

#define OV_VIO_EXTENDED_NAME		"ov_extended"
#define OV_VIO_EXTENDED_LOCATION	MODAL_PIPE_DEFAULT_BASE_DIR OV_VIO_EXTENDED_NAME "/"
#define OV_VIO_SIMPLE_NAME		"ov"
#define OV_VIO_SIMPLE_LOCATION	MODAL_PIPE_DEFAULT_BASE_DIR OV_VIO_SIMPLE_NAME "/"


#define OV_VIO_OVERLAY_NAME		"ov_overlay"
#define OV_VIO_OVERLAY_LOCATION	MODAL_PIPE_DEFAULT_BASE_DIR OV_VIO_OVERLAY_NAME "/"


#endif // OPEN_VINS_SERVER_COMMON_H
