#ifndef CAM_MODULE_H
#define CAM_MODULE_H

#include <modal_pipe.h>
#include "common.h"

/**
 * @brief      creates camera pipe clients and associated callbacks
 *
 *             This sets the camera helper callbacks and opens the client pipes
 */
int connect_cam_service(std::vector<cam_info> &cam_info_vec);



#endif // CAM_MODULE_H