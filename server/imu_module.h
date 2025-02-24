#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <modal_pipe.h>
#include "config_file.h"
#include "vft_module.h"


extern std::mutex imu_lock_mutex;

void _imu_disconnect_cb(__attribute__((unused)) int ch,
		                __attribute__((unused)) void *context);


void _imu_data_handler_cb(__attribute__((unused)) int ch, char *data, int bytes, 
                          __attribute__((unused)) void *context);

void set_frd_to_imu(imu_data_t *data_array, int i);

/**
 * @brief      creates imu pipe client and associated callbacks
 *
 *             This sets the disconnect and simple helper callbacks, and opens 
 *             the client pipe
 */
int connect_imu_service(void);

#endif // IMU_MODULE_H
