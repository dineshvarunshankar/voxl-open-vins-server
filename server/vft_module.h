#ifndef VFT_MODULE
#define VFT_MODULE

#include <modal_pipe.h>
#include <core/VioManager.h>
#include "config_file.h"


typedef struct vft_feature_set{
    uint8_t cam_id;
    float timestamp;
	std::vector<ov_core::ExtFeature> features;  // max support of 99 features
} __vft_feature_set;

extern std::deque<vft_feature_set> feature_queue;
extern std::mutex feature_queue_mtx;
extern vft_feature_set feat_set;
extern vft_feature_set feat_set_multi[4];
extern vft_feature_set feat_set_zero;

/**
 * @brief      callback when disconnected from feature tracking pipe 
 */
void _vft_disconnect_cb(__attribute__((unused)) int ch,
						__attribute__((unused)) void *context);


/**
 * @brief      callback when there is new data from feature tracking pipe 
 * 
 *             This validates the new vft data 
 */
void _vft_data_handler_cb(__attribute__((unused)) int ch, char* data, int bytes,
                          __attribute__((unused)) void* context);

/**
 * @brief      creates vft pipe client and associated callbacks
 *
 *             This sets the disconnect and simple helper callbacks, allocates
 *             memory for the vft, and opens the client pipe
 */
int connect_vft_service(void);



#endif // VFT_MODULE