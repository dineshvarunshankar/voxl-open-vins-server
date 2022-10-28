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
#ifndef VOXL_IMG_RINGBUF_H
#define VOXL_IMG_RINGBUF_H

#include <stdint.h>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string.h>

#include <modal_pipe.h>


#ifndef unlikely
#define unlikely(x)	__builtin_expect (!!(x), 0)
#endif

#ifndef likely
#define likely(x)	__builtin_expect (!!(x), 1)
#endif

#define MAX_IMAGE_SIZE  4147200

////////////////////////////////////////////////////////////////////////////////
// image ringbuffer
////////////////////////////////////////////////////////////////////////////////

typedef struct img_ringbuf_packet{
    camera_image_metadata_t metadata;       // image metadata information
    uint8_t image_pixels[MAX_IMAGE_SIZE];   // image pixels
} __attribute__((packed)) img_ringbuf_packet;

class RingBuffer {

    public:

        /**
         * @brief RingBuffer constructor
         * 
         * @param size  number of BufUnits to allocate space for
         */
        RingBuffer(int size){
            // make sure it's zero'd out
            RingBuffer::size = 0;
            index = 0;
            items_in_buf=0;
            initialized = 0;

            // allocate mem for array
            data = (img_ringbuf_packet*)calloc(size,sizeof(img_ringbuf_packet));
            if(data==NULL){
                fprintf(stderr,"ERROR in %s, failed to allocate memory\n", __FUNCTION__);
            }
            else {
                // write out other details
                RingBuffer::size = size;
                initialized = 1;
            }
        }


        /**
         * @brief generic insertertion function into the buf
         * 
         * @param new_packet  pointer to BufUnit we want inserted
         * 
         * @return < 0 on error, 0 on success
         */
        int insert_data(img_ringbuf_packet* new_packet);


        /**
         * @brief retrieve data given position in the buf
         * 
         * @param position  buf index we would like to retrieve
         * @param result    pointer to BufUnit we will fill
         * 
         * @return < 0 on error, 0 on success
         */
        int get_data_at_position(int position, img_ringbuf_packet* result);


        /**
         * @brief retrieve data given timestamp in the buf
         * 
         * @param timestamp_ns  timestamp would we like to retrieve
         * @param result        pointer to BufUnit we will fill
         * 
         * @return < 0 on error, 0 on success
         */
        int get_data_at_time(int64_t timestamp_ns, img_ringbuf_packet* result);


        /// destructor, frees the calloc'ed memory if allocated
        ~RingBuffer() {
            if(unlikely(data==NULL)){
                fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
            }
            else if(initialized){
                free(data);
            }
        }

    protected:

        /**
         * @brief retrieve ptr to data in the buf at a given position
         * UNSAFE (no mutex lock)
         * @param position  buf index we would like to retrieve
         * 
         * @return BufUnit ptr at position
         */
        img_ringbuf_packet* get_data_ptr_at_position(int position);


        /**
         * @brief retrieve timestamp of data in the buf at a given position
         * UNSAFE (no mutex lock)
         * @param position  buf index we would like to retrieve
         * 
         * @return timestamp of data at position
         */
        int64_t get_timestamp_at_position(int position);


        /// pointer to internal data, templated so BufUnits must have a static size
        img_ringbuf_packet* data;

        /// how many units we have space for in the buf
        int size;

        /// current index of the ring
        int index;

        /// running count of number of items present in our buffer
        int items_in_buf;

        /// flag to show whether or not the buf was properly initialized
        bool initialized;

        /// last succesfully inserted timestamp of data in the buf
        int64_t latest_timestamp_ns;

        /// mutex to protect our buffer
        std::mutex buf_mutex;

};

#endif //VOXL_GEN_RINGBUF_H