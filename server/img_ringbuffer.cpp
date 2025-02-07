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

#include "img_ringbuffer.h"

int RingBuffer::insert_data(img_ringbuf_packet* new_packet){
    int new_index;

    // sanity checks
    if(unlikely(data==NULL)){
        fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
        return -1;
    }

    // we are about to interact with the ringbuf, lock the mutex
    buf_mutex.lock();

    // more sanity checks
    if(unlikely(!initialized)){
        buf_mutex.unlock();
        fprintf(stderr,"ERROR in %s, ringbuf uninitialized\n", __FUNCTION__);
        return -1;
    }
    if(unlikely(new_packet->metadata.timestamp_ns <= latest_timestamp_ns)){
        buf_mutex.unlock();
        fprintf(stderr,"WARNING inserting image into overlay ringbuffer, detected timestamp out of order ov, %ld vs %ld\n", __FUNCTION__, new_packet->metadata.timestamp_ns,latest_timestamp_ns);
        return -1;
    }

    // if this is the first thing to be entered make sure to start at zero
    if(items_in_buf==0){
        new_index = 0;
    }
    else{
        // increment index and check for loop-around
        new_index = index+1;
        if(new_index >= size) new_index = 0;
    }

    // copy the data into our buffer
    memcpy(&data[new_index], new_packet, sizeof(img_ringbuf_packet));

    // bump index and increment number of items if necessary
    index = new_index;
    if(items_in_buf < size){
        items_in_buf++;
    }

    // all done, save the timestamp and unlock mutex
    latest_timestamp_ns = new_packet->metadata.timestamp_ns;
    buf_mutex.unlock();
    return 0;
}

int RingBuffer::get_data_at_position(int position, img_ringbuf_packet* result){
    // sanity checks
    if(unlikely(data==NULL || result==NULL)){
        fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
        return -1;
    }
    if(unlikely(position<0)){
        fprintf(stderr,"ERROR in %s, position must be >= 0\n", __FUNCTION__);
        return -1;
    }
    if(unlikely(!initialized)){
        fprintf(stderr,"ERROR in %s, ringbuf uninitialized\n", __FUNCTION__);
        return -1;
    }

    // about to start reading the buffer, lock the mutex
    buf_mutex.lock();

    // silently return if user requested a position beyond buffer size
    if(position >= size){
        buf_mutex.unlock();
        return -3;
    }
    // silently return if user requested an item that hasn't been added yet
    if(position >= items_in_buf){
        buf_mutex.unlock();
        return -2;
    }

    // return index is just latest index minus position due to the order we keep
    // data (populated from left to right)
    int return_index = index - position;

    // check for looparound
    if(return_index < 0){
        return_index += size;
    }

    // write out data
    *result = data[return_index];

    // all done, unlock mutex
    buf_mutex.unlock();

    return 0;
}


img_ringbuf_packet* RingBuffer::get_data_ptr_at_position(int position){
    // return index is just latest index minus position due to the order we keep
    // data (populated from left to right)
    int return_index = index - position;

    // check for looparound
    if(return_index < 0){
        return_index += size;
    }

    // write out data
    return &data[return_index];
}


int64_t RingBuffer::get_timestamp_at_position(int position){
    // silently return if user requested an item that hasn't been added yet
    if(position >= items_in_buf){
        return -2;
    }

    // return index is just latest index minus position due to the order we keep
    // data (populated from left to right)
    int return_index = index - position;

    // check for looparound
    if(return_index < 0){
        return_index += size;
    }

    // return the requested timestamp
    return data[return_index].metadata.timestamp_ns;
}


int RingBuffer::get_data_at_time(int64_t timestamp_ns, img_ringbuf_packet* result){
    // sanity checks
    if(unlikely(data==NULL)){
        fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
        return -1;
    }
    if(unlikely(!initialized)){
        fprintf(stderr,"ERROR in %s, ringbuf uninitialized\n", __FUNCTION__);
        return -1;
    }
    if(unlikely(timestamp_ns<=0)){
        fprintf(stderr,"ERROR in %s, requested timestamp must be >0\n", __FUNCTION__);
        return -1;
    }
    if(items_in_buf < 2){
        fprintf(stderr,"ERROR in %s, not enough data in buffer!\n", __FUNCTION__);
        return -2;
    }

    // about to start messing with the buffer, lock the mutex
    buf_mutex.lock();

    // don't deal with timestamps older OR newer than our buffer has data for
    if(timestamp_ns > (latest_timestamp_ns)){
        fprintf(stderr,"ERROR in %s, timestamp too new\n", __FUNCTION__);
        buf_mutex.unlock();
        return -3;
    }
    if(timestamp_ns < get_timestamp_at_position(items_in_buf-1)){
        fprintf(stderr, "ERROR in %s, requested timestamp older than oldest member in buffer\n", __FUNCTION__);
        buf_mutex.unlock();
        return -4;
    }

    // now go searching through the buffer to find which two entries to 
    // interpolate between, starting from newest. TODO: binary search
    else{
        for(int i=0;i<items_in_buf;i++){
            // timestamp to check at this point
            int64_t ts_at_i = get_timestamp_at_position(i);

            // found the right value! no interpolation needed
            if(ts_at_i == timestamp_ns){
                img_ringbuf_packet* _data_ptr = get_data_ptr_at_position(i);
                memcpy(result, _data_ptr, sizeof(img_ringbuf_packet));
                buf_mutex.unlock();
                return 0;
            }
        }
    }

    buf_mutex.unlock();
    fprintf(stderr, "ERROR: Failed to find packet with timestamp %ld\n", timestamp_ns);
    return -5;
}
