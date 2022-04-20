/**
 * rc_transform_ringbuf.c
 *
 *
 * @author     james@modalai.com
 * @date       2021
 */



#include <stdio.h>
#include <stdint.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "rc_ov_ringbuf.h"


#ifndef unlikely
#define unlikely(x)	__builtin_expect (!!(x), 0)
#endif

#ifndef likely
#define likely(x)	__builtin_expect (!!(x), 1)
#endif



int rc_ov_ringbuf_alloc(rc_ov_ringbuf_t* buf, int size)
{
    // sanity checks
    if(unlikely(buf==NULL)){
        fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
        return -1;
    }
    if(unlikely(size<2)){
        fprintf(stderr,"ERROR in %s, size must be >=2\n", __FUNCTION__);
        return -1;
    }

    // if it's already allocated, nothing to do
    if(buf->initialized && buf->size==size && buf->d!=NULL) return 0;

    // make sure it's zero'd out
    buf->size = 0;
    buf->index = 0;
    buf->items_in_buf=0;
    buf->initialized = 0;

    // allocate mem for array
    buf->d = (rc_ov_t*)calloc(size,sizeof(rc_ov_t));
    if(buf->d==NULL){
        fprintf(stderr,"ERROR in %s, failed to allocate memory\n", __FUNCTION__);
        return -1;
    }

    // write out other details
    buf->size = size;
    buf->initialized = 1;
    return 0;
}


int rc_ov_ringbuf_free(rc_ov_ringbuf_t* buf)
{
    if(unlikely(buf==NULL)){
        fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
        return -1;
    }
    if(buf->initialized){
        free(buf->d);
        // put buffer back to default
        *buf = rc_ov_ringbuf_empty();
    }
    return 0;
}


rc_ov_ringbuf_t rc_ov_ringbuf_empty(void)
{
    rc_ov_ringbuf_t new = RC_OV_RINGBUF_INITIALIZER;
    return new;
}


int rc_ov_ringbuf_insert(rc_ov_ringbuf_t* buf, rc_ov_t* new_tf)
{
    int new_index;

    // sanity checks
    if(unlikely(buf==NULL)){
        fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
        return -1;
    }

    // we are about to interact with the ringbuf, lock the mutex
    pthread_mutex_lock(&buf->mutex);

    // more sanity checks
    if(unlikely(!buf->initialized)){
        pthread_mutex_unlock(&buf->mutex);
        fprintf(stderr,"ERROR in %s, ringbuf uninitialized\n", __FUNCTION__);
        return -1;
    }
    if(unlikely(new_tf->ts <= buf->latest_ts)){
        pthread_mutex_unlock(&buf->mutex);
        fprintf(stderr,"ERROR in %s, detected timestamp out of order\n", __FUNCTION__);
        return -1;
    }

    // if this is the first thing to be entered make sure to start at zero
    if(buf->items_in_buf==0){
        new_index = 0;
    }
    else{
        // increment index and check for loop-around
        new_index = buf->index+1;
        if(new_index >= buf->size) new_index = 0;
    }

    // copy the data into our buffer
    memcpy(&buf->d[new_index], new_tf, sizeof(rc_ov_t));

    // bump index and increment number of items if necessary
    buf->index = new_index;
    if(buf->items_in_buf < buf->size){
        buf->items_in_buf++;
    }

    // all done, save the timestamp and unlock mutex
    buf->latest_ts = new_tf->ts;
    pthread_mutex_unlock(&buf->mutex);
    return 0;
}

int rc_ov_ringbuf_get_ov_at_pos(rc_ov_ringbuf_t* buf, int position, rc_ov_t* result)
{
    // sanity checks
    if(unlikely(buf==NULL || result==NULL)){
        fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
        return -1;
    }
    if(unlikely(position<0)){
        fprintf(stderr,"ERROR in %s, position must be >= 0\n", __FUNCTION__);
        return -1;
    }
    if(unlikely(!buf->initialized)){
        fprintf(stderr,"ERROR in %s, ringbuf uninitialized\n", __FUNCTION__);
        return -1;
    }

    // about to start reading the buffer, lock the mutex
    pthread_mutex_lock(&buf->mutex);

    // silently return if user requested a position beyond buffer size
    if(position >= buf->size){
        pthread_mutex_unlock(&buf->mutex);
        return -3;
    }
    // silently return if user requested an item that hasn't been added yet
    if(position >= buf->items_in_buf){
        pthread_mutex_unlock(&buf->mutex);
        return -2;
    }

    // return index is just latest index minus position due to the order we keep
    // data (populated from left to right)
    int return_index = buf->index - position;

    // check for looparound
    if(return_index < 0){
        return_index += buf->size;
    }

    // write out data
    *result = buf->d[return_index];

    // all done, unlock mutex
    pthread_mutex_unlock(&buf->mutex);

    return 0;
}


// Fetches the timestamp which is 'position' steps behind the latest
// This is unprotected, for internal use only! Be careful!
static int64_t _get_ts_at_pos(rc_ov_ringbuf_t* buf, int position)
{
    // silently return if user requested an item that hasn't been added yet
    if(position >= buf->items_in_buf){
        return -2;
    }

    // return index is just latest index minus position due to the order we keep
    // data (populated from left to right)
    int return_index = buf->index - position;

    // check for looparound
    if(return_index < 0){
        return_index += buf->size;
    }

    // return the requested timestamp
    return buf->d[return_index].ts;
}


// Fetches a pointer to the tfv which is 'position' steps behind the latest
// This is unprotected, for internal use only! Be careful!
static rc_ov_t* _get_ov_ptr_at_pos(rc_ov_ringbuf_t* buf, int position)
{
    // return index is just latest index minus position due to the order we keep
    // data (populated from left to right)
    int return_index = buf->index - position;

    // check for looparound
    if(return_index < 0){
        return_index += buf->size;
    }

    // write out data
    return &buf->d[return_index];
}



static int rc_ov_interpolate(rc_ov_t* A, rc_ov_t* B, double h, rc_ov_t* out)
{
	// linearly interpolate angular velocity
    float dif = B->last_angular_velocity_data[0] - A->last_angular_velocity_data[0];
    float new = A->last_angular_velocity_data[0] + (float)((double)dif * h);
    out->last_angular_velocity_data[0] = new;
    dif = B->last_angular_velocity_data[1] - A->last_angular_velocity_data[0];
    new = A->last_angular_velocity_data[1] + (float)((double)dif * h);
    out->last_angular_velocity_data[1] = new;
    dif = B->last_angular_velocity_data[2] - A->last_angular_velocity_data[0];
    new = A->last_angular_velocity_data[2] + (float)((double)dif * h);
    out->last_angular_velocity_data[2] = new;

	// also populate output timestamp if input timestamps are valid
	if(A->ts>0 && B->ts>0){
		int64_t diff = B->ts - A->ts;
		int64_t new = A->ts + (int64_t)((double)diff*h);
		out->ts = new;
	}
	else{
		out->ts = -1;
	}

	return 0;
}

int rc_ov_ringbuf_get_ov_at_time(rc_ov_ringbuf_t* buf, int64_t ts, rc_ov_t* result)
{
    // sanity checks
    if(unlikely(buf==NULL)){
        fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
        return -1;
    }
    if(unlikely(!buf->initialized)){
        fprintf(stderr,"ERROR in %s, ringbuf uninitialized\n", __FUNCTION__);
        return -1;
    }
    if(unlikely(ts<=0)){
        fprintf(stderr,"ERROR in %s, requested timestamp must be >0\n", __FUNCTION__);
        return -1;
    }
    if(buf->items_in_buf < 2){
        return -2;
    }

    // about to start messing with the buffer, lock the mutex
    pthread_mutex_lock(&buf->mutex);

    // allow timestamps up to 0.2s newer than our last position record
    if(ts > (buf->latest_ts+200000000)){
        fprintf(stderr,"ERROR in %s, timestamp too new\n", __FUNCTION__);
        pthread_mutex_unlock(&buf->mutex);
        return -3;
    }
    // don't deal with timestamps older than our buffer has data for
    if(ts < _get_ts_at_pos(buf,buf->items_in_buf-1)){
        fprintf(stderr, "ERROR in %s, requested timestamp older than oldest member in buffer\n", __FUNCTION__);
        pthread_mutex_unlock(&buf->mutex);
        return -4;
    }

    // next logic is going to be to find the two transforms to interpolate over
    rc_ov_t *tfv_ptr_before, *tfv_ptr_after;
    
    // check for timestamp newer than we have record of, if so, extrapolate given
    // the two most recent records
    if(ts > buf->latest_ts){
        tfv_ptr_before = _get_ov_ptr_at_pos(buf,1);
        tfv_ptr_after  = _get_ov_ptr_at_pos(buf,0);
    }

    // now go searching through the buffer to find which two entries to 
    // interpolate between, starting from newest. TODO: binary search
    else{
        for(int i=0;i<buf->items_in_buf;i++){
            // timestamp to check at this point
            int64_t ts_at_i = _get_ts_at_pos(buf,i);

            // found the right value! no interpolation needed
            if(ts_at_i == ts){
                rc_ov_t* tfv_ptr = _get_ov_ptr_at_pos(buf,i);
                memcpy(result, &tfv_ptr, sizeof(rc_ov_t));
                pthread_mutex_unlock(&buf->mutex);
                return 0;
            }

            // once we get a timestamp older than requested ts, we have found the
            // right interval, grab the appropriate transforms
            if(ts_at_i < ts){
                tfv_ptr_before = _get_ov_ptr_at_pos(buf,i);
                tfv_ptr_after  = _get_ov_ptr_at_pos(buf,i-1);
                break;
            }
        }
    }

    // calculate interpolation constant h which is between 0 and 1.
    // 0 would be right at tfv_ptr_before, 1 would be right at tfv_ptr_after.
    double h = (double)(ts-tfv_ptr_before->ts) / (double)(tfv_ptr_after->ts-tfv_ptr_before->ts);

    // do linear interpolation
    int ret = rc_ov_interpolate(tfv_ptr_before, tfv_ptr_after, h, result);
    
    pthread_mutex_unlock(&buf->mutex);
    return ret;
}

