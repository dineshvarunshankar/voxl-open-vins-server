#ifndef VOXL_OV_RC_RB_H_
#define VOXL_OV_RC_RB_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include <pthread.h>
#include <modal_pipe.h>


typedef struct rc_ov_t{
    float last_angular_velocity_data[3];
    int64_t ts;
} rc_ov_t; 

/**
 * @brief      Struct containing state of a ringbuffer and pointer to
 * dynamically allocated memory.
 */
typedef struct rc_ov_ringbuf_t {
    rc_ov_t* d;			    ///< pointer to dynamically allocated data
    int size;				///< number of elements the buffer can hold
    int index;				///< index of the most recently added value
    int items_in_buf;		///< number of items in the buffer, between 0 and size
    int initialized;		///< flag indicating if memory has been allocated for the buffer
    int64_t latest_ts;		///< latest timestamp added to buffer
    pthread_mutex_t mutex;	///< mutex to protect the buffer during read/write operations
} rc_ov_ringbuf_t;



/**
 * initializer for the rc_tfv_ringbuf_t to make sure it starts zero'd out
 */
#define RC_OV_RINGBUF_INITIALIZER {\
    .d = NULL,\
    .size = 0,\
    .index = 0,\
    .items_in_buf = 0,\
    .initialized = 0,\
    .latest_ts = 0,\
    .mutex = PTHREAD_MUTEX_INITIALIZER\
}


/**
 * @brief      Allocates memory for a ring buffer and initializes an
 * rc_tfv_ringbuf_t struct.
 *
 * If buf is already the right size then it is left untouched. Otherwise any
 * existing memory allocated for buf is freed to avoid memory leaks and new
 * memory is allocated.
 *
 * @param      buf   Pointer to user's buffer
 * @param[in]  size  Number of elements to allocate space for
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_ov_ringbuf_alloc(rc_ov_ringbuf_t* buf, int size);


/**
 * @brief      Frees the memory allocated for buffer buf.
 *
 * Also set the initialized flag to 0 so other functions don't try to access
 * unallocated memory.
 *
 * @param      buf   Pointer to user's buffer
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_ov_ringbuf_free(rc_ov_ringbuf_t* buf);


/**
 * @brief      returns an empty tf ringbuf
 *
 */
rc_ov_ringbuf_t rc_ov_ringbuf_empty(void);


/**
 * @brief      Puts a new transform into the ring buffer.
 *
 *             This is intended for dynamic transforms and your transform must
 *             have a timestamp >0
 *
 *             Use this when you have no velocity data. Use
 *             rc_tf_ringbuf_insert_tfv if you have velocity data.
 *
 *             timestamps must be monotonically increasing, if you try to add a
 *             new transform with a timestamp <= the last added timestamp this
 *             will fail.
 *
 * @param      buf     Pointer to user's buffer
 * @param      new_tf  The new tf to be inserted
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_ov_ringbuf_insert(rc_ov_ringbuf_t* buf, rc_ov_t* new_ov);



/**
 * @brief      Fetches the entry which is 'position' steps behind the last value
 *             added to the buffer.
 *
 *             If 'position' is given as 0 then the most recent entry is
 *             returned. The position obviously can't be larger than size-1.
 *             This will also check and return -2 if the buffer hasn't been
 *             filled up enough to go back that far in time.
 *
 * @param      buf     Pointer to user's buffer
 * @param[in]  pos     steps back in the buffer to fetch the entry from
 * @param      result  pointer to write the result into.
 *
 * @return     0 on success, -2 if there is not enough data in the buffer, -3 if the
 *             position exceeds buffer size, and -1 on other error
 */
int rc_ov_ringbuf_get_ov_at_pos(rc_ov_ringbuf_t* buf, int pos, rc_ov_t* result);


/**
 * @brief      fetches transform at timestamp ts
 *
 *             Interpolates between nearest two recorded transforms. If the
 *             buffer happens to contain an exact match at the requested
 *             timestamp then that tranform will be returned. If interpolation
 *             is necessary, then linear interpolation is used for the
 *             translation and SLERP is used to interpolate between rotations.
 *
 * @param[in]  buf   Pointer to user's buffer
 * @param[in]  ts    requested timestamp
 * @param[out] R     estimated rotation matrix at requested timestamp
 * @param[out] T     estimated translation at requested timestamp
 *
 * @return     Returns 0 if a valid value was found, -1 on error. -2 if there
 *             simply wasn't sufficient data in the buffer, such as when waiting
 *             for VIO to initialize or if the requested timestamp is too old
 *             return -3 if the requested timestamp was too new
 *             return -4 if the requested timestamp was too old
 */
int rc_ov_ringbuf_get_ov_at_time(rc_ov_ringbuf_t* buf, int64_t ts, rc_ov_t* result);

#ifdef __cplusplus
}
#endif

#endif // VOXL_OV_RC_RB_H_ 