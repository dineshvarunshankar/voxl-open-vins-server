/*******************************************************************************
 * Copyright 2020 ModalAI Inc.
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

#ifndef CPU_MONITOR_INTERFACE_H
#define CPU_MONITOR_INTERFACE_H


// commands that can be sent to the voxl-cpu-monitor control pipe
// be sure to update the CONTROL_COMMANDS array below if you add anything here
#define COMMAND_SET_CPU_MODE_PERF	"set_cpu_mode_perf"
#define COMMAND_SET_CPU_MODE_AUTO	"set_cpu_mode_auto"
#define COMMAND_SET_CPU_MODE_POWERSAVE		"set_cpu_mode_powersave"
#define COMMAND_SET_CPU_MODE_CONSERVATIVE	"set_cpu_mode_conservative"
#define COMMAND_SET_FAN_MODE_OFF	"set_fan_mode_off"
#define COMMAND_SET_FAN_MODE_SLOW	"set_fan_mode_slow"
#define COMMAND_SET_FAN_MODE_MAX	"set_fan_mode_max"
#define COMMAND_SET_FAN_MODE_AUTO	"set_fan_mode_auto"
#define COMMAND_SET_LED_MODE_OFF	"set_led_mode_off"
#define COMMAND_SET_LED_MODE_ON		"set_led_mode_on"
#define COMMAND_SET_LED_MODE_COLOR	"set_led_mode_color"
#define COMMAND_SET_STANDBY_ON		"set_standby_on"
#define COMMAND_SET_STANDBY_OFF		"set_standby_off"
#define COMMAND_SET_STANDBY_AUTO	"set_standby_auto"

// Be sure to update these if you add anything to the above list!
#define CPU_MON_CONTROL_COMMANDS (\
COMMAND_SET_CPU_MODE_POWERSAVE"," \
COMMAND_SET_CPU_MODE_CONSERVATIVE"," \
COMMAND_SET_CPU_MODE_PERF"," \
COMMAND_SET_CPU_MODE_AUTO"," \
COMMAND_SET_FAN_MODE_OFF"," \
COMMAND_SET_FAN_MODE_SLOW"," \
COMMAND_SET_FAN_MODE_MAX"," \
COMMAND_SET_FAN_MODE_AUTO"," \
COMMAND_SET_LED_MODE_OFF"," \
COMMAND_SET_LED_MODE_ON"," \
COMMAND_SET_LED_MODE_COLOR"," \
COMMAND_SET_STANDBY_ON"," \
COMMAND_SET_STANDBY_OFF"," \
COMMAND_SET_STANDBY_AUTO)


#define CPU_MON_MAGIC_NUMBER	(0x564F584C)
#define CPU_MON_MAX_CPU					8 // supports up to 8 cores

// thresholds to trigger overheat and overload flags
#define CPU_STATS_CPU_OVERLOAD_THRESHOLD	80.0f	// %80 total cpu use to trigger CPU Overload flag
#define CPU_STATS_CPU_OVERHEAT_THRESHOLD	90.0f	// 90C max cpu temp to trigger CPU Overheat flag

// flags that can be set in cpu_stats_t
#define CPU_STATS_FLAG_CPU_MODE_AUTO	(1<<0)	// indicates CPU clock scaling is in Auto mode
#define CPU_STATS_FLAG_CPU_MODE_PERF	(1<<1)	// indicates CPU clock scaling is in performance mode
#define CPU_STATS_FLAG_GPU_MODE_AUTO	(1<<2)	// indicates GPU clock scaling is in Auto mode
#define CPU_STATS_FLAG_GPU_MODE_PERF	(1<<3)	// indicates GPU clock scaling is in performance mode
#define CPU_STATS_FLAG_CPU_OVERLOAD		(1<<4)	// indicates the total CPU usage is over %80 use
#define CPU_STATS_FLAG_CPU_OVERHEAT		(1<<5)	// indicates the max CPU core temp is over 90C
#define CPU_STATS_FLAG_CPU_MODE_POWERSAVE		(1<<6)	// indicates CPU clock scaling is in powersave mode
#define CPU_STATS_FLAG_CPU_MODE_CONSERVATIVE	(1<<7)	// indicates CPU clock scaling is in conservative mode
#define CPU_STATS_FLAG_STANDBY_ACTIVE	(1<<8)	// indicates standby mode is active


/**
 * describes the current state of the CPU
 *
 * This packet is 152 bytes long.
 */
typedef struct cpu_stats_t{
	uint32_t magic_number;		///< unique 32-bit number used to signal the beginning of a packet
	int32_t  num_cpu;			///< number of CPU's
	float    cpu_freq[CPU_MON_MAX_CPU];	///< CPU freq (MHz)
	float    cpu_t[CPU_MON_MAX_CPU];	///< CPU temperature (C)
	float    cpu_t_max;			///< max core temp (C)
	float    cpu_load[CPU_MON_MAX_CPU];	///< CPU load (%)
	float    cpu_load_10s;		///< CPU load for past 10 seconds (%)
	float    total_cpu_load;	///< calculate total cpu load (%)
	float    gpu_freq;			///< GPU freq (MHz)
	float    gpu_t;				///< GPU temperature (C)
	float    gpu_load;			///< current gpu load (%)
	float    gpu_load_10s;		///< gpu load for past 10 seconds (%)
	float    mem_t;				///< Memory Temperature
	uint32_t mem_total_mb;		///< total memory in MB
	uint32_t mem_use_mb;		///< memory used by processes in MB, not including cache
	uint32_t flags;				///< flags
	uint32_t reserved;			///< spare reserved bytes for future expansion
}__attribute__((packed)) cpu_stats_t;


/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync. Here we use 13 packets because the
 * cpu_stats_t packet is 152-bytes long and this gives a buffer just under 2k
 *
 * Note this is NOT the size of the pipe which can hold more. This is just the
 * read buffer size allocated on the heap into which data from the pipe is read.
 */
#define CPU_STATS_RECOMMENDED_READ_BUF_SIZE	(sizeof(cpu_stats_t) * 13)


/**
 * We recommend tof servers use a pipe size of 64kB which is also the Linux
 * default pipe size. This allows 431 cpu_stats_t packets to be stored in the
 * pipe before losing data.
 */
#define CPU_STATS_RECOMMENDED_PIPE_SIZE	(64*1024)


/**
 * @brief      Use this to simultaneously validate that the bytes from a pipe
 *             contains valid data, find the number of valid packets
 *             contained in a single read from the pipe, and cast the raw data
 *             buffer as an cpu_stats_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate a cpu_stats_t array separate from the pipe read buffer.
 *             The data can be read straight out of the pipe read buffer, much
 *             like reading data directly out of a mavlink_message_t message.
 *
 *             However, this does mean the user should finish processing this
 *             data before returning the pipe data callback which triggers a new
 *             read() from the pipe.
 *
 * @param[in]  data       pointer to pipe read data buffer
 * @param[in]  bytes      number of bytes read into that buffer
 * @param[out] n_packets  number of valid packets received
 *
 * @return     Returns the same data pointer provided by the first argument, but
 *             cast to an tag_detection_t* struct for convenience. If there was an
 *             error then NULL is returned and n_packets is set to 0
 */
static inline cpu_stats_t* modal_cpu_validate_pipe_data(char* data, int bytes, int* n_packets)
{
	cpu_stats_t* new_ptr = (cpu_stats_t*) data;
	*n_packets = 0;

	// basic sanity checks
	if(bytes<0){
		fprintf(stderr, "ERROR validating cpu monitor data received through pipe: number of bytes = %d\n", bytes);
		return NULL;
	}
	if(data==NULL){
		fprintf(stderr, "ERROR validating cpu monitor data received through pipe: got NULL data pointer\n");
		return NULL;
	}
	if(bytes%sizeof(cpu_stats_t)){
		fprintf(stderr, "ERROR validating cpu monitor data received through pipe: read partial packet\n");
		fprintf(stderr, "read %d bytes, but it should be a multiple of %zu\n", bytes, sizeof(cpu_stats_t));
		return NULL;
	}

	// calculate number of packets locally until we validate each packet
	int n_packets_tmp = bytes/sizeof(cpu_stats_t);

	// check if any packets failed the magic number check
	int i, n_failed = 0;
	for(i=0;i<n_packets_tmp;i++){
		if(new_ptr[i].magic_number != CPU_MON_MAGIC_NUMBER) n_failed++;
	}
	if(n_failed>0){
		fprintf(stderr, "ERROR validating cpu monitor data received through pipe: %d of %d packets failed\n", n_failed, n_packets_tmp);
		return NULL;
	}

	*n_packets = n_packets_tmp;
	return new_ptr;
}



#endif // MONITOR_INTERFACE_H
