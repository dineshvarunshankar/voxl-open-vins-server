/*******************************************************************************
 * Copyright 2023 ModalAI Inc.
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

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <getopt.h>
#include <string.h>

#include <modal_pipe_interfaces.h>

int en_soft = 0;
int en_blank = 0;
int en_fade = 0;
int en_something = 0;

static void _print_usage(void)
{
	printf("\n\
typical usage\n\
/# voxl-reset-vins\n\
\n\
-h, --help                 print this help message\n\
-s, --soft                 do a soft reset instead of a hard reset\n\
\n");
	return;
}


static int _parse_opts(int argc, char* argv[])
{
	static struct option long_options[] =
	{
		{"help",			no_argument,		0, 'h'},
		{"soft",			no_argument,		0, 's'},
		{0, 0, 0, 0}
	};

	while(1){
		int option_index = 0;
		int c = getopt_long(argc, argv, "hs", long_options, &option_index);

		if(c == -1) break; // Detect the end of the options.

		switch(c){
		case 0:
			// for long args without short equivalent that just set a flag
			// nothing left to do so just break.
			if (long_options[option_index].flag != 0) break;
			break;

		case 'h':
			_print_usage();
			return -1;

		case 's':
			if(en_something){
				fprintf(stderr, "ERROR can't request multiple commands\n");
				return -1;
			}
			en_soft = 1;
			en_something = 1;
			break;

		default:
			_print_usage();
			return -1;
		}
	}

	return 0;
}


int main(int argc, char* argv[])
{
	// check for options
	if(_parse_opts(argc, argv)) return -1;

	int fd = open("/run/mpa/ov/control", O_WRONLY);
	if(fd<0){
		perror("Error ov control path");
		fprintf(stderr, "make sure voxl-openvins-server is running\n");
		fprintf(stderr, "systemctl status voxl-openvins-server\n");
		return -1;
	}

	int ret;
	if(en_soft){
		printf("Sending soft reset command to voxl-openvins-server\n");
		ret = write(fd, RESET_VIO_SOFT, strlen(RESET_VIO_SOFT)+1);
	}
	else{
		printf("Sending hard reset command to voxl-openvins-server\n");
		ret = write(fd, RESET_VIO_HARD, strlen(RESET_VIO_HARD)+1);
	}


	if(ret<=0){
		perror("failed to write to control pipe");
		ret = -1;
	}
	else{
		printf("success\n");
		ret = 0;
	}

	close(fd);

	return ret;
}
