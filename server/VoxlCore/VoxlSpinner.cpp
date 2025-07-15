/**
 * @file VoxlSpinner.cpp
 * @brief Main server implementation for VOXL OpenVINS
 * @author Zauberflote
 * @date 2025
 * @version 1.0
 *
 * This file contains the main server implementation for the VOXL OpenVINS system.
 * It handles initialization, pipe management, signal handling, and the main event loop.
 *
 * The server provides:
 * - VIO system initialization and configuration
 * - Pipe-based communication with sensors and clients
 * - Signal handling for graceful shutdown
 * - Resource management and cleanup
 * - Main event loop for continuous operation
 *
 * The server architecture follows a producer-consumer pattern where:
 * - IMU and camera data are received through pipe clients
 * - VIO processing is performed by the OpenVINS library
 * - Results are published through pipe servers to external clients
 * - System state is managed through atomic variables and mutexes
 */

#include <core/VioManager.h>
#include <core/VioManagerOptions.h>
#include "ImuMinimal.h"
#include "CameraManager.h"
#include <modal_pipe.h>
#include <modal_json.h>
#include <voxl_common_config.h>
#include "VoxlCommon.h"
#include "VoxlVars.h"
#include "VoxlConfigure.h"
#include <unistd.h>
#include <signal.h>
#include <sched.h>
#include <getopt.h>

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

/**
 * @brief Clean shutdown function for the server
 *
 * Performs a clean shutdown of the VOXL OpenVINS server, including
 * thread cancellation, pipe cleanup, camera shutdown, and resource
 * deallocation.
 *
 * The shutdown process is designed to be robust and handle various
 * failure scenarios gracefully. It ensures that all resources are
 * properly cleaned up to prevent system resource leaks.
 *
 * @param ret Exit code for the process (0 for success, non-zero for error)
 */
static void _quit(int ret);

/**
 * @brief Connect to client pipes (IMU and cameras)
 *
 * Establishes connections to the IMU service and camera services,
 * initializes the camera manager, and sets up the necessary pipe
 * channels for data communication.
 *
 * This function is critical for the data acquisition pipeline and
 * must complete successfully before the main processing loop can begin.
 *
 * @return 0 on success, -1 on failure
 * @see connect_imu_service()
 * @see voxl::CameraManager::initialize()
 */
static int connect_client_pipes(void);

/**
 * @brief Create server pipes for data output
 *
 * Creates the server pipes that will be used to output VIO data
 * to clients. Sets up extended VIO data, simple VIO data, and
 * status pipes with appropriate configurations.
 *
 * The pipes are configured with optimal buffer sizes and metadata
 * to ensure efficient data transmission to client applications.
 *
 * @return 0 on success, -1 on failure
 */
static int create_server_pipes(void);

/**
 * @brief Print usage information for command line options
 *
 * Displays help information about available command line options
 * and their usage. This function is called when invalid options
 * are provided or when the help option is requested.
 *
 * @param program_name Name of the program (argv[0])
 */
static void print_usage(const char *program_name);

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Clean shutdown function for the server
 *
 * This function performs a clean shutdown of the VOXL OpenVINS server,
 * including thread cancellation, pipe cleanup, camera shutdown, and
 * resource deallocation.
 *
 * The shutdown process includes:
 * - Enabling thread cancellation for all threads
 * - Closing all pipe connections (client and server)
 * - Shutting down the camera manager
 * - Removing the PID file
 * - Cleaning up image ring buffer
 * - Forcing process exit
 *
 * The function uses _exit() to ensure immediate termination and
 * prevent any cleanup issues that might occur during normal exit.
 *
 * @param ret Exit code for the process (0 for success, non-zero for error)
 */
static void _quit(int ret)
{
	printf("QUIT REQUESTED\n");

	// FORCE ALL THREADS TO BE CANCELABLE IMMEDIATELY
	if (en_debug)
		printf("Setting process-wide thread cancelation policy...\n");
	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

	// FORCE KILL THREADS CREATED BY THE PIPE CLIENT
	if (en_debug)
		printf("Forcibly closing all pipe connections...\n");
	pipe_server_close_all();
	pipe_client_close_all();

	// WAIT FOR THREADS TO TERMINATE
	if (en_debug)
		printf("Waiting for threads to terminate...\n");
	usleep(100000); // 100ms

	printf("Shutting down cameras...\n");
	voxl::CameraManager::getInstance().shutdown();
	printf("Camera shutdown complete\n");

	// REMOVE PID
	remove_pid_file(PROCESS_NAME);

	// DELETE IMG RINGBUF -- THIS MAY BE UNNECESSARY AS WE ARE NOT DOING OVERLAYS
	if (img_ringbuf)
		delete img_ringbuf;

	if (ret == 0)
		printf("Exiting Cleanly\n");
	else
		printf("error code: %d\n", ret);

	if (en_debug)
		printf("Forcibly exiting process now!\n");
	_exit(ret); // FORCE EXIT
	return;
}

/**
 * @brief Connect to client pipes (IMU and cameras)
 *
 * This function establishes connections to the IMU service and camera
 * services, initializes the camera manager, and sets up the necessary
 * pipe channels for data communication.
 *
 * The function performs the following operations:
 * - Connects to the IMU service and validates the connection
 * - Determines the number of cameras to initialize
 * - Initializes the camera manager with camera configurations
 * - Stores channel IDs for each camera for later reference
 *
 * The function is designed to fail fast if any critical component
 * cannot be initialized, ensuring system stability.
 *
 * @return 0 on success, -1 on failure
 * @see connect_imu_service()
 * @see voxl::CameraManager::initialize()
 */
static int connect_client_pipes(void)
{
	fprintf(stderr, "connecting client pipes\n");

	// CONNECT TO IMU SERVICE
	int ret = connect_imu_service();
	if (ret != 0)
	{
		fprintf(stderr, "failed to connect imu service\n");
		_quit(-1);
	}
	fprintf(stderr, "imu pipe name: %s\n", imu_name);

	// CHECK HOW MANY CAMERAS WE SHOULD INITIALIZE
	cameras_used = cam_info_vec.size();
	printf("Initializing camera system with %d cameras\n", cameras_used);

	// CONNECT CAMERAS AND INITIALIZE THEM
	ret = voxl::CameraManager::getInstance().initialize(cam_info_vec) ? 0 : -1;
	if (ret != 0)
	{
		fprintf(stderr, "failed to connect cam service\n");
		_quit(-1);
	}

	if (ret == 0)
	{
		// STORE CHANNEL ID FOR EACH CAMERA ID
		size_t i = 0;
		for (const auto &camera : voxl::CameraManager::getInstance().getAllCameras())
		{
			if (i < MAX_CAM_CNT)
			{ // 6 CAMERAS HARDCODED MAXIMUM
				camera_pipe_channels[i] = camera->get_channel();
				if (en_debug)
				{
					printf("Camera %zu using channel %d\n", i, camera_pipe_channels[i]);
				}
				i++;
			}
		}
	}

	return 0;
}

/**
 * @brief Create server pipes for data output
 *
 * This function creates the server pipes that will be used to output
 * VIO data to clients. It sets up extended VIO data, simple VIO data,
 * and status pipes with appropriate configurations.
 *
 * The function creates three main pipes:
 * - Extended VIO data pipe: Comprehensive VIO data with full state information
 * - Simple VIO data pipe: Essential pose and velocity information
 * - Status pipe: System status and health information
 *
 * Each pipe is configured with:
 * - Appropriate buffer sizes for data throughput
 * - JSON metadata including IMU service information
 * - Control command support (currently disabled)
 *
 * The pipes are designed to handle high-frequency data transmission
 * with minimal latency for real-time applications.
 *
 * @return 0 on success, -1 on failure
 */
static int create_server_pipes(void)
{
	// CREATE RE-USABLE FLAG FOR CONTROL PIPES
	int flags = SERVER_FLAG_EN_CONTROL_PIPE;

	// OV EXTENDED
	pipe_info_t info_extended =
		{
			OV_VIO_EXTENDED_NAME,			   // name
			OV_VIO_EXTENDED_LOCATION,		   // location
			"ext_vio_data_t",				   // type
			PROCESS_NAME,					   // server_name
			EXT_VIO_RECOMMENDED_READ_BUF_SIZE, // size_bytes
			0								   // server_pid
		};

	if (pipe_server_create(EXTENDED_CH, info_extended, 0))
	{
		printf("pipe_server_create(EXTENDED_CH, info_extended, flags) failed\n");
		_quit(-1);
	}

	// ADD OPTIONAL IMU FIELD IN JSON
	cJSON *json = pipe_server_get_info_json_ptr(EXTENDED_CH);
	cJSON_AddStringToObject(json, "imu", imu_name);
	pipe_server_update_info(EXTENDED_CH);

	// SET CONTROL CALLBACKS FOR EXTENDED PIPE
	pipe_server_set_control_cb(EXTENDED_CH, voxl::Publisher::getInstance().ov_vio_control_pipe_cb, NULL);
	pipe_server_set_available_control_commands(EXTENDED_CH, OV_VIO_CONTROL_COMMANDS);

	// OV SIMPLE
	pipe_info_t info_simple =
		{
			OV_VIO_SIMPLE_NAME,		   // name
			OV_VIO_SIMPLE_LOCATION,	   // location
			"vio_data_t",			   // type
			PROCESS_NAME,			   // server_name
			VIO_RECOMMENDED_PIPE_SIZE, // size_bytes
			0						   // server_pid
		};

	if (pipe_server_create(SIMPLE_CH, info_simple, flags))
	{
		printf("pipe_server_create(SIMPLE_CH, info_simple, flags) failed\n");
		_quit(-1);
	}

	// ADD OPTIONAL IMU FIELD IN JSON
	json = pipe_server_get_info_json_ptr(SIMPLE_CH);
	cJSON_AddStringToObject(json, "imu", imu_name);
	pipe_server_update_info(SIMPLE_CH);

	// SET CONTROL CALLBACKS FOR SIMPLE PIPE
	pipe_server_set_control_cb(SIMPLE_CH, voxl::Publisher::getInstance().ov_vio_control_pipe_cb, NULL);
	pipe_server_set_available_control_commands(SIMPLE_CH, OV_VIO_CONTROL_COMMANDS);

	// OV STATUS
	pipe_info_t info_status =
		{
			OV_STATUS_NAME,			   // name
			OV_STATUS_LOCATION,		   // location
			"ov_status_t",			   // type
			PROCESS_NAME,			   // server_name
			VIO_RECOMMENDED_PIPE_SIZE, // size_bytes
			0						   // server_pid
		};

	if (pipe_server_create(OVSTATUS_CH, info_status, flags))
	{
		printf("pipe_server_create(SIMPLE_CH, info_status, flags) failed\n");
		_quit(-1);
	}

	return 0;
}

/**
 * @brief Print usage information for command line options
 *
 * Displays help information about available command line options
 * and their usage. This function is called when invalid options
 * are provided or when the help option is requested.
 *
 * @param program_name Name of the program (argv[0])
 */
static void print_usage(const char *program_name)
{
	printf("Usage: %s [OPTIONS]\n", program_name);
	printf("VOXL OpenVINS Server - Visual Inertial Odometry System\n\n");
	printf("Options:\n");
	printf("  -d, --debug     Enable debug output and detailed logging\n");
	printf("  -v, --verbose   Enable verbose output and status information\n");
	printf("  -c, --config    Configuration only mode (load and validate config)\n");
	printf("  -h, --help      Display this help message\n\n");
	printf("Examples:\n");
	printf("  %s              Run server with default settings\n", program_name);
	printf("  %s -d           Run server with debug output enabled\n", program_name);
	printf("  %s -v -c        Load configuration only with verbose output\n", program_name);
	printf("  %s --debug --verbose  Run with both debug and verbose output\n", program_name);
}

// ============================================================================
// MAIN FUNCTION
// ============================================================================

using namespace ov_msckf;

/**
 * @brief Main entry point for VOXL OpenVINS server
 *
 * This function initializes the VIO system, sets up pipe communications,
 * configures signal handling, and runs the main event loop. It handles
 * the complete lifecycle of the VIO server from startup to shutdown.
 *
 * The main function performs the following operations:
 * - Loads and validates server configuration files
 * - Synchronizes camera configurations with system services
 * - Creates and configures the VIO manager with OpenVINS
 * - Sets up process management (PID file, signal handling)
 * - Configures process priority and CPU affinity
 * - Creates server pipes for data output
 * - Connects to client pipes (IMU and cameras)
 * - Runs the main event loop until shutdown is requested
 *
 * The function implements a robust initialization sequence that ensures
 * all components are properly configured before starting the main loop.
 * It also handles graceful shutdown through signal handling.
 *
 * @param argc Number of command line arguments (currently unused)
 * @param argv Array of command line argument strings (currently unused)
 * @return 0 on successful execution, non-zero on error
 *
 * @note The function uses _exit() for termination, so normal return paths
 *       are not reached. This ensures immediate process termination.
 */
int main(int argc, char *argv[])
{
	// Parse command line options
	int opt;
	const char *short_options = "dvhc";
	struct option long_options[] = {
		{"debug", no_argument, 0, 'd'},
		{"verbose", no_argument, 0, 'v'},
		{"help", no_argument, 0, 'h'},
		{"config", no_argument, 0, 'c'},
		{0, 0, 0, 0}
	};

	while ((opt = getopt_long(argc, argv, short_options, long_options, NULL)) != -1) {
		switch (opt) {
			case 'd':
				en_debug = 1;
				if (en_verbose) {
					printf("Debug mode enabled\n");
				}
				break;
			case 'v':
				en_verbose = 1;
				if (en_verbose) {
					printf("Verbose mode enabled\n");
				}
				break;
			case 'c':
				config_only = 1;
				if (en_verbose) {
					printf("Configuration only mode enabled\n");
				}
				break;
			case 'h':
				print_usage(argv[0]);
				_quit(-1);
				break; // This break is unreachable but prevents compiler warning
			default:
				print_usage(argv[0]);
				break;
		}
	}

	// Check for any non-option arguments
	if (optind < argc) {
		printf("Error: Unexpected arguments provided\n");
		print_usage(argv[0]);
		return 1;
	}

	// Load the config files
	if (voxl::read_server_config() < 0)
	{
		printf("config_file_read() < 0\n");
		_quit(-1);
	}

	// If configuration only mode is enabled, exit after loading config
	if (config_only) {
		if (en_verbose) {
			printf("Configuration loaded successfully. Exiting due to config-only mode.\n");
		}
		return 0;
	}

	// read camera multicam setup and configs
	if (voxl::sync_cam_config() < 0)
	{
		fprintf(stderr, "ERROR cam_config_file_read\n");
		_quit(-1);
	}
	if (en_verbose) {
		printf("Camera configuration synchronized successfully\n");
	}

	std::string config_path = std::string(folder_base) + "/estimator_config.yaml"; // PLACEHOLDER --> CHECK LOCATION OF YAMLS POST FLASH

	// Create the VIO Manager -- Core OpenVINS state
	vio_manager_options = VioManagerOptions();
	auto parser = std::make_shared<ov_core::YamlParser>(config_path);
	
	// THIS GETS OVERWRITTEN BY THE YAML
	std::string verbosity = "SILENT";
	parser->parse_config("verbosity", verbosity);
	vio_manager_options.print_and_load(parser);
	if (en_verbose) {
		ov_core::Printer::setPrintLevel(ov_core::Printer::DEBUG);
	} else {
		ov_core::Printer::setPrintLevel(ov_core::Printer::SILENT);
	}

	vio_manager = std::unique_ptr<VioManager>(new VioManager(vio_manager_options));

	

	/* make sure another instance isn't running
	 * if return value is -3 then a background process is running with
	 * higher privaledges and we couldn't kill it, in which case we should
	 * not continue or there may be hardware conflicts. If it returned -4
	 * then there was an invalid argument that needs to be fixed.
	 */
	if (kill_existing_process(PROCESS_NAME, 2.0) < -2)
	{
		std::cerr << "ERROR: could not kill existing process" << std::endl;
		_quit(-1);
	}

	// start signal handler so we can exit cleanly
	if (enable_signal_handler() == -1)
	{
		std::cerr << "ERROR: failed to start signal handler" << std::endl;
		_quit(-1);
		kill(getpid(), SIGKILL);
	}

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	make_pid_file(PROCESS_NAME);

	// This is a heavy multithreaded process, set to medium priority so we don't
	// starve more time sensitive things like drivers and VFC
	pipe_set_process_priority(THREAD_PRIORITY_RT_MED);

	// on qrb5165 keep this process on the larger cores
	set_cpu_affinity(cpu_set_big_cores_and_gold_core());
	print_cpu_affinity();

	// NOW TELL THAT VIO IS INITIALIZING --  THREAD SAFE
	vio_state = VIO_STATE_INITIALIZING;
	vio_error_codes = 0;

	// NOW CREATE THE PIPES
	if (create_server_pipes() < 0)
	{
		printf("connect_client_pipes < 0\n");
		_quit(0);
	}

	// Connect to the client pipes and start getting data
	if (connect_client_pipes() < 0)
	{
		printf("connect_client_pipes < 0\n");
		_quit(0);
	}

	main_running = 1;

	// zeroTimeOut = boost::posix_time::microsec_clock::local_time();

	usleep(100000); // 100ms
	voxl::Publisher::getInstance().start();

	while (main_running)
	{
		usleep(1e6);
	}

	printf("Quiting OpenVINS server\n");
	// Shutdown Nicely
	_quit(0);

	// This code will never be reached due to exit() in _quit()
	return 0;
}
