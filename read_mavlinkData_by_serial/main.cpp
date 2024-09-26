#include <cstdlib>
#include <stdbool.h>
#include <stdio.h>   // Standard input/output definitions
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <pthread.h> // This uses POSIX Threads
#include <signal.h>
#include <stdint.h>
#include "mavlink/common/mavlink.h"

struct Mavlink_Messages {
	mavlink_heartbeat_t heartbeat;
	mavlink_sys_status_t sys_status;
	mavlink_battery_status_t battery_status;
	mavlink_radio_status_t radio_status;
	mavlink_local_position_ned_t local_position_ned;
	mavlink_position_target_local_ned_t position_target_local_ned;
	mavlink_position_target_global_int_t position_target_global_int;
	mavlink_highres_imu_t highres_imu;
	mavlink_attitude_t attitude;
	mavlink_gps_status_t gps_status;
	mavlink_gps_raw_int_t gps_raw_int;
	mavlink_global_position_int_t global_position_int;
};

int fd;
pthread_mutex_t lock;

int Serial_read(uint8_t &cp){
	pthread_mutex_lock(&lock);
	int result = read(fd, &cp, 1);
	pthread_mutex_unlock(&lock);
	return result;
}

int Serial_write(char *buf, unsigned len)
{
	pthread_mutex_lock(&lock);
	const int bytesWritten = static_cast<int>(write(fd, buf, len));
	// Wait until all data has been written
	tcdrain(fd);
	pthread_mutex_unlock(&lock);
	return bytesWritten;
}

bool Serial_setup(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control){
	if(!isatty(fd)){
		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
		return false;
	}
	// Read file descritor configuration
	struct termios  config;
	if(tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return false;
	}
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

    #ifdef OLCUC
		config.c_oflag &= ~OLCUC;
	#endif

	#ifdef ONOEOT
		config.c_oflag &= ~ONOEOT;
	#endif

    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
    config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10;
    switch (baud){
		case 1200:
			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0){
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 1800:
			cfsetispeed(&config, B1800);
			cfsetospeed(&config, B1800);
			break;
		case 9600:
			cfsetispeed(&config, B9600);
			cfsetospeed(&config, B9600);
			break;
		case 19200:
			cfsetispeed(&config, B19200);
			cfsetospeed(&config, B19200);
			break;
		case 38400:
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0){
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0){
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0){
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;

		// These two non-standard (by the 70'ties ) rates are fully supported on
		// current Debian and Mac OS versions (tested since 2010).
		case 460800:
			if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0){
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 921600:
			if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0){
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		default:
			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
			return false;
			break;
	}

	// Finally, apply the configuration
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		return false;
	}

	// Done!
	return true;
}

void Serial_start(const char *uart_name, int  baudrate){
	fd = open(uart_name, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1){
		printf("failure, could not open port.\n");
		return;
	}else{
		fcntl(fd, F_SETFL, 0);
	}

    bool success = Serial_setup(baudrate, 8, 1, false, false);
	if (!success){
		printf("failure, could not configure port.\n");
		return;
	}
	if (fd <= 0){
		printf("Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", uart_name, baudrate);
		return;
	}
    printf("Connected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
}

int temp = 0;
char command[150];

void message_handle(uint8_t *cp){
	mavlink_status_t status;
	mavlink_message_t message;
	Mavlink_Messages current_messages;
	
	uint8_t msgReceived = mavlink_parse_char(MAVLINK_COMM_1, *cp, &message, &status);
	temp = message.msgid;
	//printf("msgId: %d -> ", message.msgid);
	switch (message.msgid){
		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:{
			if(temp != message.msgid){
				int32_t lat = mavlink_msg_global_position_int_get_lat(&message); // /10.000.000
				int32_t lon = mavlink_msg_global_position_int_get_lon(&message);
				int32_t alt = mavlink_msg_global_position_int_get_alt(&message);
				//mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
				printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT %d, %d, %d\n", lat, lon, alt);
			}
			break;
		}
		
		case MAVLINK_MSG_ID_GPS_RAW_INT:{
			if(temp != message.msgid){
				int32_t lat = mavlink_msg_gps_raw_int_get_lat(&message);
				int32_t lon = mavlink_msg_gps_raw_int_get_lon(&message);
				int32_t alt = mavlink_msg_gps_raw_int_get_alt(&message);
				float gps[3] = {(float)lat/10000000, (float)lon/10000000, (float)alt/1000};
				//mavlink_msg_gps_raw_int_decode(&message, &(current_messages.gps_raw_int));
				printf("MAVLINK_MSG_ID_GPS_RAW_INT %f, %f, %f\n", gps[0], gps[1], gps[2]);

				// sprintf(command, "exiftool -GPSLatitude=%.06f -GPSLongitude=%.06f -GPSAltitude=%.02f -overwrite_original %s", 
				// 	gps[0], gps[1], gps[2], "img1.jpg");
				// int error = system(command);
				// printf("comand: %s, error: %d\n", command, error);
				// memset(command, '\0', 150);
			}
			break;
		}
		
		case MAVLINK_MSG_ID_GPS_STATUS:{
			//printf("MAVLINK_MSG_ID_GPS_STATUS\n");
			mavlink_msg_gps_status_decode(&message, &(current_messages.gps_status));
			break;
		}
		
		case MAVLINK_MSG_ID_HEARTBEAT:{
			//printf("MAVLINK_MSG_ID_HEARTBEAT\n");
			mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
			break;
		}

		case MAVLINK_MSG_ID_SYS_STATUS:{
			//printf("MAVLINK_MSG_ID_SYS_STATUS\n");
			mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
			break;
		}

		case MAVLINK_MSG_ID_BATTERY_STATUS:{
			//printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
			mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
			break;
		}

		case MAVLINK_MSG_ID_RADIO_STATUS:{
			//printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
			mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
			break;
		}

		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:{
			//printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
			mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
			break;
		}

		case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:{
			//printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
			mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
			break;
		}

		case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:{
			//printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
			mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
			break;
		}

		case MAVLINK_MSG_ID_HIGHRES_IMU:{
			//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
			mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
			break;
		}

		case MAVLINK_MSG_ID_ATTITUDE:{
			//printf("MAVLINK_MSG_ID_ATTITUDE\n");
			mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
			break;
		}

		default:{
			//printf("Warning, did not handle message id %i\n",message.msgid);
			break;
		}
	}
}

int main(int argc, char **argv){
    char *uart_name = (char*)"/dev/ttyACM0";
    int baudrate = 57600;
	if(argc < 2){
		printf("Fail format. Ex: ./main /dev/ttyACM0\n");
		exit(0);
	}
    Serial_start(argv[1], baudrate);
	uint8_t cp;

	while(1){
		usleep(100000);
		int result = Serial_read(cp);
		if (result > 0){
			message_handle(&cp);
			/*
			msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
			printf("msgId: %d -> ", message.msgid);
			switch (message.msgid){
				case MAVLINK_MSG_ID_HEARTBEAT:{
					printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					break;
				}

				default:{
					printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}
			}*/

		}
	}
    return 0;
}
