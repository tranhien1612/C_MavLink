/* send function: manual for 4 axis, override button*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <time.h>
#include "mavlink/common/mavlink.h"
// Use this under path mavlink.h in the nanopi board. (MAJ joystick))
// #include </root/joystick/mavlink/generated/include/mavlink/v2.0/common/mavlink.h>
#define JS_DEV "/dev/input/js0"
// #define _POSIX_C_SOURCE 199309L

void receive_some(int socket_fd, struct sockaddr_in* src_addr, socklen_t* src_addr_len, bool* src_addr_set);
void handle_heartbeat(const mavlink_message_t* message);

void send_some(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void send_heartbeat(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void send_mavlink_command_long(int sock, const struct sockaddr_in* target_addr, socklen_t src_addr_len, uint8_t system_id, uint8_t component_id, uint8_t target_system, uint8_t target_component, uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7);
void set_mode_rtl(int socket_fd, const struct sockaddr_in* target_addr, socklen_t src_addr_len);
void send_mavlink_command_manual(int sock, const struct sockaddr_in* target_addr, socklen_t src_addr_len, float param1, float param2, float param3, float param4, float param5);
void send_mavlink_command_override(int sock, const struct sockaddr_in* target_addr, socklen_t src_addr_len, float param1, float param2, float param3, float param4, float param5, float param6, float param7, float param8);
int map_range(float x, int in_min, int in_max, int out_min, int out_max);
int binary_to_decimal(int bit1, int bit2, int bit3);
int buttonValue_toRC(int numberBut, float in);

int main(int argc, char* argv[])
{   
    int fd = open(JS_DEV, O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        perror("Failed to open joystick");
        return 1;
    }
    // Get the number of buttons
    __u8 num_buttons;
    if (ioctl(fd, JSIOCGBUTTONS, &num_buttons) < 0) {
        perror("Failed to get number of buttons");
        close(fd);
        return 1;
    }
    printf("Joystick has %d buttons\n", num_buttons);

    // Get the number of axes
    __u8 num_axes;
    if (ioctl(fd, JSIOCGAXES, &num_axes) < 0) {
        perror("Failed to get number of axes");
        close(fd);
        return 1;
    }
    printf("Joystick has %d axes\n", num_axes);

    // Allocate memory to store button and axis states
    unsigned char *button_states = (unsigned char *)calloc(num_buttons, sizeof(unsigned char));
    __s16 *axis_states = (__s16 *)calloc(num_axes, sizeof(__s16));

    if (!button_states || !axis_states) {
        perror("Memory allocation failed");
        close(fd);
        return 1;
    }
    struct js_event js;
    
    const int socket_fd = socket(PF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0) {
        // printf("socket error: %s\n", strerror(errno));
        return -1;
    }
    // Bind to port
    struct sockaddr_in addr = {};
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    inet_pton(AF_INET, "0.0.0.0", &(addr.sin_addr)); // listen on all network interfaces
    addr.sin_port = htons(14555); // default port on the ground

    if (bind(socket_fd, (struct sockaddr*)(&addr), sizeof(addr)) != 0) {
        // printf("bind error: %s\n", strerror(errno));
        return -2;
    }

    struct sockaddr_in src_addr = {};
    socklen_t src_addr_len = sizeof(src_addr);
    bool src_addr_set = false;
    bool quit = false;
    static struct timespec last_manual_control_time = {0};
    struct timespec current_manual_control_time;
    bool trigger_first = false;
    int value =0;

    uint16_t manual_channels[4] = {0, 0, 495, 0};
    uint16_t override_channels[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
    //                                RC5, RC6,  RC7,  RC8,  RC9,  RC10, RC11, RC12
    /*
        rc2-a2-chan3: throttle; rc3-a3-chan4: yaw; rc1-a1-chan1: pitch, rc0-a0-chan2: roll, a4: gimbal
    */
    
    receive_some(socket_fd, &src_addr, &src_addr_len, &src_addr_set);
    send_mavlink_command_override(socket_fd, &src_addr, src_addr_len, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000);
    while (!quit) {
        receive_some(socket_fd, &src_addr, &src_addr_len, &src_addr_set);
        if (read(fd, &js, sizeof(struct js_event)) >= 0) {
            switch (js.type & ~JS_EVENT_INIT) {
                case JS_EVENT_BUTTON:
                    // printf("button is changed: %d \n", js.number);
                    button_states[js.number] = js.value;
                    override_channels[js.number+2] = map_range(js.value, 0, 1, 1000, 2000); //RC7
                    break;
                case JS_EVENT_AXIS:
                    axis_states[js.number] = js.value;
                    if (js.number==2) {
                        manual_channels[js.number] = map_range(axis_states[js.number], -32767, 32767, 1000, 0);
                    } else if (js.number==4 || js.number==5) {
                        override_channels[js.number-4] = map_range(axis_states[js.number], -32767, 32767, 1000, 2000); //offset is 4 to mapping with override is index 0
                    }
                    else {
                        manual_channels[js.number] = map_range(axis_states[js.number], -32767, 32767, -1000, 1000);
                    }
                    break;
            }
        }
        // timer: 50hz
        clock_gettime(CLOCK_MONOTONIC, &current_manual_control_time);
        if (current_manual_control_time.tv_sec > last_manual_control_time.tv_sec ||
                (current_manual_control_time.tv_sec == last_manual_control_time.tv_sec && current_manual_control_time.tv_nsec - last_manual_control_time.tv_nsec >= 20000000)) {
            send_mavlink_command_override(socket_fd, &src_addr, src_addr_len, override_channels[0], override_channels[1],override_channels[2], override_channels[3], override_channels[4], override_channels[5], override_channels[6], override_channels[7]);
            send_mavlink_command_manual(socket_fd, &src_addr, src_addr_len, manual_channels[1], manual_channels[0], manual_channels[2], manual_channels[3], 0);
            last_manual_control_time = current_manual_control_time;
        }
    }
    // Clean up
    free(button_states);
    free(axis_states);
    close(fd);
    close(socket_fd);
    return 0;
}

int map_range(float x, int in_min, int in_max, int out_min, int out_max) {
    if (x > in_max) {
        return out_max;
    } 
    if (x < in_min) {
        return out_min;
    } else {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
int binary_to_decimal(int bit1, int bit2, int bit3) {
    return bit1*4 + bit2*2 + bit3;
}
int buttonValue_toRC(int numberBut, float in) {
    int out = 0;
    if (numberBut == 0 || numberBut == 8 || numberBut == 4) {
        out = map_range(in, 0, 1, 1000, 2000);
    } 
    else if (numberBut == 1 || numberBut == 5) {
        in = (in==2) ? 2.5 : in;
        out = map_range(in, 1,4,1000, 2000);
    }
    return out;
}
void send_mavlink_command_override(int sock, const struct sockaddr_in* target_addr, socklen_t src_addr_len, float param1, float param2, float param3, float param4, float param5, float param6, \
                                    float param7, float param8) {
    mavlink_message_t msg;
    mavlink_msg_rc_channels_override_pack(
        255, // System ID
        0, // Component ID
        &msg,
        1, // Target system ID
        1, // Target component ID
        0, // RC channel 1
        0, // RC channel 2
        0, // RC channel 3
        0, // RC channel 4
        param1, // RC channel 5
        param2, // RC channel 6
        param3, // RC channel 7
        param4, // RC channel 8
        param5,
        param6,
        param7,
        param8,
        0,0,0,0,0,0
    );
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buffer, &msg);
    if (sendto(sock, buffer, len, 0, (const struct sockaddr*)target_addr, src_addr_len) == -1) {
        printf("Failed to send OVERRIDE message. \n");
    } 
}
void send_mavlink_command_manual(int sock, const struct sockaddr_in* target_addr, socklen_t src_addr_len, float param1, float param2, float param3, float param4, float param5) {
    mavlink_message_t msg;
    mavlink_msg_manual_control_pack (
        255,
        0,
        &msg,
        1,
        param1, // RC channel 2
        param2, // RC channel 3
        param3, // RC channel 4
        param4, // RC channel 5
        param5, // RC channel 6
        0, // RC channel 7
        0,0,0,0,0,0,0,0,0
    );
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buffer, &msg);
    if (sendto(sock, buffer, len, 0, (const struct sockaddr*)target_addr, src_addr_len) == -1) {
        printf("Failed to send MANUAL message. \n");
    } 
}
void set_mode_rtl(int socket_fd, const struct sockaddr_in* target_addr, socklen_t src_addr_len) {
    send_mavlink_command_long(socket_fd, target_addr, src_addr_len, 255, 0, 1, 1, MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0, 0, 0, 0, 0);
}
void send_mavlink_command_long(int sock, const struct sockaddr_in* target_addr, socklen_t src_addr_len, uint8_t system_id, uint8_t component_id, uint8_t target_system, uint8_t target_component, uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7) {
    mavlink_command_long_t cmd;
    cmd.target_system = target_system;
    cmd.target_component = target_component;
    cmd.command = command;
    cmd.confirmation = 0;
    cmd.param1 = param1;
    cmd.param2 = param2;
    cmd.param3 = param3;
    cmd.param4 = param4;
    cmd.param5 = param5;
    cmd.param6 = param6;
    cmd.param7 = param7;

    mavlink_message_t msg;
    mavlink_msg_command_long_encode(system_id, component_id, &msg, &cmd);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buffer, &msg);
    if (sendto(sock, buffer, len, 0, (const struct sockaddr*)target_addr, src_addr_len) == -1) {
        printf("Failed to send COMMAND_LONG message. \n");
    } 
    // else {
    //     printf("COMMAND_LONG message sent successfully.\n");
    // }
}
void receive_some(int socket_fd, struct sockaddr_in* src_addr, socklen_t* src_addr_len, bool* src_addr_set)
{
    // We just receive one UDP datagram and then return again.
    char buffer[2048]; // enough for MTU 1500 bytes

    const int ret = recvfrom(
            socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)(src_addr), src_addr_len);

    if (ret < 0) {
        // printf("recvfrom error: %s\n", strerror(errno));
    } else if (ret == 0) {
        // peer has done an orderly shutdown
        return;
    }

    *src_addr_set = true;

    mavlink_message_t message;
    mavlink_status_t status;
    for (int i = 0; i < ret; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status) == 1) {

            // printf(
            //     "Received message %d from %d/%d\n",
            //     message.msgid, message.sysid, message.compid);

            switch (message.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                handle_heartbeat(&message);
                break;
            }
        }
    }
}
void handle_heartbeat(const mavlink_message_t* message)
{
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(message, &heartbeat);

    // printf("Got heartbeat from ");
    switch (heartbeat.autopilot) {
        case MAV_AUTOPILOT_GENERIC:
            // printf("generic");
            break;
        case MAV_AUTOPILOT_ARDUPILOTMEGA:
            // printf("ArduPilot");
            break;
        case MAV_AUTOPILOT_PX4:
            // printf("PX4");
            break;
        default:
            // printf("other");
            break;
    }
    // printf(" autopilot\n");
}
void send_some(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    // Whenever a second has passed, we send a heartbeat.
    static time_t last_time = 0;
    time_t current_time = time(NULL);
    if (current_time - last_time >= 1) {
        send_heartbeat(socket_fd, src_addr, src_addr_len);
        last_time = current_time;
    }
}
void send_heartbeat(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    mavlink_message_t message;

    const uint8_t system_id = 42;
    const uint8_t base_mode = 0;
    const uint8_t custom_mode = 0;
    mavlink_msg_heartbeat_pack_chan(
        system_id,
        MAV_COMP_ID_PERIPHERAL,
        MAVLINK_COMM_0,
        &message,
        MAV_TYPE_GENERIC,
        MAV_AUTOPILOT_GENERIC,
        base_mode,
        custom_mode,
        MAV_STATE_STANDBY);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &message);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        // printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("Sent heartbeat\n");
    }
}
