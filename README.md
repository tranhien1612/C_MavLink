# C Mavlink

## CLone project

Clone project: 
```
git clone https://github.com/tranhien1612/C_MavLink.git
```
Unzip ```include``` folder: 
```
cd ./C_MavLink/read_mavlinkData_by_serial/
unzip include.zip
cp -r include ../send_joystickData
```
Tree folder:
```
├── include
│   └── mavlink
│       ├── checksum.h
│       ├── common
│       │   ├── common.h
│       │   ├── mavlink.h
│       │   ├── mavlink_msg_actuator_control_target.h
│       │   ├── mavlink_msg_actuator_output_status.h
│       │   ├── mavlink_msg_adsb_vehicle.h
│       │   ├── ...
│       │   └── version.h
│       ├── mavlink_conversions.h
│       ├── mavlink_get_info.h
│       ├── mavlink_helpers.h
│       ├── mavlink_sha256.h
│       ├── mavlink_types.h
│       ├── minimal
│       │   ├── mavlink.h
│       │   ├── mavlink_msg_heartbeat.h
│       │   ├── mavlink_msg_protocol_version.h
│       │   ├── minimal.h
│       │   ├── testsuite.h
│       │   └── version.h
│       ├── protocol.h
│       └── standard
│           ├── mavlink.h
│           ├── standard.h
│           ├── testsuite.h
│           └── version.h
├── include.zip
├── main.cpp
└── Makefile
```
## ```read_mavlinkData_by_serial```: Read MavLink Data via Serial.
### Setup Hardware

Connect the USB programming cable to your Pixhawk.

If you want to be able to interact with this example in Pixhawk's NuttX shell, you'll need a Telemetry Radio or an FTDI developer's cable. See the Exploration section below for more detail.

Also Note: Using a UART (serial) connection should be preferred over using the USB port for flying systems. The reason being that the driver for the USB port is much more complicated, so the UART is a much more trusted port for flight-critical functions. To learn how this works though the USB port will be fine and instructive.

### Execution

You have to pick a port name, try searching for it with:
```
ls /dev/ttyACM* 
ls /dev/ttyUSB*
```
Run the example executable on the host shell:
```
make
./main /dev/ttyACM0
```

To stop the program, use the key sequence Ctrl-C.
## ```send_joystickData```: Read joystick data and send them to Mavlink via UDP
### Setup Hardware

Connect the USB programming cable to your joystick

### Execution

Run the example executable on the host shell:
```
make
./main
```

To stop the program, use the key sequence Ctrl-C.
