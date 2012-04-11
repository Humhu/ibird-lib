/******************************************************************************
* Name: cmd_const.h
* Desc: Commonly used command definitions are defined here.
* Date: 2010-07-10
* Author: stanbaek
******************************************************************************/
#ifndef __CMD_CONST_H
#define __CMD_CONST_H

#define MAX_CMD_FUNC_SIZE           0x40 // 0x00 - 0x3F

// CMD values of 0x00(0) - 0x3F(127) are defined here
// Values 0x00 through 0x10 are reserved for bootloader
// Use will interfere with bootloader!
#define CMD_NACK                    0x00        // Reserved - Do not use!
#define CMD_ACK                     0x01        // Reserved - Do not use!
#define CMD_READ_PM                 0x02        // Reserved - Do not use!
#define CMD_WRITE_PM                0x03        // Reserved - Do not use!
#define CMD_WRITE_CM                0x07        // Reserved - Do not use!
#define CMD_RESET                   0x08        // Reserved - Do not use!
#define CMD_READ_ID                 0x09        // Reserved - Do not use!
#define CMD_GET_VERSION             0x0A        // Reserved - Do not use!
#define COMMAND_READ_GOTO           0x10        // Reserved - Do not use!

#define CMD_SET_BEHAVIOR_RUNNING    0x0C        // Toggle the behavior state machine
#define CMD_SET_REGULATOR_STATE     0x11        // Set the regulator module running

#define CMD_SET_REF_PITCH           0x13        // Set the pitch reference
#define CMD_SET_REF_YAW             0x14        // Set the yaw reference

#define CMD_SET_PID_YAW             0x16        // Set the yaw PID parameters
#define CMD_SET_PID_PITCH           0x18        // Set the pitch PID parameters
#define CMD_SET_RC_VALUES           0x1D        // Set remote control values in RC mode

#define CMD_ECHO                    0x1F        // send back the received packet

// for IMU
#define CMD_RECORD_SENSOR_DUMP      0x22        // Begin saving IMU data to flash
#define CMD_GET_MEM_CONTENTS        0x23        // Transmit data in flash
#define CMD_RUN_GYRO_CALIB          0x24        // Begin gyroscope calibration procedure
#define CMD_GET_GYRO_CALIB_PARAM    0x25        // get gyroscope calibration offset

#define CMD_SET_ESTIMATE_RUNNING    0x27
#define CMD_REQUEST_TELEMETRY       (0x28)      // Request state update
#define CMD_RESPONSE_TELEMETRY      (0x29)      // 
#define CMD_RECORD_TELEMETRY        (0x30)      // Save telemetry to flash

#define CMD_ADDRESS_REQUEST         (0x2D)        // Request an address from network coordinator
#define CMD_ADDRESS_OFFER           (0x2E)        // Offer an address to a client
#define CMD_ADDRESS_ACCEPT          (0x2F)        // Accept an offer from coordinator

#define CMD_DIRECTORY_UPDATE        (0x30)        // Update list of other active addresses

#define CMD_REQUEST_CLOCK_UPDATE    (0x32)        // Frame clock sync
#define CMD_RESPONSE_CLOCK_UPDATE   (0x33)

#define CMD_SET_YAW_RATE_FILTER     (0x34)        // Set yaw filter coefficients
#define CMD_SET_PITCH_RATE_FILTER   (0x35)        // Set pitch filter coefficients

#define CMD_REQUEST_RAW_FRAME       (0x37)
#define CMD_RESPONSE_RAW_FRAME      (0x38)
#define CMD_CENTROID_REPORT         (0x39)
#define CMD_SET_BACKGROUND_FRAME    (0x3A)

#define CMD_BASE_ECHO               (0x3F)        // send back the received packet

// CMD values of 0x80(128) - 0xEF(239) are reserved.
// CMD values of 0xF0(240) - 0xFF(255) are reserved for future use

#endif  // __CMD_CONST_H

