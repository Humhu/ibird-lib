/**
* Copyright (c) 2010-2012, Regents of the University of California
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright notice,
*   this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
* - Neither the name of the University of California, Berkeley nor the names
*   of its contributors may be used to endorse or promote products derived
*   from this software without specific prior written permission.
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
*
*
* Command Codes
*
* by Stan Baek
*
* v.0.5-hhu
*
* Revisions:
*   Stan Baek           2010-07-10      Initial implementation
*   Humphrey Hu         2012-04-11      Switched to standard format
*
 *
* Notes:
*
*/
#ifndef __CMD_CONST_H
#define __CMD_CONST_H

#define MAX_CMD_FUNC_SIZE               (0x60) // 0x00 - 0x3F

// CMD values of 0x00(0) - 0x3F(127) are defined here
// Values 0x00 through 0x10 are reserved for bootloader
// Use will interfere with bootloader!
#define CMD_NACK                        (0x00)      // Reserved - Do not use!
#define CMD_ACK                         (0x01)      // Reserved - Do not use!
#define CMD_READ_PM                     (0x02)      // Reserved - Do not use!
#define CMD_WRITE_PM                    (0x03)      // Reserved - Do not use!
#define CMD_WRITE_CM                    (0x07)      // Reserved - Do not use!
#define CMD_RESET                       (0x08)      // Reserved - Do not use!
#define CMD_READ_ID                     (0x09)      // Reserved - Do not use!
#define CMD_GET_VERSION                 (0x0A)      // Reserved - Do not use!
#define COMMAND_READ_GOTO               (0x10)      // Reserved - Do not use!

#define CMD_PING                        (0x1E)      // Do nothing
#define CMD_ECHO                        (0x1F)      // send back the received packet

#define CMD_SET_REGULATOR_MODE          (0x21)      // Set the regulator mode
#define CMD_SET_REGULATOR_REF           (0x22)      // Set the regulator references
#define CMD_SET_REGULATOR_PID           (0x23)      // Set regulator PID parameters
#define CMD_SET_REGULATOR_RATE_FILTER   (0x24)      // Set yaw filter coefficients
#define CMD_SET_RC_VALUES               (0x25)      // Set remote control values in RC mode
#define CMD_SET_RATE_MODE               (0x26)      // Set slew enable mode
#define CMD_SET_RATE_SLEW               (0x27)      // Set position slew rate

// for IMU
#define CMD_RECORD_SENSOR_DUMP          (0x28)      // Begin saving IMU data to flash
#define CMD_GET_MEM_CONTENTS            (0x29)      // Transmit data in flash
#define CMD_RUN_GYRO_CALIB              (0x2A)      // Begin gyroscope calibration procedure
#define CMD_GET_GYRO_CALIB_PARAM        (0x2B)      // get gyroscope calibration offset

#define CMD_SET_ESTIMATE_RUNNING        (0x2C)      // Begin attitude estimation
#define CMD_REQUEST_TELEMETRY           (0x2D)      // Request state update
#define CMD_RESPONSE_TELEMETRY          (0x2E)      // Response to state update request
#define CMD_RECORD_TELEMETRY            (0x2F)      // Save telemetry to flash

#define CMD_ADDRESS_REQUEST             (0x30)      // Request an address from network coordinator
#define CMD_ADDRESS_OFFER               (0x31)      // Offer an address to a client
#define CMD_ADDRESS_ACCEPT              (0x32)      // Accept an offer from coordinator

// TODO: Are these the same? Consolidate into one?
#define CMD_DIR_UPDATE_REQUEST          (0x34)      // Request a directory update
#define CMD_DIR_UPDATE_RESPONSE         (0x35)      // A directory update
#define CMD_DIR_DUMP_REQUEST            (0x36)      // Request a full directory update
#define CMD_DIR_DUMP_RESPONSE           (0x37)      // A full directory update

#define CMD_CLOCK_UPDATE_REQUEST        (0x38)      // Request for clock update process
#define CMD_CLOCK_UPDATE_RESPONSE       (0x39)      // Response to clock update request

#define CMD_RAW_FRAME_REQUEST           (0x40)      // Request for raw frame transmission
#define CMD_RAW_FRAME_RESPONSE          (0x41)      // Response to request for raw frame
#define CMD_CENTROID_REPORT             (0x42)      // Report of centroid info
#define CMD_SET_BACKGROUND_FRAME        (0x43)      // Capture and set background frame

#define CMD_CAM_PARAM_REQUEST           (0x48)      // Request camera parameters
#define CMD_CAM_PARAM_RESPONSE          (0x49)      // Response to camera parameter request
#define CMD_SET_HP                      (0x4A)      // Set CV high pass on/off

#define CMD_ZERO_ESTIMATE               (0x4C)      // Zero attitude estimate
#define CMD_REQUEST_ATTITUDE            (0x50)      // Request attitude
#define CMD_RESPONSE_ATTITUDE           (0x51)      // Response to request for attitude

#define CMD_SET_TELEM_SUBSAMPLE         (0x52)      // Telemetry subsampling divider
#define CMD_SET_SLEW_LIMIT              (0x53)      // Reference slew limiting

// CMD values of 0x80(128) - 0xEF(239) are reserved.
// CMD values of 0xF0(240) - 0xFF(255) are reserved for future use

#endif  // __CMD_CONST_H

