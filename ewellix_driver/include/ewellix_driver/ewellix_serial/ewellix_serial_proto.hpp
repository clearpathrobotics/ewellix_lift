/**
 *
 *  \file       ewellix_serial_proto.hpp
 *  \brief      Definitions for Ewellix serial communication
 *  \author     Luis Camero <lcamero@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2024, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */
#ifndef EWELLIX_DRIVER__EWELLIX_SERIAL_PROTO_HPP_
#define EWELLIX_DRIVER__EWELLIX_SERIAL_PROTO_HPP_

/**
 * Ewellix RS232 Interface for SCU control unit
 */

/**
 * Command Data Get
 *   Byte: Value
 *      0: 'R'
 *      1: 'G'
 *      2: DataID[0]
 *      3: DataID[1]
 *   e.g. Get firmware version: R G 01 00
 *
 * Response Data Get
 *   Byte: Value
 *      0: ACK or Error
 *      1: DataCount[0] (Number of following data bytes lower)
 *      2: DataCount[1] (Number of following data bytes upper)
 *      3: DataByte[0]
 *      ...
 *      n: DataByte[DataCount-1]
 */

/**
 * Command Data Transfer
 *   Byte: Value
 *      0: 'R'
 *      1: 'T'
 *      2: DataCount[0] (Number of following data bytes lower)
 *      3: DataCount[1] (Number of following data bytes upper)
 *      4: DataID[0]
 *      5: DataID[1]
 *      6: DataByte[0]
 *      ...
 *      n: DataByte[DataCount-1]
 *
 *  Response Data Transfer
 *    Byte: Value
 *       0: ACK or Error
 */


/**
 * Command Cyclic
 *   Byte: Value
 *      0: 'R'
 *      1: 'C'
 *      2: DataCount[0]
 *      3: DataCount[1]
 *      4: IndexOfCyclicObject
 *      5: CyclicObjectDataByte[0]
 *      ...
 *      6: CyclicObjectDataByte[DataCount]
 * Response Cyclic
 *   Byte: Value
 *      0: ACK or Error
 *      1: DataCount[0]
 *      2: DataCount[1]
 *      3: CyclicObjectDataByte[0]
 *      ...
 *      4: CyclicObjectDataByte[DataCount - 1]
 */

/**
 * Command Execute Function
 *   Byte: Value
 *      0: 'R'
 *      1: 'E'
 *      2: FunctionID
 *      3: ParameterID[0]
 *      4: ParameterID[1]
 *
 * Response Execute Function
 *   Byte: Value
 *      0: ACK or Error
 */

/**
 * Command Stop
 *   Byte: Value
 *      0: 'R'
 *      1: 'S'
 *      2: FunctionID
 *      3: ParameterID[0]
 *
 * Response Stop
 *   Byte: Value
 *      0: ACK or Error
 */

/**
 * Command Open
 *   Byte: Value
 *      0: 'R'
 *      1: 'O'
 *      2: SafetyID[0]
 *
 * Response Open
 *   Byte: Value
 *      0: ACK
 */

/**
 * Command Abort
 *   Byte: Value
 *      0: 'R'
 *      1: 'A'
 *
 * Response Abort
 *   Byte: Value
 *      0: ACK or Error
 */

/**
 * Command Set:
 */
#define COMMAND_REMOTE            0x52 // R
#define COMMAND_DATA_GET          0x47 // G
#define COMMAND_DATA_TRANSFER     0x54 // T
#define COMMAND_CYCLIC            0x43 // C
#define COMMAND_EXECUTE_FUNCTION  0x45 // E
#define COMMAND_STOP              0x53 // S
#define COMMAND_OPEN              0x4F // O
#define COMMAND_ABORT             0x41 // A

/**
 * Communication error and acknowledge codes
 */
#define ACK                   0x06
#define ERROR_CHECKSUM        0x80
#define ERROR_PARAMETER_DATA  0x81
#define ERROR_PARAMETER_COUNT 0x82
#define ERROR_INVALID_COMMAND 0x83
#define ERROR_PERMISSION      0x84

/**
 * Data Fields
 * Remote read-only registers.
 */
#define DATA_FIRMWARE                   0x0001
#define DATA_CONFIGURATION              0x0002
#define DATA_ACTUAL_POSITION_ACTUATOR   0x0011
#define DATA_ACTUAL_STATE_BINARY        0x0020
#define DATA_ACTUAL_STATE_ANALOG        0x0030
#define DATA_ACTUAL_STATE_KEYS          0x0040
#define DATA_NUMBER_CYCLE_IN_ACTUATOR   0x0061
#define DATA_NUMBER_CYCLE_OUT_ACTUATOR  0x0071
#define DATA_NUMBER_ERROR_ACTUATOR      0x0081
#define DATA_NUMBER_TOTAL_OVER_CURRENT  0x008F
#define DATA_CUMULATED_STROKE_ACTUATOR  0x0091
#define DATA_CURRENT_ACTUATOR           0x00A1
#define DATA_MAX_CURRENT_ACTUATOR       0x00B1
#define DATA_MAX_TOTAL_CURRENT          0x00BF
#define DATA_MAX_TEMP_RECTIFIER         0x00C0
#define DATA_OVER_TEMP_RECTIFIER        0x00C1
#define DATA_ERROR_CODE_HISTORY         0x00D0 // to 0X00D5
#define DATA_STATUS_2_ACTUATOR          0x00E1
#define DATA_SPEED_ACTUATOR             0x00F1
#define DATA_BATTERY_MAINS              0x0100
#define DATA_BINARY_OUTPUT_STATUS       0x0110
#define DATA_LED_HS                     0x0120
#define DATA_LED_LB                     0x0130
#define DATA_BUZZER                     0x0140
#define DATA_SENSOR_SUPPLY              0x0150
#define DATA_LOCK_STATUS                0x0162
#define DATA_BATTERY_VOLTAGE            0x0164
#define DATA_LOCKING_BOX_DETECTED       0x0165
#define DATA_USER                       0x0166
#define DATA_STATUS_1_ACTUATOR          0x0171

#define DATA_CONVERSION_FACTOR_ACTUATOR 0x1011

#define DATA_USER_POSITION_ACTUATOR     0x2001

/**
 * Writable Data Fields
 * Stored in volatile register.
 * Initialized after reset with preset values.
 */
#define WRITEABLE_DATA_CYCLIC_OBJECT              0x3001
#define WRITEABLE_DATA_REMOTE_SPEED_FUNCTION      0x3010 // to 301A, F1-10
#define WRITEABLE_DATA_REMOTE_POSITION_ACTUATOR   0x3021
#define EMPTY_SHORT                               0xFFFF
#define EMPTY_BYTE                                0xFF

/**
 * Functions
 */
#define FUNCTION_NO_MOTION        0x00
#define FUNCTION_IN               0x01
#define FUNCTION_OUT              0x02
#define FUNCTION_MEM1             0x03
#define FUNCTION_MEM2             0x04
#define FUNCTION_MEM3             0x05
#define FUNCTION_MEM4             0x06
#define FUNCTION_INTERMEDIATE_IN  0x07
#define FUNCTION_INTERMEDIATE_OUT 0x08
#define FUNCTION_REMOTE           0x09

/**
 * Safety Modes
 */

#define SAFETY_MODE_TERMINATE   0x00
#define SAFETY_MODE_STOP        0x01
#define SAFETY_MODE_REMOTE_STOP 0x02

/**
 * Stop Modes
 */
#define STOP_MODE_FAST 0x00
#define STOP_MODE_SOFT 0x01

/**
 * Actuators
 */
#define ACTUATOR_1   0x00
#define ACTUATOR_2   0x01
#define ACTUATOR_3   0x02
#define ACTUATOR_4   0x03
#define ACTUATOR_5   0x04
#define ACTUATOR_6   0x05
#define ACTUATOR_ALL 0x07

/**
 * Cyclic Object
 */
#define CYCLIC_OBJECT_1 0x00
#define CYCLIC_OBJECT_2 0x01
#define CYCLIC_OBJECT_3 0x02
#define CYCLIC_OBJECT_4 0x03
#define CYCLIC_OBJECT_5 0x04

#endif // EWELLIX_DRIVER__EWELLIX_SERIAL_PROTO_HPP_
