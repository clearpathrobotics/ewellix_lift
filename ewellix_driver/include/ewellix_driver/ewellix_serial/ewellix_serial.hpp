/**
 *
 *  \file       ewellix_serial.hpp
 *  \brief      Ewellix serial communication
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

#ifndef EWELLIX_DRIVER__EWELLIX_SERIAL_HPP_
#define EWELLIX_DRIVER__EWELLIX_SERIAL_HPP_

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include "serial/serial.h"
#include "ewellix_driver/ewellix_serial/ewellix_serial_proto.hpp"

namespace ewellix_driver
{

class EwellixSerial
{
  public:
    struct DataCycle1
    {
      signed int actual_position_a1;
      signed int actual_position_a2;
      unsigned short speed_a1;
      unsigned short speed_a2;
      unsigned char status1_a1;
      unsigned char status1_a2;
    };

    struct DataCycle2
    {
      std::vector<int> actual_positions;
      std::vector<int> remote_positions;
      std::vector<uint16_t> speeds;
      std::vector<uint16_t> currents;
      std::vector<uint8_t> status1;
      std::vector<uint32_t> errors;
    };

    struct Status1
    {
      bool available;
      bool limit_in_out;
      bool switch1;
      bool switch2;
      bool motion;
      bool reached;
      bool out_position;
      bool stroke;
    };

    struct SCUError
    {
      bool fault_ram;
      bool fault_rom;
      bool fault_cpu;
      bool stack_overrun;
      bool sequence_error;
    };

    EwellixSerial();
    EwellixSerial(std::string port, int baud_rate, int timeout);
    ~EwellixSerial();

    bool open();
    void close();

    bool activate();
    bool deactivate();
    bool cycle();
    bool get(uint16_t field, std::vector<uint8_t> &data);
    bool set(uint16_t field, const std::vector<uint8_t> data);
    bool execute(uint8_t actuator, uint8_t function);
    bool stop(uint8_t actuator, uint8_t mode);

    bool cycle1(int a1_position, int a2_position, std::vector<uint8_t> &data);
    bool setCycle1();
    bool getCycle1(std::vector<uint8_t> &data);
    DataCycle1 convertCycle1(const std::vector<uint8_t> data);

    bool cycle2(const std::vector<int> positions, std::vector<uint8_t> &data);
    bool setCycle2();
    bool getCycle2(std::vector<uint8_t> &data);
    DataCycle2 convertCycle2(const std::vector<uint8_t> data);

    bool send(const std::vector<uint8_t> message);
    bool receive(const std::vector<uint8_t> message, std::vector<uint8_t> &response, std::vector<uint8_t> &data);
    bool call(uint8_t command, const std::vector<uint8_t>parameters, std::vector<uint8_t> &response, std::vector<uint8_t> &data);

    bool generateMessage(uint8_t command, const std::vector<uint8_t> parameters, std::vector<uint8_t> &message);
    void appendShort(std::vector<uint8_t> &message, uint16_t data);
    void appendInteger(std::vector<uint8_t> &message, int data);
    void appendVector(std::vector<uint8_t> &message, const std::vector<uint8_t> v);
    void appendIntegerVector(std::vector<uint8_t> &message, const std::vector<int> v);

    uint16_t convertShort(const std::vector<uint8_t> data);
    uint32_t convertInteger(const std::vector<uint8_t> data);
    float convertFloat(const std::vector<uint8_t> data);
    Status1 convertStatus1(const std::vector<uint8_t> data);
    bool checkResponseChecksum(const std::vector<uint8_t> response);
    uint16_t calculateChecksum(const std::vector<uint8_t> message);

    // Execute Motion
    bool executeAllRemote()
    {
      return execute(ACTUATOR_ALL, FUNCTION_REMOTE);
    };

    // Stop Motion
    bool stopAll()
    {
      return stop(ACTUATOR_ALL, STOP_MODE_FAST);
    };

    // Get Actuator Data
    bool getActuatorData(uint16_t field, uint16_t actuator, std::vector<uint8_t> &data)
    {
      return get(field + actuator, data);
    };

    bool getActuatorActualPosition(uint16_t actuator, std::vector<uint8_t> &data)
    {
      return getActuatorData(DATA_ACTUAL_POSITION_ACTUATOR, actuator, data);
    };

    bool getActuatorError(uint16_t actuator, std::vector<uint8_t> &data)
    {
      return getActuatorData(DATA_NUMBER_ERROR_ACTUATOR, actuator, data);
    };

    bool getActuatorCumulatedStroke(uint16_t actuator, std::vector<uint8_t> &data)
    {
      return getActuatorData(DATA_CUMULATED_STROKE_ACTUATOR, actuator, data);
    };

    bool getActuatorCurrent(uint16_t actuator, std::vector<uint8_t> &data)
    {
      return getActuatorData(DATA_CURRENT_ACTUATOR, actuator, data);
    };

    bool getActuatorMaxCurrent(uint16_t actuator, std::vector<uint8_t> &data)
    {
      return getActuatorData(DATA_MAX_CURRENT_ACTUATOR, actuator, data);
    };

    bool getActuatorStatus1(uint16_t actuator, std::vector<uint8_t> &data)
    {
      return getActuatorData(DATA_STATUS_1_ACTUATOR, actuator, data);
    };

    bool getActuatorStatus2(uint16_t actuator, std::vector<uint8_t> &data)
    {
      return getActuatorData(DATA_STATUS_2_ACTUATOR, actuator, data);
    };

    bool getActuatorSpeed(uint16_t actuator, std::vector<uint8_t> &data)
    {
      return getActuatorData(DATA_SPEED_ACTUATOR, actuator, data);
    };

    bool getActuatorRemotePosition(uint16_t actuator, std::vector<uint8_t> &data)
    {
      return getActuatorData(WRITEABLE_DATA_REMOTE_POSITION_ACTUATOR, actuator, data);
    };
    serial::Serial * serial_ = nullptr;

  private:
    uint16_t CRC_TABLE[256] = {
      0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,
      0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
      0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,
      0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
      0x2462,0x3443,0x0420,0x1401,0x64E6,0x74C7,0x44A4,0x5485,
      0xA56A,0xB54B,0x8528,0x9509,0xE5EE,0xF5CF,0xC5AC,0xD58D,
      0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,0x5695,0x46B4,
      0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,0xC7BC,
      0x48C4,0x58E5,0x6886,0x78A7,0x0840,0x1861,0x2802,0x3823,
      0xC9CC,0xD9ED,0xE98E,0xF9AF,0x8948,0x9969,0xA90A,0xB92B,
      0x5AF5,0x4AD4,0x7AB7,0x6A96,0x1A71,0x0A50,0x3A33,0x2A12,
      0xDBFD,0xCBDC,0xFBBF,0xEB9E,0x9B79,0x8B58,0xBB3B,0xAB1A,
      0x6CA6,0x7C87,0x4CE4,0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,
      0xEDAE,0xFD8F,0xCDEC,0xDDCD,0xAD2A,0xBD0B,0x8D68,0x9D49,
      0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,0x2E32,0x1E51,0x0E70,
      0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,0x9F59,0x8F78,
      0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,0xE16F,
      0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
      0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,
      0x02B1,0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,
      0xB5EA,0xA5CB,0x95A8,0x8589,0xF56E,0xE54F,0xD52C,0xC50D,
      0x34E2,0x24C3,0x14A0,0x0481,0x7466,0x6447,0x5424,0x4405,
      0xA7DB,0xB7FA,0x8799,0x97B8,0xE75F,0xF77E,0xC71D,0xD73C,
      0x26D3,0x36F2,0x0691,0x16B0,0x6657,0x7676,0x4615,0x5634,
      0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,0xB98A,0xA9AB,
      0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,0x28A3,
      0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BF9,0x9BD8,0xABBB,0xBB9A,
      0x4A75,0x5A54,0x6A37,0x7A16,0x0AF1,0x1AD0,0x2AB3,0x3A92,
      0xFD2E,0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,
      0x7C26,0x6C07,0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CC1,
      0xEF1F,0xFF3E,0xCF5D,0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,
      0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3EB2,0x0ED1,0x1EF0
    };

    std::string port_;
    int baud_rate_;
    int timeout_;


};

}

#endif // EWELLIX_DRIVER__EWELLIX_SERIAL_HPP_
