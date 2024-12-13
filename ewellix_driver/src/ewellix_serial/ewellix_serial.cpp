/**
Software License Agreement (BSD)

\file      ewellix_serial.cpp
\authors   Luis Camero <lcamero@clearpathrobotics.com>
\copyright Copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of Clearpath Robotics nor the names of its contributors
  may be used to endorse or promote products derived from this software
  without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include "ewellix_driver/ewellix_serial/ewellix_serial.hpp"

namespace ewellix_driver
{

EwellixSerial::EwellixSerial():
  port_("/ttyUSB0"),
  baud_rate_(38400),
  timeout_(1000)
{

}

EwellixSerial::EwellixSerial(std::string port, int baud_rate, int timeout):
  port_(port),
  baud_rate_(baud_rate),
  timeout_(timeout)
{

}

EwellixSerial::~EwellixSerial()
{

}


/**
 * Open the serial communication.
 *
 * @return true if successful communication.
 */
bool
EwellixSerial::open()
{
  this->close();

  serial_ = new serial::Serial();

  serial_->setPort(port_);
  serial_->setBaudrate(baud_rate_);
  serial::Timeout serial_timeout_ = serial::Timeout::simpleTimeout(timeout_);
  serial_->setTimeout(serial_timeout_);

  if(!serial_->isOpen())
  {
    try
    {
      serial_->open();
      return true;
    }
    catch (serial::IOException e)
    {
      std::cout << __PRETTY_FUNCTION__ << ": IOException: " << e.what() << std::endl;
      return false;
    }
  }
  return false;
}

/**
 * Close serial communication.
 */
void
EwellixSerial::close()
{
  if(serial_ != nullptr && serial_->isOpen())
  {
    serial_->close();
    delete serial_;
    serial_ = nullptr;
  }
}


/**
 * Activate remote function.
 *
 * @return true if successful activation.
 */
bool
EwellixSerial::activate()
{
  std::vector<uint8_t> parameters = {SAFETY_MODE_STOP};
  std::vector<uint8_t> response;
  std::vector<uint8_t> data;

  return call(COMMAND_OPEN, parameters, response, data);
}

/**
 * Deactivate remote function
 *
 * @return true if successful deactivation.
 */
bool
EwellixSerial::deactivate()
{
  std::vector<uint8_t> parameters = {};
  std::vector<uint8_t> response;
  std::vector<uint8_t> data;

  return call(COMMAND_ABORT, parameters, response, data);
}

/**
 * Cylic message to maintain remote function.
 *
 * @return true if successful
 */
bool
EwellixSerial::cycle()
{
  std::vector<uint8_t> parameters = {0x01, 0x00, 0xff};
  std::vector<uint8_t> response;
  std::vector<uint8_t> data;

  return call(COMMAND_CYCLIC, parameters, response, data);
}

bool
EwellixSerial::setCycle1()
{
  std::vector<uint8_t> data;

  // Write: Remote Position Actuator1
  this->appendShort(data, WRITEABLE_DATA_REMOTE_POSITION_ACTUATOR + ACTUATOR_1);
  // Write: Remote Position Actuator2
  this->appendShort(data, WRITEABLE_DATA_REMOTE_POSITION_ACTUATOR + ACTUATOR_2);
  // Padding
  this->appendShort(data, EMPTY_SHORT);
  this->appendShort(data, EMPTY_SHORT);
  this->appendShort(data, EMPTY_SHORT);
  this->appendShort(data, EMPTY_SHORT);
  // Read: Actual Position Actuator1
  this->appendShort(data, DATA_ACTUAL_POSITION_ACTUATOR + ACTUATOR_1);
  // Read: Actual Position Actuator2
  this->appendShort(data, DATA_ACTUAL_POSITION_ACTUATOR + ACTUATOR_2);
  // Read: Speed Actuator1
  this->appendShort(data, DATA_SPEED_ACTUATOR + ACTUATOR_1);
  // Read: Speed Actuator2
  this->appendShort(data, DATA_SPEED_ACTUATOR + ACTUATOR_2);
  // Read: Status1 Actuator1
  this->appendShort(data, DATA_STATUS_1_ACTUATOR + ACTUATOR_1);
  // Read: Status1 Actuator2
  this->appendShort(data, DATA_STATUS_1_ACTUATOR + ACTUATOR_2);

  return this->set(WRITEABLE_DATA_CYCLIC_OBJECT + CYCLIC_OBJECT_1, data);
}

bool
EwellixSerial::getCycle1(std::vector<uint8_t> &data)
{
  return this->get(WRITEABLE_DATA_CYCLIC_OBJECT + CYCLIC_OBJECT_1, data);
}

bool
EwellixSerial::cycle1(int a1_position, int a2_position, std::vector<uint8_t> &data)
{
  data.clear();
  std::vector<uint8_t> parameters;
  std::vector<uint8_t> response;

  std::vector<uint8_t> write_data = {CYCLIC_OBJECT_1};
  this->appendInteger(write_data, a1_position);
  this->appendInteger(write_data, a2_position);

  uint16_t size = write_data.size();

  this->appendShort(parameters, size);
  this->appendVector(parameters, write_data);

  return call(COMMAND_CYCLIC, parameters, response, data);
}

/**
 * Get Data
 */
bool
EwellixSerial::get(uint16_t field, std::vector<uint8_t> &data)
{
  data.clear();
  std::vector<uint8_t> parameters;
  std::vector<uint8_t> response;

  this->appendShort(parameters, static_cast<uint16_t>(field));

  return call(COMMAND_DATA_GET, parameters, response, data);
}

/**
 * Set Data
 */
bool
EwellixSerial::set(uint16_t field, const std::vector<uint8_t> data)
{
  std::vector<uint8_t> empty;
  std::vector<uint8_t> parameters;
  std::vector<uint8_t> response;

  uint16_t size = 2 + data.size();

  this->appendShort(parameters, size);
  this->appendShort(parameters, field);
  this->appendVector(parameters, data);

  return call(COMMAND_DATA_TRANSFER, parameters, response, empty);
}

/**
 * Execute command
 */
bool
EwellixSerial::execute(uint8_t motor, uint8_t function)
{
  std::vector<uint8_t> empty;
  std::vector<uint8_t> parameters;
  std::vector<uint8_t> response;

  parameters.push_back(motor);
  parameters.push_back(function);
  parameters.push_back(EMPTY_BYTE);

  return call(COMMAND_EXECUTE_FUNCTION, parameters, response, empty);
}

/**
 * Stop command
 */
bool
EwellixSerial::stop(uint8_t motor, uint8_t mode)
{
  std::vector<uint8_t> empty;
  std::vector<uint8_t> parameters;
  std::vector<uint8_t> response;

  parameters.push_back(motor);
  parameters.push_back(mode);

  return call(COMMAND_STOP, parameters, response, empty);
}

/**
 * Check Response Checksum.
 *
 * @return true if computed checksum matches response checksum.
 */
bool
EwellixSerial::checkResponseChecksum(const std::vector<uint8_t> response)
{
  // Extract CRC from Response
  uint8_t crc_lsb = *(response.end() - 2);
  uint8_t crc_msb = *(response.end() - 1);
  uint16_t response_crc = (crc_msb << 8) | crc_lsb;
  // Create Message without CRC from Response
  std::vector<uint8_t> response_no_crc;
  response_no_crc.insert(response_no_crc.end(), response.begin(), response.end() - 2);
  // Compute CRC
  uint16_t crc = this->calculateChecksum(response_no_crc);

  return(crc == response_crc);
}

/**
 * Compute Checksum
 * Cyclic redundancy check.
 *
 * @return checksum
 */
uint16_t
EwellixSerial::calculateChecksum(const std::vector<uint8_t> message)
{
  uint16_t crc = 0;
  for (auto & byte : message)
  {
    crc = static_cast<uint16_t>(
      CRC_TABLE[((crc >> 8) ^ byte) & 0xFF] ^ (crc << 8));
  }
  return crc;
}

/**
 * Send message
 *
 * @return true if message was sent successfully.
 */
bool
EwellixSerial::send(const std::vector<uint8_t> message)
{
  // Lock

  // Check serial status
  if(serial_->isOpen())
  {
    try
    {
      serial_->write(message);
      serial_->flush();
    }
    catch (serial::IOException e)
    {
      return false;
    }
  }
  else
  {
    return false;
  }
  return true;
}

/**
 * Receive message
 *
 * @return true if message received without errors.
 */
bool
EwellixSerial::receive(const std::vector<uint8_t> message, std::vector<uint8_t> &response, std::vector<uint8_t> &data)
{
  bool receiving = true;
  bool success = true;
  uint8_t command = message[1];
  uint16_t count = 0;
  uint16_t crc_count = 2;
  uint16_t data_count = 0;
  // Lock

  while(receiving)
  {
    // Check if data is available
    if(serial_->available())
    {
      // Receive byte
      std::vector<uint8_t> buffer;
      serial_->read(buffer, 1);
      uint8_t byte = buffer[0];

      // Parse byte
      switch(count)
      {
        case 0:
          // Byte 0: Always 'R'
          if(byte != COMMAND_REMOTE)
          {
            break;
          }
          response.push_back(byte);
          count++;
          break;
        case 1:
          // Byte 1: Command Type
          if(byte != command)
          {
            // Error!
            return false;
          }
          // Special Case Cyclic without Data
          if(byte == COMMAND_CYCLIC && message[4] == EMPTY_BYTE)
          {
            command = COMMAND_REMOTE;
          }
          response.push_back(byte);
          count++;
          break;
        case 2:
          // Byte 2: Acknowledge or Error
          success &= byte == ACK;
          receiving = success;
          response.push_back(byte);
          count++;
          break;
        default:
          switch(command)
          {
            case COMMAND_DATA_GET:
              /*
               * Response Data Get
               *   Byte: Value
               *      0: ACK or Error
               *      1: DataCount[0] (Number of following data bytes lower)
               *      2: DataCount[1] (Number of following data bytes upper)
               *      3: DataByte[0]
               *      ...
               *      n: DataByte[DataCount-1]
               */
            case COMMAND_CYCLIC:
              /* Response Cyclic:
               *   Byte: Value
               *      0: ACK or Error
               *      1: DataCount[0]
               *      2: DataCount[1]
               *      3: CyclicObjectDataByte[0]
               *      ...
               *      4: CyclicObjectDataByte[DataCount - 1]
               */
              switch(count)
              {
                case 3:
                  // Byte 3: Data Count Lower Byte
                  data_count += byte;
                  response.push_back(byte);
                  count++;
                  break;
                case 4:
                  // Byte 4: Data Count Upper Byte
                  data_count += byte << 8;
                  response.push_back(byte);
                  count++;
                  break;
                default:
                  // Byte 5+: Read Data
                  if (data_count > 0)
                  {
                    data.push_back(byte);
                    data_count--;
                    response.push_back(byte);
                    count++;
                    break;
                  }
                  // Read Checksum
                  crc_count--;
                  response.push_back(byte);
                  count++;
                  // Done
                  receiving = crc_count > 0;
                  break;
              }
              break;
            case COMMAND_DATA_TRANSFER:
            case COMMAND_EXECUTE_FUNCTION:
            case COMMAND_STOP:
            case COMMAND_OPEN:
            case COMMAND_ABORT:
            default:
              // Last 2 Bytes: Read Checksum
              crc_count--;
              response.push_back(byte);
              count++;
              // Done
              receiving = crc_count > 0;
              break;
          }
          break;
      }
    }
    else
    {
      // Wait
    }
  }
  // Check Checksum from Response
  success &= this->checkResponseChecksum(response);

  if(!success)
  {
    std::cout << __PRETTY_FUNCTION__ << ": Message: ";
    for (auto byte : message)
    {
      std::cout << "  " << std::to_string(byte) << "  ";
    }
    std::cout << std::endl;
    std::cout << __PRETTY_FUNCTION__ << ": Response " << success << ": ";
    for (auto byte : response)
    {
      std::cout << "  " << std::to_string(byte) << "  ";
    }
    std::cout << std::endl;
  }
  return success;
}

/**
 * Append short to byte vector
 */
void
EwellixSerial::appendShort(std::vector<uint8_t> &message, uint16_t data)
{
  message.push_back(static_cast<uint8_t> (data & 0x00FF));
  message.push_back(static_cast<uint8_t> (data >> 8));
}

/**
 * Append integer to byte vector
 */
void
EwellixSerial::appendInteger(std::vector<uint8_t> &message, int data)
{
  message.push_back(static_cast<uint8_t> (data & 0x00FF));
  message.push_back(static_cast<uint8_t> ((data >> 8) & 0x00FF));
  message.push_back(static_cast<uint8_t> ((data >> 16) & 0x00FF));
  message.push_back(static_cast<uint8_t> ((data >> 24) & 0x00FF));
}

/**
 * Append vector to byte vector
 */
void
EwellixSerial::appendVector(std::vector<uint8_t> &message, const std::vector<uint8_t> v)
{
  message.insert(message.end(), v.begin(), v.end());
}

/**
 * Create message
 */
bool
EwellixSerial::generateMessage(uint8_t command, const std::vector<uint8_t> parameters, std::vector<uint8_t> &message)
{
  // Clear
  message.clear();

  // Add Remote Command
  message.push_back(COMMAND_REMOTE);
  message.push_back(command);

  // Data count
  uint8_t data_count;

  // Check required parameters
  switch (command)
  {
  case COMMAND_DATA_GET:
    /**
     * Command Data Get
     *   Byte: Value
     *      0: 'R'
     *      1: 'G'
     *      2: DataID[0]
     *      3: DataID[1]
     *   e.g. Get firmware version: R G 01 00
     */
    // Check message length
    if(parameters.size() != 2)
    {
      std::cout << __PRETTY_FUNCTION__ << ": RG command must have 2 parameter bytes. Given " << std::to_string(parameters.size()) << "." << std::endl;
      return false;
    }
    break;
  case COMMAND_DATA_TRANSFER:
    /**
     * Command Data Transfer
     *   Byte: Value
     *      0: 'R'
     *      1: 'T'
     *      2: DataCount[0] (Number of data bytes lower)
     *      3: DataCount[1] (Number of data bytes upper)
     *      4: DataID[0]
     *      5: DataID[1]
     *      6: DataByte[0]
     *      ...
     *      n: DataByte[DataCount-1]
     */
    // Check message length
  case COMMAND_CYCLIC:
    /**
     * Command Cyclic
     *    Byte: Value
     *       0: 'R'
     *       1: 'C'
     *       2: DataCount[0]
     *       3: DataCount[1]
     *       4: IndexOfCyclicObject
     *       5: CyclicObjectDataByte[0]
     *       ...
     *       6: CyclicObjectDataByte[DataCount - 1]
     */
    if(parameters.size() < 3)
    {
      std::cout << __PRETTY_FUNCTION__ << ": RT/RC data command must have at least 3 parameter bytes: 2 to specify data length and at least one containing data. Given " << std::to_string(parameters.size()) << "." << std::endl;
      return false;
    }
    // Check data count entry
    data_count = this->convertShort(parameters);
    if(data_count != (parameters.size() - 2))
    {
      std::cout << __PRETTY_FUNCTION__ << ": RT/RC data command must have parameters starting with 2 bytes to represent data length. The first two bytes do not match the length of the data: DataCount " << std::to_string(data_count) << " != Data length " << std::to_string(parameters.size() - 2) << "." << std::endl;
      return false;
    }
    break;
  case COMMAND_EXECUTE_FUNCTION:
    /**
     * Command Execute Function
     *   Byte: Value
     *      0: 'R'
     *      1: 'E'
     *      2: FunctionID
     *      3: ParameterID[0]
     *      4: ParameterID[1]
     */
    break;
  case COMMAND_STOP:
    /**
     * Command Stop
     *   Byte: Value
     *      0: 'R'
     *      1: 'S'
     *      2: FunctionID
     *      3: ParameterID[0]
     */
    break;
  case COMMAND_OPEN:
    /**
     * Command Open
     *   Byte: Value
     *      0: 'R'
     *      1: 'O'
     *      2: SafetyID[0]
     */
    break;
  case COMMAND_ABORT:
    /**
     * Command Abort
     *    Byte: Value
     *       0: 'R'
     *       1: 'A'
     */
    break;
  default:
    break;
  }

  // Add parameters
  message.reserve(message.size() + parameters.size());
  message.insert(message.end(), parameters.begin(), parameters.end());

  // Checksum
  uint16_t crc = this->calculateChecksum(message);

  // Append checksum to end of message
  this->appendShort(message, crc);

  return true;
}

/**
 * Call, send message and receive response
 */
bool
EwellixSerial::call(uint8_t command, const std::vector<uint8_t>parameters, std::vector<uint8_t> &response, std::vector<uint8_t> &data)
{
  std::vector<uint8_t> message;

  if(!this->generateMessage(command, parameters, message))
  {
    return false;
  }

  if(!this->send(message))
  {
    return false;
  }

  if(!this->receive(message, response, data))
  {
    return false;
  }

  return true;
}

/**
 * Convert Data from vector to short
 */
uint16_t
EwellixSerial::convertShort(const std::vector<uint8_t> data)
{
  return(*reinterpret_cast<const uint16_t *>(&data[0]));
}

/**
 * Convert Data from vector to integer
 */
uint32_t
EwellixSerial::convertInteger(const std::vector<uint8_t> data)
{
  return(*reinterpret_cast<const uint32_t *>(&data[0]));
}

/**
 * Convert data from vector to float
 */
float
EwellixSerial::convertFloat(const std::vector<uint8_t> data)
{
  return(*reinterpret_cast<const float *>(&data[0]));
}

/**
 * Convert data from Cycle1 data to struct DataCycle1
 */
EwellixSerial::DataCycle1
EwellixSerial::convertCycle1(const std::vector<uint8_t> data)
{
  DataCycle1 converted;
  converted.actual_position_a1 = *reinterpret_cast<const signed int *>(&data[0]);
  converted.actual_position_a2 = *reinterpret_cast<const signed int *>(&data[4]);
  converted.speed_a1 = *reinterpret_cast<const unsigned short *>(&data[8]);
  converted.speed_a2 = *reinterpret_cast<const unsigned short *>(&data[10]);
  converted.status1_a1 = *reinterpret_cast<const unsigned char *>(&data[12]);
  converted.status1_a2 = *reinterpret_cast<const unsigned char *>(&data[13]);
  return converted;
}

/**
 * Convert data from Status1 byte to struct Status1
 */
EwellixSerial::Status1
EwellixSerial::convertStatus1(const std::vector<uint8_t> data)
{
  EwellixSerial::Status1 converted;
  converted.available = bool(data[0] & 0x01);
  converted.limit_in_out = bool(data[0] & 0x02);
  converted.switch1 = bool(data[0] & 0x04);
  converted.switch2 = bool(data[0] & 0x08);
  converted.motion = bool(data[0] & 0x10);
  converted.reached = bool(data[0] & 0x20);
  converted.out_position = bool(data[0] & 0x40);
  converted.stroke = bool(data[0] & 0x80);
  return converted;
}

}

