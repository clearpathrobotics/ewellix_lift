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

/**
 * EwellixSerial constructor with default values
 */
EwellixSerial::EwellixSerial():
  port_("/ttyUSB0"),
  baud_rate_(38400),
  timeout_(1000)
{

}

/**
 * EwellixSerial constructor with custom values
 */
EwellixSerial::EwellixSerial(std::string port, int baud_rate, int timeout):
  port_(port),
  baud_rate_(baud_rate),
  timeout_(timeout)
{

}

/**
 * EwellixSerial destructor. Deactivates connection and closes the serial port.
 */
EwellixSerial::~EwellixSerial()
{
  this->deactivate();
  this->close();
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
  std::vector<uint8_t> parameters = {SafetyMode::REMOTE_STOP};
  std::vector<uint8_t> response;
  std::vector<uint8_t> data;

  return this->call(Command::OPEN, parameters, response, data);
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

  return this->call(Command::ABORT, parameters, response, data);
}

/**
 * Cylic message to maintain remote function.
 *
 * @return true if successful
 */
bool
EwellixSerial::cycle()
{
  std::vector<uint8_t> parameters = {0x01, 0x00, Empty::BYTE};
  std::vector<uint8_t> response;
  std::vector<uint8_t> data;

  return this->call(Command::CYCLIC, parameters, response, data);
}

/**
 * Setup the CyclicObject1.
 * A cyclic object is an array of 12, 16-bit entries. The first six shorts correspond
 * to the parameters that the cyclic command will write to, while the later six
 * correspond to the parameters that the cyclic command will retrieve.
 *
 * The first cyclic object is setup to write the remote actuator position and to
 * retrieve the position, speed, and status of only the first two actuators.
 *
 * @return true if command is successful.
 */
bool
EwellixSerial::setCyclicObject1()
{
  std::vector<uint8_t> data;

  // Write: Remote Position Actuator1
  this->appendShort(data, WritableDataField::REMOTE_POSITION_ACTUATOR + Actuator::ACTUATOR_1);
  // Write: Remote Position Actuator2
  this->appendShort(data, WritableDataField::REMOTE_POSITION_ACTUATOR + Actuator::ACTUATOR_2);
  // Padding
  this->appendShort(data, Empty::SHORT);
  this->appendShort(data, Empty::SHORT);
  this->appendShort(data, Empty::SHORT);
  this->appendShort(data, Empty::SHORT);
  // Read: Actual Position Actuator1
  this->appendShort(data, DataField::ACTUAL_POSITION_ACTUATOR + Actuator::ACTUATOR_1);
  // Read: Actual Position Actuator2
  this->appendShort(data, DataField::ACTUAL_POSITION_ACTUATOR + Actuator::ACTUATOR_2);
  // Read: Speed Actuator1
  this->appendShort(data, DataField::SPEED_ACTUATOR + Actuator::ACTUATOR_1);
  // Read: Speed Actuator2
  this->appendShort(data, DataField::SPEED_ACTUATOR + Actuator::ACTUATOR_2);
  // Read: Status1 Actuator1
  this->appendShort(data, DataField::STATUS_1_ACTUATOR + Actuator::ACTUATOR_1);
  // Read: Status1 Actuator2
  this->appendShort(data, DataField::STATUS_1_ACTUATOR + Actuator::ACTUATOR_2);

  return this->set(WritableDataField::CYCLIC_OBJECT + CyclicObject::OBJECT_1, data);
}

/**
 * Get the CyclicObject1.
 *
 * Retrieve the 16-bit array of length 12. Used to ensure that the CyclicObject1
 * was set correctly.
 *
 * @param[out] data, returns the data retrieved. If successful will return with the
 * 24 byte array corresponding to the CyclicObject.
 *
 * @return true if successfully retrieve data.
 */
bool
EwellixSerial::getCyclicObject1(std::vector<uint8_t> &data)
{
  return this->get(WritableDataField::CYCLIC_OBJECT + CyclicObject::OBJECT_1, data);
}

/**
 * Cycle using the CylicObject1
 *
 * A cycle command must be sent at least every 500ms to maintain the remote
 * communication. The CyclicObject allows data to be set and retrieve while
 * simultaneously sending a cycle command.
 *
 * @param[in] a1_position: remote/desired position of the first actuator
 * @param[in] a2_position: remote/desired position of the second actuators
 * @param[out] data: byte array with the current position, speed, and status of the
 * first two actuators.
 * @return true if successfully sent cycle command and retrieved data.
 */
bool
EwellixSerial::cycle1(int a1_position, int a2_position, std::vector<uint8_t> &data)
{
  data.clear();
  std::vector<uint8_t> parameters;
  std::vector<uint8_t> response;

  std::vector<uint8_t> write_data = {CyclicObject::OBJECT_1};
  this->appendInteger(write_data, a1_position);
  this->appendInteger(write_data, a2_position);

  uint16_t size = write_data.size();

  this->appendShort(parameters, size);
  this->appendVector(parameters, write_data);

  return this->call(Command::CYCLIC, parameters, response, data);
}

/**
 * Setup the CyclicObject2.
 * A cyclic object is an array of 12, 16-bit entries. The first six shorts correspond
 * to the parameters that the cyclic command will write to, while the later six
 * correspond to the parameters that the cyclic command will retrieve.
 *
 * The second cyclic object is setup to write the remote actuator positions, and
 * retrieve the current positions, remote positions, speeds, currents, status of all
 * six possible actuators and retrieve the last five error codes.
 *
 * @return true if command is successful.
 */
bool
EwellixSerial::setCyclicObject2()
{
  std::vector<uint8_t> data;

  // Write: Remote Position
  this->appendShort(data, WritableDataField::REMOTE_POSITION_ALL);
  // Padding
  this->appendShort(data, Empty::SHORT);
  this->appendShort(data, Empty::SHORT);
  this->appendShort(data, Empty::SHORT);
  this->appendShort(data, Empty::SHORT);
  this->appendShort(data, Empty::SHORT);
  // Read: Actual Position
  this->appendShort(data, DataField::ACTUAL_POSITION_ALL);
  // Read: Remote Position
  this->appendShort(data, WritableDataField::REMOTE_POSITION_ALL);
  // Read: Speed
  this->appendShort(data, DataField::SPEED_ALL);
  // Read: Current
  this->appendShort(data, DataField::CURRENT_ALL);
  // Read: Status1
  this->appendShort(data, DataField::STATUS_1_ALL);
  // Read: Error
  this->appendShort(data, DataField::ERROR_CODE_HISTORY);

  return this->set(WritableDataField::CYCLIC_OBJECT + CyclicObject::OBJECT_2, data);
}

/**
 * Get the CyclicObject2.
 *
 * Retrieve the 16-bit array of length 12. Used to ensure that the CyclicObject1
 * was set correctly.
 *
 * @param[out] data, returns the data retrieved. If successful will return with the
 * 24 byte array corresponding to the CyclicObject.
 *
 * @return true if successfully retrieve data.
 */
bool
EwellixSerial::getCyclicObject2(std::vector<uint8_t> &data)
{
  return this->get(WritableDataField::CYCLIC_OBJECT + CyclicObject::OBJECT_2, data);
}

/**
 * Cycle using the CylicObject2
 *
 * A cycle command must be sent at least every 500ms to maintain the remote
 * communication. The CyclicObject allows data to be set and retrieve while
 * simultaneously sending a cycle command.
 *
 * @param[in] positions: remote/desired positions of actuators. Can pass an array
 * of at most size 6 corresponding to the maximum number of actuators.
 * @param[out] data: byte array with the current positions, remote positions, speeds, currents, and status of all actuators. And, the error code history.
 * @return true if successfully sent cycle command and retrieved data.
 */
bool
EwellixSerial::cycle2(const std::vector<int> positions, std::vector<uint8_t> &data)
{
  data.clear();
  std::vector<uint8_t> parameters;
  std::vector<uint8_t> response;

  std::vector<uint8_t> write_data = {CyclicObject::OBJECT_2};
  this->appendIntegerVector(write_data, positions);
  for(size_t i = positions.size(); i < 6; i++)
  {
    this->appendInteger(write_data, 0);
  }

  uint16_t size = write_data.size();

  this->appendShort(parameters, size);
  this->appendVector(parameters, write_data);

  return this->call(Command::CYCLIC, parameters, response, data);
}

/**
 * Get Data command.
 *
 * @param[in] field. Parameter field of data to retrieve.
 * @param[out] data. Byte array with data corresponding to parameter field.
 * @return true if command successfully retrieved data.
 */
bool
EwellixSerial::get(uint16_t field, std::vector<uint8_t> &data)
{
  data.clear();
  std::vector<uint8_t> parameters;
  std::vector<uint8_t> response;

  this->appendShort(parameters, static_cast<uint16_t>(field));

  return this->call(Command::DATA_GET, parameters, response, data);
}

/**
 * Set Data command.
 *
 * @param[in] field. Parameter field of data to set.
 * @param[in] data. Byte array with data to write to parameter field.
 * @return true if command successfully set data.
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

  return this->call(Command::DATA_TRANSFER, parameters, response, empty);
}

/**
 * Execute command
 *
 * @param[in] Actuator index.
 * @param[in] Function to execute.
 * @return true if execute command sent successfully.
 */
bool
EwellixSerial::execute(uint8_t actuator, uint8_t function)
{
  std::vector<uint8_t> empty;
  std::vector<uint8_t> parameters;
  std::vector<uint8_t> response;

  parameters.push_back(actuator);
  parameters.push_back(function);
  parameters.push_back(Empty::BYTE);

  return this->call(Command::EXECUTE_FUNCTION, parameters, response, empty);
}

/**
 * Stop command.
 *
 * @param[in] Actuator index.
 * @param[in] Stop mode.
 * @return true if stop command sent successfully.
 */
bool
EwellixSerial::stop(uint8_t actuator, uint8_t mode)
{
  std::vector<uint8_t> empty;
  std::vector<uint8_t> parameters;
  std::vector<uint8_t> response;

  parameters.push_back(actuator);
  parameters.push_back(mode);

  return this->call(Command::STOP, parameters, response, empty);
}

/**
 * Check Response Checksum.
 *
 * @param[in] response. Byte array of entire response.
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
 * @param[in] message. Byte array of entire message to be sent.
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
 * @param[in] message. Byte array of entire message to be sent with checksum.
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
 * Receive message.
 * Parse one byte at a time and ensure that response matches the protocol.
 *
 * @param[in] message. Byte array that was sent.
 * @param[out] response. Byte array to populate with received bytes.
 * @param[out] data. Byte array to populate with data retrieved. Only used by
 * certain commands.
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
  std::chrono::milliseconds elapsed;
  std::chrono::steady_clock::time_point current;
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

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
          if(byte != Command::REMOTE)
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
            std::cout << __PRETTY_FUNCTION__ << ": Response command type does not match sent command. Sent: " << command << " Received: " << byte << "Trying again..." << std::endl;
            response.clear();
            count = 0;
            break;
          }
          // Special Case Cyclic without Data
          if(byte == Command::CYCLIC && message[4] == Empty::BYTE)
          {
            command = Command::REMOTE;
          }
          response.push_back(byte);
          count++;
          break;
        case 2:
          // Byte 2: Acknowledge or Error
          success &= byte == Communication::ACK;
          receiving = success;
          response.push_back(byte);
          count++;
          break;
        default:
          switch(command)
          {
            case Command::DATA_GET:
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
            case Command::CYCLIC:
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
            case Command::DATA_TRANSFER:
            case Command::EXECUTE_FUNCTION:
            case Command::STOP:
            case Command::OPEN:
            case Command::ABORT:
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
      current = std::chrono::steady_clock::now();
      elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current-start);

      receiving &= elapsed.count() < timeout_;
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
 * Create message
 *
 * @param[in] command. Type of action to command.
 * @param[in] parameters. Parameters associated with action.
 * @param[out] message. Complete message with CRC checksum.
 * @return true if parameters match command and message was generated successfully.
 */
bool
EwellixSerial::generateMessage(uint8_t command, const std::vector<uint8_t> parameters, std::vector<uint8_t> &message)
{
  // Clear
  message.clear();

  // Add Remote Command
  message.push_back(Command::REMOTE);
  message.push_back(command);

  // Data count
  uint8_t data_count;

  // Check required parameters
  switch (command)
  {
  case Command::DATA_GET:
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
  case Command::DATA_TRANSFER:
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
  case Command::CYCLIC:
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
  case Command::EXECUTE_FUNCTION:
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
  case Command::STOP:
    /**
     * Command Stop
     *   Byte: Value
     *      0: 'R'
     *      1: 'S'
     *      2: FunctionID
     *      3: ParameterID[0]
     */
    break;
  case Command::OPEN:
    /**
     * Command Open
     *   Byte: Value
     *      0: 'R'
     *      1: 'O'
     *      2: SafetyID[0]
     */
    break;
  case Command::ABORT:
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
 * Call.
 * Generate and send message from given action and parameters.
 * Receive response.
 *
 * @param[in] command. Action type.
 * @param[in] parameters. Parameters associated with action.
 * @param[out] response. Byte array of entire received message.
 * @param[out] data. Byte array of data. Only used for actions that retrieve data.
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
 * Utility: append short to byte vector with LSB first.
 *
 * @param[out] message to append data to.
 * @param[in] 16-bit data to append to message.
 */
void
EwellixSerial::appendShort(std::vector<uint8_t> &message, uint16_t data)
{
  message.push_back(static_cast<uint8_t> (data & 0x00FF));
  message.push_back(static_cast<uint8_t> (data >> 8));
}

/**
 * Utility: append integer to byte vector with LSB first.
 *
 * @param[out] message to append data to.
 * @param[in] 32-bit data to append to message.
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
 * Utility: append byte vector to byte vector.
 *
 * @param[out] message to append data to.
 * @param[in] byte vector to append to message.
 */
void
EwellixSerial::appendVector(std::vector<uint8_t> &message, const std::vector<uint8_t> v)
{
  message.insert(message.end(), v.begin(), v.end());
}

/**
 * Utility: append vector of integers to byte vector.
 *
 * @param[out] message to append data to.
 * @param[in] integer vector to append to message with LSB first.
 */
void
EwellixSerial::appendIntegerVector(std::vector<uint8_t> &message, const std::vector<int> v)
{
  for(auto integer : v)
  {
    this->appendInteger(message, integer);
  }
}

/**
 * Utility: convert data from vector to short
 *
 * @param[in] data. Byte vector of data to cast to 16-bit.
 * @return 16-bit, unsigned short.
 */
uint16_t
EwellixSerial::convertShort(const std::vector<uint8_t> data)
{
  return(*reinterpret_cast<const uint16_t *>(&data[0]));
}

/**
 * Utility: convert data from vector to integer
 *
 * @param[in] data. Byte vector of data to cast to 32-bit
 * @return 32-bit, unsigned integer.
 */
uint32_t
EwellixSerial::convertInteger(const std::vector<uint8_t> data)
{
  return(*reinterpret_cast<const uint32_t *>(&data[0]));
}

/**
 * Utility: convert data from vector to float
 *
 * @param[in] data. Byte vector of data to cast to 32-bit float.
 * @return 32-bit, float.
 */
float
EwellixSerial::convertFloat(const std::vector<uint8_t> data)
{
  return(*reinterpret_cast<const float *>(&data[0]));
}

/**
 * Status1 constructor with all zeros
 */
EwellixSerial::Status1::Status1() : code(0),
  available(false), limit_in_out(false), switch1(false), switch2(false),
  motion(false), reached(false), out_position(false), stroke(false)
{
}

/**
 * Status1 constructor setting variables with data from response.
 *
 * @param[in] data from response.
 */
EwellixSerial::Status1::Status1(const std::vector<uint8_t> data) : code(0),
  available(false), limit_in_out(false), switch1(false), switch2(false),
  motion(false), reached(false), out_position(false), stroke(false)
{
  this->setFromData(data);
}

/**
 * Status1::setFromData, set variables from data from response.
 *
 * @param[in] data from response.
 */
void
EwellixSerial::Status1::setFromData(const std::vector<uint8_t> data)
{
  code = data[0];
  available =  bool(data[0] & (0x01 << 0));
  limit_in_out = bool(data[0] & (0x01 << 1));
  switch1 = bool(data[0] & (0x01 << 2));
  switch2 = bool(data[0] & (0x01 << 3));
  motion = bool(data[0] & (0x01 << 4));
  reached = bool(data[0] & (0x01 << 5));
  out_position = bool(data[0] & (0x01 << 6));
  stroke = bool(data[0] & (0x01 << 7));
}

/**
 * SCUError constructor with all zeros
 */
EwellixSerial::SCUError::SCUError() : code(0),
  fault_ram(false), fault_rom(false), fault_cpu(false), stack_overrun(false),
  sequence_error(false), hand_switch_short(false), binary_inputs_short(false),
  faulty_relay(false), move_enable_comms(false), move_enable_incorrect(false),
  over_temperature(false), battery_discharge(false), over_current(false),
  drive_1_error(false), drive_2_error(false), drive_3_error(false),
  drive_4_error(false), drive_5_error(false), drive_6_error(false),
  position_difference(false), remote_timeout(false), lockbox_comm_error(false),
  ram_config_data_crc(false), ram_user_data_crc(false), ram_lockbox_data_crc(false),
  ram_dynamic_data_crc(false), ram_calib_data_crc(false), ram_hw_settings_crc(false),
  io_test(false), idf_opsys_error(false)
{
}

/**
 * SCUError constructor setting variables with data from response.
 *
 * @param[in] data from response.
 */
EwellixSerial::SCUError::SCUError(const std::vector<uint8_t> data) : code(0),
  fault_ram(false), fault_rom(false), fault_cpu(false), stack_overrun(false),
  sequence_error(false), hand_switch_short(false), binary_inputs_short(false),
  faulty_relay(false), move_enable_comms(false), move_enable_incorrect(false),
  over_temperature(false), battery_discharge(false), over_current(false),
  drive_1_error(false), drive_2_error(false), drive_3_error(false),
  drive_4_error(false), drive_5_error(false), drive_6_error(false),
  position_difference(false), remote_timeout(false), lockbox_comm_error(false),
  ram_config_data_crc(false), ram_user_data_crc(false), ram_lockbox_data_crc(false),
  ram_dynamic_data_crc(false), ram_calib_data_crc(false), ram_hw_settings_crc(false),
  io_test(false), idf_opsys_error(false)
{
  this->setFromData(data);
}

/**
 * SCUError::setFromData, set variables from data from response.
 *
 * @param[in] data from response.
 */
void
EwellixSerial::SCUError::setFromData(const std::vector<uint8_t> data)
{
  code = *reinterpret_cast<const int *>(&data[0]);

  fault_ram = bool(data[0] & (0x01 << 0)); // 1
  fault_rom = bool(data[0] & (0x01 << 1)); // 2
  fault_cpu = bool(data[0] & (0x01 << 2)); // 3
  stack_overrun = bool(data[0] & (0x01 << 3)); // 4
  sequence_error = bool(data[0] & (0x01 << 4)); // 5
  hand_switch_short = bool(data[0] & (0x01 << 5)); // 6
  binary_inputs_short = bool(data[0] & (0x01 << 6)); // 7
  faulty_relay = bool(data[0] & (0x01 << 7)); // 8
  // 9, empty
  move_enable_comms = bool(data[1] & (0x01 << 1)); // 10
  move_enable_incorrect = bool(data[1] & (0x01 << 2)); // 11
  over_temperature = bool(data[1] & (0x01 << 3)); // 12
  battery_discharge = bool(data[1] & (0x01 << 4)); // 13
  over_current = bool(data[1] & (0x01 << 5)); // 14
  drive_1_error = bool(data[1] & (0x01 << 6)); // 15
  drive_2_error = bool(data[1] & (0x01 << 7)); // 16

  drive_3_error = bool(data[2] & (0x01 << 0)); // 17
  drive_4_error = bool(data[2] & (0x01 << 1)); // 18
  drive_5_error = bool(data[2] & (0x01 << 2)); // 19
  drive_6_error = bool(data[2] & (0x01 << 3)); // 20
  position_difference = bool(data[2] & (0x01 << 4)); // 21
  remote_timeout = bool(data[2] & (0x01 << 5)); // 22
  // 23, empty
  lockbox_comm_error = bool(data[2] & (0x01 << 7)); // 24

  ram_config_data_crc = bool(data[3] & (0x01 << 0)); // 25
  ram_user_data_crc = bool(data[3] & (0x01 << 1)); // 26
  ram_lockbox_data_crc = bool(data[3] & (0x01 << 2)); // 27
  ram_dynamic_data_crc = bool(data[3] & (0x01 << 3)); // 28
  ram_calib_data_crc = bool(data[3] & (0x01 << 4)); // 29
  ram_hw_settings_crc = bool(data[3] & (0x01 << 5)); // 30
  io_test = bool(data[3] & (0x01 << 6)); // 31
  idf_opsys_error = bool(data[3] & (0x01 << 7)); // 32
}

/**
 * Cycle1Data constructor with all zeros.
 */
EwellixSerial::Cycle1Data::Cycle1Data() :
  actual_positions({0, 0}), speeds({0, 0}),
  status({EwellixSerial::Status1(), EwellixSerial::Status1()})
{
}

/**
 * Cycle1Data constructor setting variables with data from response.
 *
 * @param[in] data from response.
 */
EwellixSerial::Cycle1Data::Cycle1Data(const std::vector<uint8_t> data) :
  actual_positions({0, 0}), speeds({0, 0}),
  status({EwellixSerial::Status1(), EwellixSerial::Status1()})
{
  this->setFromData(data);
}

/**
 * Cycle1Data::setFromData, set variables from data from response.
 *
 * @param[in] data from response.
 */
void
EwellixSerial::Cycle1Data::setFromData(const std::vector<uint8_t> data)
{
  assert(data.size() == 14);
  actual_positions[0] = *reinterpret_cast<const signed int *>(&data[0]);
  actual_positions[1] = *reinterpret_cast<const signed int *>(&data[4]);
  speeds[0] = *reinterpret_cast<const unsigned short *>(&data[8]);
  speeds[1] = *reinterpret_cast<const unsigned short *>(&data[10]);
  status[0].setFromData({data[12]});
  status[1].setFromData({data[13]});
 }

/**
 * Cycle2Data constructor with all zeros.
 */
EwellixSerial::Cycle2Data::Cycle2Data() :
  actual_positions({0, 0, 0, 0, 0, 0}), remote_positions({0, 0, 0, 0, 0, 0}),
  speeds({0, 0, 0, 0, 0, 0}), currents({0, 0, 0, 0, 0, 0}),
  status(std::vector<EwellixSerial::Status1>(6,EwellixSerial::Status1())),
  errors(std::vector<EwellixSerial::SCUError>(6,EwellixSerial::SCUError()))
{
}

/**
 * Cycle2Data constructor setting variables with data from response.
 *
 * @param[in] data from response.
 */
EwellixSerial::Cycle2Data::Cycle2Data(const std::vector<uint8_t> data) :
  actual_positions({0, 0, 0, 0, 0, 0}), remote_positions({0, 0, 0, 0, 0, 0}),
  speeds({0, 0, 0, 0, 0, 0}), currents({0, 0, 0, 0, 0, 0}),
  status(std::vector<EwellixSerial::Status1>(6,EwellixSerial::Status1())),
  errors(std::vector<EwellixSerial::SCUError>(6,EwellixSerial::SCUError()))
{
  this->setFromData(data);
}

/**
 * Cycle2Data::setFromData
 *
 * @param[in] data from response.
 */
void
EwellixSerial::Cycle2Data::setFromData(const std::vector<uint8_t> data)
{
  assert(data.size() == 98);
  for(int i = 0; i < 6; i++)
  {
    actual_positions[i] = *reinterpret_cast<const int *>(&data[0 + i*sizeof(int)]);
    remote_positions[i] = *reinterpret_cast<const int *>(&data[24 + i*sizeof(int)]);
    speeds[i] = *reinterpret_cast<const uint16_t *>(&data[48 + i*sizeof(uint16_t)]);
    currents[i] = *reinterpret_cast<const uint16_t *>(&data[60 + i*sizeof(uint16_t)]);
    status[i].setFromData({data[72 + i*sizeof(uint8_t)]});
  }
  for(int i = 0; i < 6; i++)
  {
    errors[i].setFromData({
      data[78 + i*sizeof(int) + 0],
      data[78 + i*sizeof(int) + 1],
      data[78 + i*sizeof(int) + 2],
      data[78 + i*sizeof(int) + 3]
    });
  }
}
}

