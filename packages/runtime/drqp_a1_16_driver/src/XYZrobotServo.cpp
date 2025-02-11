// Copyright (C) Pololu Corporation.
// Copyright (c) 2017-2025 Anton Matosov
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "drqp_a1_16_driver/XYZrobotServo.h"
#include <cassert>
#include <cstdio>

static_assert(
  __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "This code assumes little-endian byte order.");

std::string to_string(XYZrobotServoError errorCode)
{
#define TO_STR_PREFIX "A1-16 servo error "

  switch (errorCode) {
  case XYZrobotServoError::None:
    return TO_STR_PREFIX "None: No error.";

  case XYZrobotServoError::HeaderTimeout:
    return TO_STR_PREFIX
      "HeaderTimeout: There was a timeout waiting to receive the 7-byte acknowledgment "
      "header.";

  case XYZrobotServoError::HeaderByte1Wrong:
    return TO_STR_PREFIX "HeaderByte1Wrong: The first byte of received header was not 0xFF.";

  case XYZrobotServoError::HeaderByte2Wrong:
    return TO_STR_PREFIX "HeaderByte2Wrong: The second byte of the received header was not 0xFF.";

  case XYZrobotServoError::IdWrong:
    return TO_STR_PREFIX "IdWrong: The ID byte in the received header was wrong.";

  case XYZrobotServoError::CmdWrong:
    return TO_STR_PREFIX "CmdWrong: The CMD bytes in the received header was wrong.";

  case XYZrobotServoError::SizeWrong:
    return TO_STR_PREFIX "SizeWrong: The size byte in the received header was wrong.";

  case XYZrobotServoError::Data1Timeout:
    return TO_STR_PREFIX
      "Data1Timeout: There was a timeout reading the first expected block of data in the "
      "acknowledgment.";

  case XYZrobotServoError::Data2Timeout:
    return TO_STR_PREFIX
      "Data2Timeout: There was a timeout reading the second expected block of data in the "
      "acknowledgment.";

  case XYZrobotServoError::Checksum1Wrong:
    return TO_STR_PREFIX "Checksum1Wrong: The first byte of the checksum was wrong.";

  case XYZrobotServoError::Checksum2Wrong:
    return TO_STR_PREFIX "Checksum2Wrong: The second byte of the checksum was wrong.";

  case XYZrobotServoError::ReadOffsetWrong:
    return TO_STR_PREFIX
      "ReadOffsetWrong: The offset byte returned by an EEPROM Read or RAM Read command was "
      "wrong.";

  case XYZrobotServoError::ReadLengthWrong:
    return TO_STR_PREFIX
      "ReadLengthWrong: The length byte returned by an EEPROM Read or RAM Read command was "
      "wrong.";
  }
  return TO_STR_PREFIX "Unknown";

#undef TO_STR_PREFIX
}

XYZrobotServo::XYZrobotServo(Stream& stream, uint8_t id)
{
  this->stream = &stream;
  this->id = id;
  this->lastError = XYZrobotServoError::None;
}

void XYZrobotServo::eepromWrite(uint8_t startAddress, const void* data, uint8_t dataSize)
{
  memoryWrite(CMD_EEPROM_WRITE, startAddress, data, dataSize);
}

void XYZrobotServo::eepromRead(uint8_t startAddress, void* data, uint8_t dataSize)
{
  memoryRead(CMD_EEPROM_READ, startAddress, data, dataSize);
}

void XYZrobotServo::ramWrite(uint8_t startAddress, const void* data, uint8_t dataSize)
{
  memoryWrite(CMD_RAM_WRITE, startAddress, data, dataSize);
}

void XYZrobotServo::ramRead(uint8_t startAddress, void* data, uint8_t dataSize)
{
  memoryRead(CMD_RAM_READ, startAddress, data, dataSize);
}

void XYZrobotServo::writeBaudRateEeprom(XYZrobotServoBaudRate baud)
{
  uint8_t b = (uint8_t)baud;
  memoryWrite(CMD_EEPROM_WRITE, 5, &b, 1);
}

XYZrobotServoBaudRate XYZrobotServo::readBaudRateEeprom()
{
  uint8_t b = 0;
  memoryRead(CMD_EEPROM_READ, 5, &b, 1);
  return (XYZrobotServoBaudRate)b;
}

void XYZrobotServo::writeIdEeprom(uint8_t id)
{
  memoryWrite(CMD_EEPROM_WRITE, 6, &id, 1);
}

uint8_t XYZrobotServo::readIdEeprom()
{
  uint8_t id = 0;
  memoryRead(CMD_EEPROM_READ, 6, &id, 1);
  return id;
}

void XYZrobotServo::writeIdRam(uint8_t id)
{
  memoryWrite(CMD_RAM_WRITE, 0, &id, 1);
}

void XYZrobotServo::writeAckPolicyEeprom(XYZrobotServoAckPolicy policy)
{
  uint8_t p = (uint8_t)policy;
  eepromWrite(7, &p, 1);
}

XYZrobotServoAckPolicy XYZrobotServo::readAckPolicyEeprom()
{
  uint8_t result = 0;
  eepromRead(7, &result, 1);
  return (XYZrobotServoAckPolicy)result;
}

void XYZrobotServo::writeAckPolicyRam(XYZrobotServoAckPolicy policy)
{
  uint8_t p = (uint8_t)policy;
  ramWrite(1, &p, 1);
}

void XYZrobotServo::writeAlarmLedPolicyRam(uint8_t policy)
{
  ramWrite(2, &policy, 1);
}

void XYZrobotServo::writeSpdctrlPolicyRam(XYZrobotServoSpdctrlPolicy policy)
{
  uint8_t p = (uint8_t)policy;
  ramWrite(4, &p, 1);
}

void XYZrobotServo::writeMaxPwmRam(uint16_t value)
{
  ramWrite(16, &value, 2);
}

uint16_t XYZrobotServo::readMaxPwmRam()
{
  uint16_t result = 0;
  ramRead(16, &result, 2);
  return result;
}

void XYZrobotServo::writeLedControl(uint8_t control)
{
  ramWrite(53, &control, 1);
}

XYZrobotServoAckPolicy XYZrobotServo::readAckPolicyRam()
{
  uint8_t result = 0;
  ramRead(1, &result, 1);
  return (XYZrobotServoAckPolicy)result;
}

XYZrobotServoStatus XYZrobotServo::readStatus()
{
  flushRead();

  XYZrobotServoStatus status;
  sendRequest(id, CMD_STAT, nullptr, 0);
  readAck(CMD_STAT, &status, sizeof(XYZrobotServoStatus));
  return status;
}

void XYZrobotServo::setPosition(uint16_t position, uint8_t playtime)
{
  sendIJog({position, SET_POSITION_CONTROL, id, playtime});
}

void XYZrobotServo::setSpeed(int16_t speed, uint8_t playtime)
{
  sendIJog({static_cast<uint16_t>(speed), SET_SPEED_CONTROL, id, playtime});
}

void XYZrobotServo::torqueOff()
{
  sendIJog({0, SET_TORQUE_OFF, id, 0});
}

void XYZrobotServo::torqueOn()
{
  sendIJog({0, SET_POSITION_CONTROL_SERVO_ON, id, 0});
}

void XYZrobotServo::rollback()
{
  sendRequest(id, CMD_ROLLBACK, nullptr, 0);
}

void XYZrobotServo::reboot()
{
  sendRequest(id, CMD_REBOOT, nullptr, 0);
}

void XYZrobotServo::flushRead()
{
  stream->flushRead();
}

void XYZrobotServo::sendRequest(
  uint8_t headerId, uint8_t cmd, const void* buffer1, uint8_t data1Size, const void* buffer2,
  uint8_t data2Size)
{
  const uint8_t* data1 = static_cast<const uint8_t*>(buffer1);
  const uint8_t* data2 = static_cast<const uint8_t*>(buffer2);
  uint8_t header[7];

  uint8_t size = data1Size + data2Size + sizeof(header);

  uint8_t checksum = size ^ id ^ cmd;
  for (uint8_t i = 0; i < data1Size; i++) {
    checksum ^= data1[i];
  }
  for (uint8_t i = 0; i < data2Size; i++) {
    checksum ^= data2[i];
  }

  header[0] = 0xFF;
  header[1] = 0xFF;
  header[2] = size;
  header[3] = headerId;
  header[4] = cmd;
  header[5] = checksum & 0xFE;
  header[6] = ~checksum & 0xFE;

  stream->writeBytes(header, sizeof(header));
  if (data1Size) {
    stream->writeBytes(data1, data1Size);
  }
  if (data2Size) {
    stream->writeBytes(data2, data2Size);
  }

  lastError = XYZrobotServoError::None;
}

void XYZrobotServo::readAck(
  uint8_t cmd, void* buffer1, uint8_t data1Size, void* buffer2, uint8_t data2Size)
{
  uint8_t* data1 = static_cast<uint8_t*>(buffer1);
  uint8_t* data2 = static_cast<uint8_t*>(buffer2);

  // The CMD byte for an acknowledgment always has bit 6 set.
  cmd |= 0x40;

  uint8_t header[7];

  uint8_t size = sizeof(header) + data1Size + data2Size;

  uint8_t byteCount = stream->readBytes(header, sizeof(header));
  if (byteCount != sizeof(header)) {
    lastError = XYZrobotServoError::HeaderTimeout;
    return;
  }

  if (header[0] != 0xFF) {
    lastError = XYZrobotServoError::HeaderByte1Wrong;
    return;
  }

  if (header[1] != 0xFF) {
    lastError = XYZrobotServoError::HeaderByte2Wrong;
    return;
  }

  if (header[3] != id) {
    lastError = XYZrobotServoError::IdWrong;
    return;
  }

  if (header[4] != cmd) {
    lastError = XYZrobotServoError::CmdWrong;
    return;
  }

  if (header[2] != size) {
    lastError = XYZrobotServoError::SizeWrong;
    return;
  }

  if (data1Size) {
    byteCount = stream->readBytes(data1, data1Size);
    if (byteCount != data1Size) {
      lastError = XYZrobotServoError::Data1Timeout;
      return;
    }
  }

  if (data2Size) {
    byteCount = stream->readBytes(data2, data2Size);
    if (byteCount != data2Size) {
      lastError = XYZrobotServoError::Data2Timeout;
      return;
    }
  }

  uint8_t checksum = size ^ id ^ cmd;
  for (uint8_t i = 0; i < data1Size; i++) {
    checksum ^= data1[i];
  }
  for (uint8_t i = 0; i < data2Size; i++) {
    checksum ^= data2[i];
  }

  if (header[5] != (checksum & 0xFE)) {
    lastError = XYZrobotServoError::Checksum1Wrong;
    return;
  }

  if (header[6] != (~checksum & 0xFE)) {
    lastError = XYZrobotServoError::Checksum2Wrong;
    return;
  }

  lastError = XYZrobotServoError::None;
}

void XYZrobotServo::memoryWrite(
  uint8_t cmd, uint8_t startAddress, const void* data, uint8_t dataSize)
{
  uint8_t request[2];
  request[0] = startAddress;
  request[1] = dataSize;

  sendRequest(id, cmd, request, sizeof(request), data, dataSize);
}

void XYZrobotServo::memoryRead(uint8_t cmd, uint8_t startAddress, void* data, uint8_t dataSize)
{
  flushRead();

  uint8_t request[2];
  request[0] = startAddress;
  request[1] = dataSize;
  sendRequest(id, cmd, request, sizeof(request));

  uint8_t response[4];
  readAck(cmd, response, 4, data, dataSize);
  if (isFailed()) {
    return;
  }

  // Despite what the A1-16 datasheet says, the first two bytes of the response
  // tend to 0, and the start address and data size come after that.

  if (response[2] != request[0]) {
    lastError = XYZrobotServoError::ReadOffsetWrong;
    return;
  }

  if (response[3] != request[1]) {
    lastError = XYZrobotServoError::ReadLengthWrong;
    return;
  }
}

void XYZrobotServo::sendIJog(IJogData data)
{
  sendRequest(id, CMD_I_JOG, &data, sizeof(data));
}
