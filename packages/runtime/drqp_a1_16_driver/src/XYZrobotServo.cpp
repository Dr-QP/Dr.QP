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

static_assert(sizeof(XYZrobotServoRAM) == 80, "XYZrobotServoRAM has wrong size");

static_assert(offsetof(XYZrobotServoRAM, sID) == 0, "XYZrobotServoRAM::sID has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, ACK_Policy) == 1, "XYZrobotServoRAM::ACK_Policy has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Alarm_LED_Policy) == 2,
  "XYZrobotServoRAM::Alarm_LED_Policy has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Torque_Policy) == 3,
  "XYZrobotServoRAM::Torque_Policy has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, SPDctrl_Policy) == 4,
  "XYZrobotServoRAM::SPDctrl_Policy has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Max_Temperature) == 5,
  "XYZrobotServoRAM::Max_Temperature has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Min_Voltage) == 6, "XYZrobotServoRAM::Min_Voltage has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Max_Voltage) == 7, "XYZrobotServoRAM::Max_Voltage has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Acceleration_Ratio) == 8,
  "XYZrobotServoRAM::Acceleration_Ratio has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Max_Wheel_Ref_Position) == 12,
  "XYZrobotServoRAM::Max_Wheel_Ref_Position has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Max_PWM) == 16, "XYZrobotServoRAM::Max_PWM has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Overload_Threshold) == 18,
  "XYZrobotServoRAM::Overload_Threshold has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Min_Position) == 20,
  "XYZrobotServoRAM::Min_Position has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Max_Position) == 22,
  "XYZrobotServoRAM::Max_Position has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Position_Kp) == 24, "XYZrobotServoRAM::Position_Kp has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Position_Kd) == 26, "XYZrobotServoRAM::Position_Kd has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Position_Ki) == 28, "XYZrobotServoRAM::Position_Ki has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Close_to_Open_Ref_Position) == 30,
  "XYZrobotServoRAM::Close_to_Open_Ref_Position has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Open_to_Close_Ref_Position) == 32,
  "XYZrobotServoRAM::Open_to_Close_Ref_Position has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Ramp_Speed) == 36, "XYZrobotServoRAM::Ramp_Speed has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, LED_Blink_Period) == 38,
  "XYZrobotServoRAM::LED_Blink_Period has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Packet_Timeout_Detection_Period) == 40,
  "XYZrobotServoRAM::Packet_Timeout_Detection_Period has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Overload_Detection_Period) == 42,
  "XYZrobotServoRAM::Overload_Detection_Period has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Inposition_Margin) == 44,
  "XYZrobotServoRAM::Inposition_Margin has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Over_Voltage_Detection_Period) == 45,
  "XYZrobotServoRAM::Over_Voltage_Detection_Period has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Over_Temperature_Detection_Period) == 46,
  "XYZrobotServoRAM::Over_Temperature_Detection_Period has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Calibration_Difference) == 47,
  "XYZrobotServoRAM::Calibration_Difference has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Status_Error) == 48,
  "XYZrobotServoRAM::Status_Error has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Status_Detail) == 49,
  "XYZrobotServoRAM::Status_Detail has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, LED_Control) == 53, "XYZrobotServoRAM::LED_Control has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Voltage) == 54, "XYZrobotServoRAM::Voltage has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Temperature) == 55, "XYZrobotServoRAM::Temperature has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Current_Control_Mode) == 56,
  "XYZrobotServoRAM::Current_Control_Mode has wrong offset");
static_assert(offsetof(XYZrobotServoRAM, Tick) == 57, "XYZrobotServoRAM::Tick has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Joint_Position) == 60,
  "XYZrobotServoRAM::Joint_Position has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, PWM_Output_Duty) == 64,
  "XYZrobotServoRAM::PWM_Output_Duty has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Bus_Current) == 66, "XYZrobotServoRAM::Bus_Current has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Position_Goal) == 68,
  "XYZrobotServoRAM::Position_Goal has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Position_Ref) == 70,
  "XYZrobotServoRAM::Position_Ref has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Omega_Goal) == 72, "XYZrobotServoRAM::Omega_Goal has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Omega_Ref) == 74, "XYZrobotServoRAM::Omega_Ref has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Requested_Counts) == 76,
  "XYZrobotServoRAM::Requested_Counts has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, ACK_Counts) == 78, "XYZrobotServoRAM::ACK_Counts has wrong offset");

static_assert(sizeof(XYZrobotServoEEPROM) == 54, "XYZrobotServoEEPROM has wrong size");

static_assert(
  offsetof(XYZrobotServoEEPROM, Model_Number) == 0,
  "XYZrobotServoEEPROM::Model_Number has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Year) == 1, "XYZrobotServoEEPROM::Year has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Version_Month) == 2,
  "XYZrobotServoEEPROM::Version_Month has wrong offset");
static_assert(offsetof(XYZrobotServoEEPROM, Day) == 3, "XYZrobotServoEEPROM::Day has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Baud_Rate) == 5, "XYZrobotServoEEPROM::Baud_Rate has wrong offset");
static_assert(offsetof(XYZrobotServoEEPROM, sID) == 6, "XYZrobotServoEEPROM::sID has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Calibration_Difference) == 53,
  "XYZrobotServoEEPROM::Calibration_Difference has wrong offset");
