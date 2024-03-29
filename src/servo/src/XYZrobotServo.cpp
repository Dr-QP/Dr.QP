#include "servo/XYZrobotServo.h"

#define CMD_EEPROM_WRITE 0x01
#define CMD_EEPROM_READ  0x02
#define CMD_RAM_WRITE    0x03
#define CMD_RAM_READ     0x04
#define CMD_I_JOG        0x05
#define CMD_S_JOG        0x06
#define CMD_STAT         0x07
#define CMD_ROLLBACK     0x08
#define CMD_REBOOT       0x09

#define SET_POSITION_CONTROL 0
#define SET_SPEED_CONTROL 1
#define SET_TORQUE_OFF 2
#define SET_POSITION_CONTROL_SERVO_ON 3

XYZrobotServo::XYZrobotServo(Stream & stream, uint8_t id)
{
  this->stream = &stream;
  this->id = id;
  this->lastError = XYZrobotServoError::None;
}

void XYZrobotServo::eepromWrite(uint8_t startAddress, const uint8_t * data, uint8_t dataSize)
{
  memoryWrite(CMD_EEPROM_WRITE, startAddress, data, dataSize);
}

void XYZrobotServo::eepromRead(uint8_t startAddress, uint8_t * data, uint8_t dataSize)
{
  memoryRead(CMD_EEPROM_READ, startAddress, data, dataSize);
}

void XYZrobotServo::ramWrite(uint8_t startAddress, const uint8_t * data, uint8_t dataSize)
{
  memoryWrite(CMD_RAM_WRITE, startAddress, data, dataSize);
}

void XYZrobotServo::ramRead(uint8_t startAddress, uint8_t * data, uint8_t dataSize)
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
  ramWrite(16, (uint8_t *)&value, 2);
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
  sendRequest(CMD_STAT, NULL, 0);
  readAck(CMD_STAT, (uint8_t *)&status, sizeof(XYZrobotServoStatus));
  return status;
}

void XYZrobotServo::setPosition(uint16_t position, uint8_t playtime)
{
  sendIJog(position, SET_POSITION_CONTROL, playtime);
}

void XYZrobotServo::setSpeed(int16_t speed, uint8_t playtime)
{
  sendIJog(speed, SET_SPEED_CONTROL, playtime);
}

void XYZrobotServo::torqueOff()
{
  sendIJog(0, SET_TORQUE_OFF, 0);
}

void XYZrobotServo::rollback()
{
  sendRequest(CMD_ROLLBACK, NULL, 0);
}

void XYZrobotServo::reboot()
{
  sendRequest(CMD_REBOOT, NULL, 0);
}

void XYZrobotServo::flushRead()
{
  stream->flushRead();
  // while(stream->available()) { stream->read(); }
}

void XYZrobotServo::sendRequest(uint8_t cmd,
  const uint8_t * data1, uint8_t data1Size,
  const uint8_t * data2, uint8_t data2Size)
{
  uint8_t header[7];

  uint8_t size = data1Size + data2Size + sizeof(header);

  uint8_t checksum = size ^ id ^ cmd;
  for (uint8_t i = 0; i < data1Size; i++) { checksum ^= data1[i]; }
  for (uint8_t i = 0; i < data2Size; i++) { checksum ^= data2[i]; }

  header[0] = 0xFF;
  header[1] = 0xFF;
  header[2] = size;
  header[3] = id;
  header[4] = cmd;
  header[5] = checksum & 0xFE;
  header[6] = ~checksum & 0xFE;

  stream->write(header, sizeof(header));
  if (data1Size) { stream->write(data1, data1Size); }
  if (data2Size) { stream->write(data2, data2Size); }

  lastError = XYZrobotServoError::None;
}

void XYZrobotServo::readAck(uint8_t cmd,
  uint8_t * data1, uint8_t data1Size,
  uint8_t * data2, uint8_t data2Size)
{
  // The CMD byte for an acknowledgment always has bit 6 set.
  cmd |= 0x40;

  uint8_t header[7];

  uint8_t size = sizeof(header) + data1Size + data2Size;

  uint8_t byteCount = stream->readBytes(header, sizeof(header));
  if (byteCount != sizeof(header))
  {
    lastError = XYZrobotServoError::HeaderTimeout;
    return;
  }

  if (header[0] != 0xFF)
  {
    lastError = XYZrobotServoError::HeaderByte1Wrong;
    return;
  }

  if (header[1] != 0xFF)
  {
    lastError = XYZrobotServoError::HeaderByte2Wrong;
    return;
  }

  if (header[3] != id)
  {
    lastError = XYZrobotServoError::IdWrong;
    return;
  }

  if (header[4] != cmd)
  {
    lastError = XYZrobotServoError::CmdWrong;
    return;
  }

  if (header[2] != size)
  {
    lastError = XYZrobotServoError::SizeWrong;
    return;
  }

  if (data1Size)
  {
    byteCount = stream->readBytes(data1, data1Size);
    if (byteCount != data1Size)
    {
      lastError = XYZrobotServoError::Data1Timeout;
      return;
    }
  }

  if (data2Size)
  {
    byteCount = stream->readBytes(data2, data2Size);
    if (byteCount != data2Size)
    {
      lastError = XYZrobotServoError::Data2Timeout;
      return;
    }
  }

  uint8_t checksum = size ^ id ^ cmd;
  for (uint8_t i = 0; i < data1Size; i++) { checksum ^= data1[i]; }
  for (uint8_t i = 0; i < data2Size; i++) { checksum ^= data2[i]; }

  if (header[5] != (checksum & 0xFE))
  {
    lastError = XYZrobotServoError::Checksum1Wrong;
    return;
  }

  if (header[6] != (~checksum & 0xFE))
  {
    lastError = XYZrobotServoError::Checksum2Wrong;
    return;
  }

  lastError = XYZrobotServoError::None;
}

void XYZrobotServo::memoryWrite(uint8_t cmd, uint8_t startAddress,
  const uint8_t * data, uint8_t dataSize)
{
  uint8_t request[2];
  request[0] = startAddress;
  request[1] = dataSize;

  sendRequest(cmd, request, sizeof(request), data, dataSize);
}

void XYZrobotServo::memoryRead(uint8_t cmd, uint8_t startAddress,
  uint8_t * data, uint8_t dataSize)
{
  flushRead();

  uint8_t request[2];
  request[0] = startAddress;
  request[1] = dataSize;
  sendRequest(cmd, request, sizeof(request));

  uint8_t response[4];
  readAck(cmd, response, 4, data, dataSize);
  if (getLastError()) { return; }

  // Despite what the A1-16 datasheet says, the first two bytes of the response
  // tend to 0, and the start address and data size come after that.

  if (response[2] != request[0])
  {
    lastError = XYZrobotServoError::ReadOffsetWrong;
    return;
  }

  if (response[3] != request[1])
  {
    lastError = XYZrobotServoError::ReadLengthWrong;
    return;
  }
}

void XYZrobotServo::sendIJog(uint16_t goal, uint8_t type, uint8_t playtime)
{
  uint8_t data[5];
  data[0] = goal & 0xFF;
  data[1] = goal >> 8 & 0xFF;
  data[2] = type;
  data[3] = id;
  data[4] = playtime;
  sendRequest(CMD_I_JOG, data, sizeof(data));
}
