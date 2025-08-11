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

#pragma once

#include <cstdint>
#include <array>
#include <vector>

enum class XYZrobotServoStatusError : uint8_t {
  None = 0,

  // Exceed Potentiometer Range Error. Blue LED on
  ExceedPotentiometerRangeError = 0x01,

  // Over Voltage Limits Error. Red LED on/ White LED off
  OverVoltageLimitsError = 0x02,

  // Over Temperature Error. Red LED on/ White LED off
  OverTemperatureError = 0x04,

  // Overload/Over-current Error. Red LED on/ White LED off
  OverloadOvercurrentError = 0x08,

  // Reserved
  Reserved = 0x10,

  // Requested Packet Checksum Error. Green LED on
  RequestedPacketChecksumError = 0x20,

  // Requested Packet Data Error. Green LED on
  RequestedPacketDataError = 0x40,

  // Requested Packet RX FIFO Error. Green LED on
  RequestedPacketRxFifoError = 0x80,
};

enum class XYZrobotServoStatusDetail : uint8_t {
  None = 0,

  // Motor Moving
  MotorMoving = 0x10,

  // Motor In-Position (Position control mode only)
  MotorInPosition = 0x20,

  // 1: Torque on (Position/Speed control), 0: Torque off
  TorqueOn = 0x40,

  // Motor Braked
  MotorBraked = 0x80,
};

/// The possible communication errors that can happen when reading the
/// acknowledgment packet from a servo.
enum class XYZrobotServoError {
  /// No error.
  None = 0,

  /// There was a timeout waiting to receive the 7-byte acknowledgment header.
  HeaderTimeout = 1,

  /// The first byte of received header was not 0xFF.
  HeaderByte1Wrong = 2,

  /// The second byte of the received header was not 0xFF.
  HeaderByte2Wrong = 3,

  /// The ID byte in the received header was wrong.
  IdWrong = 4,

  /// The CMD bytes in the received header was wrong.
  CmdWrong = 5,

  /// The size byte in the received header was wrong.
  SizeWrong = 6,

  /// There was a timeout reading the first expected block of data in the
  /// acknowledgment.
  Data1Timeout = 7,

  /// There was a timeout reading the second expected block of data in the
  /// acknowledgment.
  Data2Timeout = 8,

  /// The first byte of the checksum was wrong.
  Checksum1Wrong = 9,

  /// The second byte of the checksum was wrong.
  Checksum2Wrong = 10,

  /// The offset byte returned by an EEPROM Read or RAM Read command was wrong.
  ReadOffsetWrong = 16,

  /// The length byte returned by an EEPROM Read or RAM Read command was wrong.
  ReadLengthWrong = 17,
};

/// This struct represents the data returned by a STAT command.
struct XYZrobotServoStatus
{
  XYZrobotServoStatusError statusError;
  XYZrobotServoStatusDetail statusDetail;
  uint16_t pwm;     // The torque applied to motor
  uint16_t posRef;  /// Servo position goal. If no goal, this is just its current measured position.
  uint16_t position;  // Servo current position
  uint16_t iBus;      // ：The Current applied to motor. The Value is 200 times the actual current.
} __attribute__((packed));

struct IJogData
{
  uint16_t goal;
  uint8_t type;
  uint8_t id;
  uint8_t playtime;  // play time may be modified for a long movement
} __attribute__((packed));

// I-JOG - independent control move
// Stops once the goal is reached, even if it requires more time then specified in playtime
template <size_t ServoCount>
struct IJogCommand
{
  std::array<IJogData, ServoCount> data;
};

class ServoProtocol
{
public:
  static constexpr uint8_t kBroadcastId = 0xFE;

  virtual ~ServoProtocol() = default;

  virtual XYZrobotServoStatus readStatus() = 0;
  virtual bool isFailed() const = 0;
  virtual void reboot() = 0;
  virtual XYZrobotServoError getLastError() const = 0;

  virtual void sendJogCommand(const std::vector<IJogData>& cmd) = 0;
};
