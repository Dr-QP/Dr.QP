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

#pragma once

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <chrono>

#include "drqp_serial/Stream.h"

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

std::string to_string(XYZrobotServoError errorCode);

template <class charT, class charTraitsT>
static inline std::basic_ostream<charT, charTraitsT>& operator<<(
  std::basic_ostream<charT, charTraitsT>& out, XYZrobotServoError errorCode)
{
  return out << to_string(errorCode);
}

enum class XYZrobotServoBaudRate : uint8_t {
  xyzB9600 = 0x01,
  xyzB19200 = 0x02,
  xyzB57600 = 0x06,
  xyzB115200 = 0x0C,
};

static inline uint32_t XYZrobotServoBaudRateToInt(XYZrobotServoBaudRate baud)
{
  switch (baud) {
  case XYZrobotServoBaudRate::xyzB9600:
    return 9600;
  case XYZrobotServoBaudRate::xyzB19200:
    return 19200;
  case XYZrobotServoBaudRate::xyzB57600:
    return 57600;
  case XYZrobotServoBaudRate::xyzB115200:
    return 115200;
  default:
    return 0;
  }
}

/// The possible values for the ACK_Policy parameter stored in the servo's
/// EEPROM and RAM.  This parameter determins which commands the servo will send
/// an acknowledgment response for.
enum class XYZrobotServoAckPolicy {
  // The servo only responds to STAT commands.
  OnlyStat = 0,

  // The servo only responds to STAT, EEPROM Read, and RAM Read commands.
  OnlyReadAndStat = 1,

  // The servo responds to all commands.
  All = 2,
};

enum class XYZrobotServoSpdctrlPolicy {
  OpenLoop = 0,
  CloseLoop = 1,
};

template <class DurationT>
uint8_t toPlaytime(const DurationT& duration)
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() / 10;
}

/// This struct represents the data returned by a STAT command.
struct XYZrobotServoStatus
{
  uint8_t statusError;
  uint8_t statusDetail;
  uint16_t pwm;
  uint16_t posRef;  /// Servo position goal. If no goal, this is just its current measured position.
  uint16_t position;  // Servo current position
  uint16_t iBus;
} __attribute__((packed));

struct SJogData
{
  uint16_t goal;
  uint8_t type;
  uint8_t id;
} __attribute__((packed));

// S-JOG - synchronous control move
// Stops when playtime duration is reached, even if goal is not reached in position control
template <size_t ServoCount>
struct SJogCommand
{
  uint8_t playtime;  // goal position may not be reached for a short play time
  std::array<SJogData, ServoCount> data;
};

class DynamicSJogCommand
{
public:
  explicit DynamicSJogCommand(size_t count) : count_(count)
  {
    size_ = count * sizeof(SJogData) + sizeof(uint8_t);
    data_.reset(new uint8_t[size_]);
  }

  void* data() const
  {
    return data_.get();
  }
  size_t size() const
  {
    return size_;
  }

  void setPlaytime(uint8_t playtime)
  {
    *(data_.get()) = playtime;
  }

  SJogData& at(size_t index)
  {
    if (index >= count_) {
      throw std::out_of_range("SJog data index out of rang");
    }

    SJogData* spos = reinterpret_cast<SJogData*>(data_.get() + 1);
    return spos[index];
  }

private:
  std::unique_ptr<uint8_t[]> data_ = nullptr;
  size_t size_ = 0;
  size_t count_;
};

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
class DynamicIJogCommand
{
public:
  explicit DynamicIJogCommand(size_t count) : data_(count) {}

  const IJogData* data() const
  {
    return data_.data();
  }

  size_t size() const
  {
    return data_.size() * sizeof(data_[0]);
  }

  IJogData& at(size_t pos)
  {
    return data_.at(pos);
  }

private:
  std::vector<IJogData> data_;
};

#define SET_POSITION_CONTROL 0
#define SET_SPEED_CONTROL 1
#define SET_TORQUE_OFF 2
#define SET_POSITION_CONTROL_SERVO_ON 3

#define CMD_EEPROM_WRITE 0x01
#define CMD_EEPROM_READ 0x02
#define CMD_RAM_WRITE 0x03
#define CMD_RAM_READ 0x04
#define CMD_I_JOG 0x05
#define CMD_S_JOG 0x06
#define CMD_STAT 0x07
#define CMD_ROLLBACK 0x08
#define CMD_REBOOT 0x09

struct SharedMemData
{
  uint8_t sID;               // Servo ID 1-253
  uint8_t ACK_Policy;        // see XYZrobotServoAckPolicy
  uint8_t Alarm_LED_Policy;  // see writeAlarmLedPolicyRam

  // Shut down Motor when Voltage/Load/Temperature. Torque Free Control: 0, Torque Limited: 1
  uint8_t Torque_Policy;
  uint8_t SPDctrl_Policy;  // Speed open/close loop control. Open loop: 0, Close loop: 1

  // The limit of A1-16 servo operating temperature. The value is in Degrees Celsius
  uint8_t Max_Temperature;
  // The min value of A1-16 servo operating voltage. The value is 16 times the actual voltage
  uint8_t Min_Voltage;

  // The max value of A1-16 servo operating voltage. The value is 16 times the actual voltage
  uint8_t Max_Voltage;

  // 0-50. Note: acceleration_time = deceleration_time = play_time * Acceleration_Ratio/100
  // Play time | Acceleration_Ratio | Reference position trajectory
  // 0         | 0                  | Ramp-to-step position command, see (36)
  // 1-255     | 0                  | Constant speed profile
  // 1-255     | 1-50               | T-curve speed profile
  uint8_t Acceleration_Ratio;

  uint8_t reserved1[3];

  uint16_t Max_Wheel_Ref_Position;  // Start virtual position for speed close loop control.

  uint8_t reserved2[2];

  uint16_t Max_PWM;  // The max value of A1-16 servo output torque

  uint16_t Overload_Threshold;  // The max value of A1-16 servo output torque
  uint16_t Min_Position;        // Min operational angle
  uint16_t Max_Position;        // Max operational angle
  uint16_t Position_Kp;  // The P control law is implemented below with a sampling time of 10 msec
  uint16_t Position_Kd;  // The PD control law is implemented below with a sampling time of 10 msec
  uint16_t Position_Ki;  // The PID control law is implemented below with a sampling time of 10 msec
  uint16_t Close_to_Open_Ref_Position;  // close loop continuous rotate mode close to open position
  uint16_t Open_to_Close_Ref_Position;  // close loop continuous rotate mode open to close position.

  uint8_t reserved3[2];

  uint16_t Ramp_Speed;       // 0 (step position command), 1~1023 (slope of ramp-to-step)
  uint8_t LED_Blink_Period;  // Blinking Period of LED with a sampling time of 10 msec.

  uint8_t reserved4;

  // Packet Timeout Detection Period of
  // LED with a sampling time of 10 msec. 1 = 10ms
  uint8_t Packet_Timeout_Detection_Period;

  uint8_t reserved5;

  // Overload Detection Period of servo with a
  // sampling time of 10 msec. 1 = 10ms
  uint8_t Overload_Detection_Period;

  uint8_t reserved6;

  uint8_t Inposition_Margin;

  // Over Voltage Detection Period of servo
  // with a sampling time of 10 msec. 1 = 10ms
  uint8_t Over_Voltage_Detection_Period;

  // Over Temperature Detection Period of servo
  // with a sampling time of 10 msec. 1 = 10ms
  uint8_t Over_Temperature_Detection_Period;

  // The difference between newtral point and position raw data.
  uint8_t Calibration_Difference;
} __attribute__((packed));

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

struct XYZrobotServoRAM : public SharedMemData
{
  XYZrobotServoStatusError Status_Error;
  XYZrobotServoStatusDetail Status_Detail;

  uint8_t reserved7[3];

  // i = 0 (LEDi off), 1 (LEDi on); (see Alarm_LED_Policy)
  // Bit 0：White LED
  // Bit 1：Blue LED
  // Bit 2：Green LED
  // Bit 3：Red LED
  uint8_t LED_Control;

  // The voltage currently applied to servo. The Value is 16 times the actual voltage.
  uint8_t Voltage;

  // The internal temperature of motor in Degrees Celsius.
  uint8_t Temperature;

  // 0 (position control), 1 (speed control), 2 (torque off)
  uint8_t Current_Control_Mode;

  // Time servo operation. 1 = 10ms
  uint8_t Tick;

  uint8_t reserved8[2];

  // Servo Position
  uint16_t Joint_Position;

  uint8_t reserved9[2];

  // The torque applied to motor
  uint16_t PWM_Output_Duty;

  // The Current applied to motor. The Value is 200 times the actual current.
  uint16_t Bus_Current;

  // Servo goal of position control mode
  uint16_t Position_Goal;

  // Ref point for position control
  uint16_t Position_Ref;

  // Goal speed of speed close-loop control
  uint16_t Omega_Goal;

  // Ref speed of speed close-loop control
  uint16_t Omega_Ref;

  // Total # of requested packets received since power on.
  uint16_t Requested_Counts;

  // Total # of ACK packets send since power on.
  uint16_t ACK_Counts;
} __attribute__((packed));

struct EEPROM_header
{
  // Servo model name
  uint8_t Model_Number;

  // Year
  uint8_t Year;

  // bit 0-3: month; bit 4-7: version of servo firmware
  uint8_t Version_Month;

  // Day
  uint8_t Day;

  uint8_t reserved1;

  // Baud rate
  XYZrobotServoBaudRate Baud_Rate;
} __attribute__((packed));

struct XYZrobotServoEEPROM : public EEPROM_header, public SharedMemData
{
} __attribute__((packed));

class XYZrobotServo
{
public:
  static constexpr uint8_t kBroadcastId = 0xFE;

  XYZrobotServo(Stream&, uint8_t id);

  /// Writes data from the specified buffer to the servo's EEPROM.
  ///
  /// After running this command, we recommend delaying for 10 ms per data byte
  /// before sending the next command to this servo, since writing to EEPROM
  /// takes some time and the servo cannot receive more commands until it is
  /// done.
  void eepromWrite(uint8_t startAddress, const void* data, uint8_t dataSize);

  /// Reads data from the servo's EEPROM and stores it in the specified buffer.
  ///
  /// The data size should be 35 or less: otherwise the A1-16 seems to return a
  /// response with an invalid CRC.
  void eepromRead(uint8_t startAddress, void* data, uint8_t dataSize);

  /// Writes data from the specified buffer to the servo's RAM.
  void ramWrite(uint8_t startAddress, const void* data, uint8_t dataSize);

  /// Reads data from the servo's RAM and stores it in the specified buffer.
  ///
  /// The data size should be 35 or less: otherwise the A1-16 seems to return a
  /// response with an invalid CRC.
  void ramRead(uint8_t startAddress, void* data, uint8_t dataSize);

  /// Write the Baud_Rate parameter byte in EEPROM, which determines which baud
  /// rate the servo uses on its serial interface.
  ///
  /// After running this command, we recommend delaying for 10 ms before sending
  /// the next command to this servo, since writing to EEPROM takes some time
  /// and the servo cannot receive more commands until it is done.
  void writeBaudRateEeprom(XYZrobotServoBaudRate baudRate);

  /// Reads the Baud_Rate parameter byte in EEPROM, which determines which baud
  /// rate the servo uses on its serial interface.
  XYZrobotServoBaudRate readBaudRateEeprom();

  /// Write the sID parameter byte in EEPROM, which determines which ID the
  /// servo uses on its serial interface.
  ///
  /// After running this command, we recommend delaying for 10 ms before sending
  /// the next command to this servo, since writing to EEPROM takes some time
  /// and the servo cannot receive more commands until it is done.
  void writeIdEeprom(uint8_t id);

  /// Reads the sID parameter byte in EEPROM, which determines which ID the
  /// servo uses on its serial interface.
  uint8_t readIdEeprom();

  /// Write the sID parameter byte in RAM, which determines which ID the
  /// servo uses on its serial interface.
  ///
  /// Write the ACK_Policy parameter byte in RAM.
  void writeIdRam(uint8_t id);

  /// Write the ACK_Policy parameter byte in EEPROM.
  ///
  /// After running this command, we recommend delaying for 10 ms before sending
  /// the next command to this servo, since writing to EEPROM takes some time
  /// and the servo cannot receive more commands until it is done.
  void writeAckPolicyEeprom(XYZrobotServoAckPolicy);

  /// Read the ACK_Policy parameter byte in EEPROM.
  XYZrobotServoAckPolicy readAckPolicyEeprom();

  /// Write the ACK_Policy parameter byte in RAM.
  void writeAckPolicyRam(XYZrobotServoAckPolicy);

  /// Write the Alarm_LED_Policy byte in RAM.  This controls which LEDs on the
  /// servo are controlled by the user and which are controlled by the system.
  ///
  /// A 0 bit means the LED is controlled by the system, and a 1 bit means the
  /// LED is controlled by the user.
  ///
  /// - Bit 0: White LED
  /// - Bit 1: Blue LED
  /// - Bit 2: Green LED
  /// - Bit 3: Red LED
  ///
  /// To control user LEDs, see writeLedControl().
  void writeAlarmLedPolicyRam(uint8_t);

  /// Sets the SPDctrl_Policy variable in RAM.
  void writeSpdctrlPolicyRam(XYZrobotServoSpdctrlPolicy policy);

  /// Sets the maximum PWM value in RAM.
  ///
  /// This should be a number between 0 and 1023 that indicates how strong the
  /// servo should be allowed to drive its motor, with 1023 corresponding to
  /// 100%.
  void writeMaxPwmRam(uint16_t value);
  uint16_t readMaxPwmRam();

  enum : uint8_t {
    kWhiteLedBit = 1,
    kBlueLedBit = 1 << 1,
    kGreenLedBit = 1 << 2,
    kRedLedBit = 1 << 3,
  };
  /// After calling writeAlarmLedPolicyRam(), you can use this to control any
  /// LEDs that are configured as user LED.
  ///
  /// - Bit 0: White LED
  /// - Bit 1: Blue LED
  /// - Bit 2: Green LED
  /// - Bit 3: Red LED
  void writeLedControl(uint8_t);

  /// Read the ACK_Policy parameter byte in RAM.
  XYZrobotServoAckPolicy readAckPolicyRam();

  /// Sends a STAT command to the servo and returns the results.
  XYZrobotServoStatus readStatus();

  /// Sends an I-JOG command to set the target/goal position for this servo.
  ///
  /// The position should be a number between 0 and 1023.
  ///
  /// The playtime should the desired time for the movement to take, in units of
  /// 10 ms.  For example, a playtime of 50 would correspond to 500 ms or 0.5
  /// seconds.
  void setPosition(uint16_t position, uint8_t playtime = 0);

  /// Sends an I-JOG command to set the speed for this servo.
  ///
  /// The speed should be a number between -1023 and 1023.
  ///
  /// A value of 0 causes an abrupt stop.  Non-zero values generally cause the
  /// servo to smoothly ramp its speed up or down to the specified value.
  ///
  /// The playtime specifies how long this command will last, in units of 10 ms.
  /// A value of 0 makes it last indefinitely.
  ///
  /// See writeSpeedControlPolicyRam().
  void setSpeed(int16_t speed, uint8_t playtime = 0);

  /// Sends an I-JOG command to turn off the servo's motor.
  ///
  /// Note that this command interacts badly with the A1-16 servo's speed
  /// ramping feature.  If you are in speed control mode and then send this
  /// command, the servo will still remember what speed it was using before it
  /// received the torqueOff() command.  If you later send a setSpeed() command
  /// with a non-zero speed, it will ramp up or down to that speed, starting
  /// from the remembered speed instead of starting from zero.  If you encounter
  /// this issue, you can call `setSpeed(0)` immediately before the next
  /// non-zero setSpeed() command to solve it.
  void torqueOff();

  /// Sends an I-JOG command to turn on the servo's motor in position control.
  ///
  /// Use this command to smoothly recover from torqueOff() call
  void torqueOn();

  // Resets all parameters in EEPROM to their default values.
  //
  // After running this command, we recommend delaying for 2500 ms before
  // sending the next command to this servo, since it takes the servo a while to
  // change its parameters.
  void rollback();

  // Resets the servo.
  //
  // After running this command, we recommend delaying for 2500 ms before
  // sending the next command to this servo, since it takes the servo a while to
  // restart.
  void reboot();

  /// Returns the communication error from the last command.  The return value
  /// will be 0 if there was no error and non-zero if there was an error.  The
  /// return value will be one of the values of the XYZrobotServoError enum.
  XYZrobotServoError getLastError() const
  {
    return lastError;
  }
  bool isOk() const
  {
    return getLastError() == XYZrobotServoError::None;
  }
  bool isFailed() const
  {
    return getLastError() != XYZrobotServoError::None;
  }

  /// Get the servo ID assigned to this object.
  uint8_t getId() const
  {
    return id;
  }

  template <size_t ServoCount>
  void sendJogCommand(const SJogCommand<ServoCount>& cmd)
  {
    sendRequest(kBroadcastId, CMD_S_JOG, &cmd, sizeof(cmd));
  }

  void sendJogCommand(const DynamicSJogCommand& cmd)
  {
    sendRequest(kBroadcastId, CMD_S_JOG, cmd.data(), cmd.size());
  }

  template <size_t ServoCount>
  void sendJogCommand(const IJogCommand<ServoCount>& cmd)
  {
    sendRequest(kBroadcastId, CMD_I_JOG, &cmd, sizeof(cmd));
  }

  void sendJogCommand(const DynamicIJogCommand& cmd)
  {
    sendRequest(kBroadcastId, CMD_I_JOG, cmd.data(), cmd.size());
  }

private:
  void flushRead();

  void sendRequest(
    uint8_t headerId, uint8_t cmd, const void* data1, uint8_t data1Size,
    const void* data2 = nullptr, uint8_t data2Size = 0);

  void readAck(
    uint8_t cmd, void* data1, uint8_t data1Size, void* data2 = nullptr, uint8_t data2Size = 0);

  void memoryWrite(uint8_t cmd, uint8_t startAddress, const void* data, uint8_t dataSize);

  void memoryRead(uint8_t cmd, uint8_t startAddress, void* data, uint8_t dataSize);

  void sendIJog(IJogData data);

  XYZrobotServoError lastError;

  uint8_t id;

  Stream* stream;
};
