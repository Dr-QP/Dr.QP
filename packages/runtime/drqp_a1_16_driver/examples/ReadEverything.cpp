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

// This example reads all the information from a smart servo and
// prints it to the serial monitor.

#include <signal.h>

#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <thread>
#include <type_traits>

#include "drqp_a1_16_driver/XYZrobotServo.h"
// #include "drqp_serial/TcpSerial.h"
#include "drqp_serial/UnixSerial.h"

// #define servoSerial Serial1

const uint8_t kServoId = 5;

// UnixSerial servoSerial("/dev/cu.SLAB_USBtoUART");
UnixSerial servoSerial("/dev/ttySC0");  // on Dr.QP raspi
// UnixSerial servoSerial("/dev/ttySC1"); // extra one on Dr.QP raspi

// https://techtinkering.com/2013/04/02/connecting-to-a-remote-serial-port-over-tcpip/
// connection: &con00
//   accepter: tcp,2022
//   connector: serialdev,/dev/ttySC0,115200n81,local
//   trace-both: '/var/log/trace-\p'
// TcpSerial servoSerial("192.168.1.136", 2022);

// OR
// socat pty,link=$HOME/dev/ttyVSC0,waitslave tcp:192.168.1.136:2022
// UnixSerial servoSerial("/Users/antonmatosov/dev/ttyVSC0"); // virtual port
// forward on macOS

XYZrobotServo servo(servoSerial, kServoId);

void setup()
{
  // Serial.begin(115200); // console output

  // 115200 8N1 => 11520 B/s / 24 frames = 480 Bytes Per Frame / 18 servos = 26
  // bytes per servo per frame

  // Turn on the serial port and set its baud rate.
  servoSerial.begin(115200);
  // servoSerial.setTimeout(20);

  // To receive data, a pull-up is needed on the RX line because
  // the servos do not pull the line high while idle.
  // pinMode(DDD2, INPUT_PULLUP);
}

struct CustomStatus48
{
  uint8_t statusError;
  uint8_t statusDetail;
  uint8_t reserved[3];
  uint8_t ledControl;
  uint8_t voltage;
  uint8_t temperature;
  uint8_t currentControlMode;
  uint8_t tick;  // 8
  uint16_t reserved2;
  uint16_t jointPosition;
  uint16_t reserved3;
  uint16_t pwmOutputDuty;
  uint16_t busCurrent;
  uint16_t posGoal;
  uint16_t posRef;
  uint16_t speedGoal;
  uint16_t speedRef;  // 18 + 8 = 26
} __attribute__((packed));

void readAndPrintStatus(XYZrobotServo& servo)
{
  XYZrobotServoStatus status = servo.readStatus();
  if (servo.isFailed()) {
    std::cout << "error reading status: " << servo.getLastError() << std::endl;
  } else {
    std::cout << "status:\n";
    std::cout << "  statusError: 0x";
    std::cout << std::hex << static_cast<int>(status.statusError) << "\n";
    std::cout << "  statusDetail: 0x";
    std::cout << std::hex << static_cast<int>(status.statusDetail) << "\n";
    std::cout << "  pwm: ";
    std::cout << std::dec << status.pwm << "\n";
    std::cout << "  posRef: ";
    std::cout << std::dec << status.posRef << "\n";
    std::cout << "  position: ";
    std::cout << std::dec << status.position << "\n";
    std::cout << "  iBus: ";
    std::cout << std::dec << status.iBus << "\n";
  }
}

CustomStatus48 readCustomStatus(XYZrobotServo& servo)
{
  CustomStatus48 status;
  const size_t statusSize = sizeof(CustomStatus48);
  servo.ramRead(48, reinterpret_cast<uint8_t*>(&status), statusSize);
  if (servo.isFailed()) {
    std::cout << "error reading custom status: " << servo.getLastError() << std::endl;
  } else {
    std::cout << "status:\n";
    std::cout << "  statusError: 0x" << std::hex << std::dec << status.statusError << "\n";
    std::cout << "  statusDetail: 0x" << std::hex << std::dec << status.statusDetail << "\n";
    std::cout << "  ledControl: " << std::dec << status.ledControl << "\n";
    std::cout << "  voltage: " << std::dec << status.voltage / 16 << "V\n";
    std::cout << "  temperature: " << std::dec << status.temperature << "˚C\n";
    std::cout << "  currentControlMode: " << std::dec << status.currentControlMode << "\n";
    std::cout << "  tick: " << std::dec << status.tick * 10 << "ms\n";
    std::cout << "  jointPosition: " << std::dec << status.jointPosition << "\n";
    std::cout << "  pwmOutputDuty: " << std::dec << status.pwmOutputDuty << "\n";
    std::cout << "  busCurrent: " << static_cast<float>(status.busCurrent) / 200.f << "A\n";
    std::cout << "  posGoal: " << std::dec << status.posGoal << "\n";
    std::cout << "  posRef: " << std::dec << status.posRef << "\n";
    std::cout << "  speedGoal: " << std::dec << status.speedGoal << "\n";
    std::cout << "  speedRef: " << std::dec << status.speedRef << "\n";
  }

  return status;
}

void readAndPrintRAM(XYZrobotServo& servo)
{
  XYZrobotServoRAM ram;
  servo.ramRead(0, &ram, sizeof(ram));
  if (servo.isFailed()) {
    std::cout << "error reading RAM: " << servo.getLastError() << "\n";
  } else {
    std::cout << "RAM:\n";
    std::cout << "  sID: " << std::dec << static_cast<int>(ram.sID) << "\n";
    std::cout << "  ACK_Policy: " << std::dec << static_cast<int>(ram.ACK_Policy) << "\n";
    std::cout << "  Alarm_LED_Policy: " << std::dec << static_cast<int>(ram.Alarm_LED_Policy) << "\n";
    std::cout << "  Torque_Policy: " << std::dec << static_cast<int>(ram.Torque_Policy) << "\n";
    std::cout << "  SPDctrl_Policy: " << std::dec << static_cast<int>(ram.SPDctrl_Policy) << "\n";
    std::cout << "  Max_Temperature: " << std::dec << static_cast<int>(ram.Max_Temperature) << "\n";
    std::cout << "  Min_Voltage: " << std::dec << ram.Min_Voltage / 16.0 << "V\n";
    std::cout << "  Max_Voltage: " << std::dec << ram.Max_Voltage / 16.0 << "V\n";
    std::cout << "  Acceleration_Ratio: " << std::dec << static_cast<int>(ram.Acceleration_Ratio) << "\n";
    std::cout << "  Max_Wheel_Ref_Position: " << std::dec << ram.Max_Wheel_Ref_Position << "\n";
    std::cout << "  Max_PWM: " << std::dec << ram.Max_PWM << "\n";
    std::cout << "  Overload_Threshold: " << std::dec << ram.Overload_Threshold << "\n";
    std::cout << "  Min_Position: " << std::dec << ram.Min_Position << "\n";
    std::cout << "  Max_Position: " << std::dec << ram.Max_Position << "\n";
    std::cout << "  Position_Kp: " << std::dec << ram.Position_Kp << "\n";
    std::cout << "  Position_Kd: " << std::dec << ram.Position_Kd << "\n";
    std::cout << "  Position_Ki: " << std::dec << ram.Position_Ki << "\n";
    std::cout << "  Close_to_Open_Ref_Position: " << std::dec << ram.Close_to_Open_Ref_Position << "\n";
    std::cout << "  Open_to_Close_Ref_Position: " << std::dec << ram.Open_to_Close_Ref_Position << "\n";
    std::cout << "  Ramp_Speed: " << std::dec << ram.Ramp_Speed << "\n";
    std::cout << "  LED_Blink_Period: " << std::dec << static_cast<int>(ram.LED_Blink_Period) << "\n";
    std::cout << "  Packet_Timeout_Detection_Period: " << std::dec << static_cast<int>(ram.Packet_Timeout_Detection_Period) << "\n";
    std::cout << "  Overload_Detection_Period: " << std::dec << static_cast<int>(ram.Overload_Detection_Period) << "\n";
    std::cout << "  Inposition_Margin: " << std::dec << static_cast<int>(ram.Inposition_Margin) << "\n";
    std::cout << "  Over_Voltage_Detection_Period: " << std::dec << static_cast<int>(ram.Over_Voltage_Detection_Period) << "\n";
    std::cout << "  Over_Temperature_Detection_Period: " << std::dec << static_cast<int>(ram.Over_Temperature_Detection_Period) << "\n";
    std::cout << "  Calibration_Difference: " << std::dec << static_cast<int>(ram.Calibration_Difference) << "\n";
    std::cout << "  Status_Error: " << std::dec << static_cast<int>(ram.Status_Error) << "\n";
    std::cout << "  Status_Detail: " << std::dec << static_cast<int>(ram.Status_Detail) << "\n";
    std::cout << "  LED_Control: " << std::dec << static_cast<int>(ram.LED_Control) << "\n";
    std::cout << "  Voltage: " << std::dec << ram.Voltage / 16.0 << "V\n";
    std::cout << "  Temperature: " << std::dec << static_cast<int>(ram.Temperature) << "˚C\n";
    std::cout << "  Current_Control_Mode: " << std::dec << static_cast<int>(ram.Current_Control_Mode) << "\n";
    std::cout << "  Tick: " << std::dec << static_cast<int>(ram.Tick) * 10 << "ms\n";
    std::cout << "  Joint_Position: " << std::dec << ram.Joint_Position << "\n";
    std::cout << "  PWM_Output_Duty: " << std::dec << ram.PWM_Output_Duty << "\n";
    std::cout << "  Bus_Current: " << std::dec << ram.Bus_Current / 200.0 << "A\n";
    std::cout << "  Position_Goal: " << std::dec << ram.Position_Goal << "\n";
    std::cout << "  Position_Ref: " << std::dec << ram.Position_Ref << "\n";
    std::cout << "  Omega_Goal: " << std::dec << ram.Omega_Goal << "\n";
    std::cout << "  Omega_Ref: " << std::dec << ram.Omega_Ref << "\n";
    std::cout << "  Requested_Counts: " << std::dec << ram.Requested_Counts << "\n";
    std::cout << "  ACK_Counts: " << std::dec << ram.ACK_Counts << "\n";
  }
}

void readAndPrintEEPROM(XYZrobotServo& servo)
{
  XYZrobotServoEEPROM eeprom;
  servo.eepromRead(0, &eeprom, sizeof(eeprom));
  if (servo.isFailed()) {
    std::cout << "error reading EEPROM: " << servo.getLastError() << "\n";
  } else {
    std::cout << "EEPROM:\n";
    std::cout << "  Model_No: " << std::dec << static_cast<int>(eeprom.Model_Number) << "\n";
    std::cout << "  Date: " << std::dec << static_cast<int>(eeprom.Year) << "-"
              << static_cast<int>(eeprom.Version_Month & 0xF) << "-"
              << static_cast<int>(eeprom.Day) << "\n";
    std::cout << "  Firmware_Version: " << std::dec << ((eeprom.Version_Month >> 4) & 0xF) << "\n";
    std::cout << "  Baud_Rate: " << XYZrobotServoBaudRateToInt(eeprom.Baud_Rate) << "\n";
    std::cout << "  sID: " << std::dec << static_cast<int>(eeprom.sID) << "\n";
    std::cout << "  ACK_Policy: " << std::dec << static_cast<int>(eeprom.ACK_Policy) << "\n";
    std::cout << "  Alarm_LED_Policy: " << std::dec << static_cast<int>(eeprom.Alarm_LED_Policy) << "\n";
    std::cout << "  Torque_Policy: " << std::dec << static_cast<int>(eeprom.Torque_Policy) << "\n";
    std::cout << "  SPDctrl_Policy: " << std::dec << static_cast<int>(eeprom.SPDctrl_Policy) << "\n";
    std::cout << "  Max_Temperature: " << std::dec << static_cast<int>(eeprom.Max_Temperature) << "\n";
    std::cout << "  Min_Voltage: " << std::dec << eeprom.Min_Voltage / 16.0 << "V\n";
    std::cout << "  Max_Voltage: " << std::dec << eeprom.Max_Voltage / 16.0 << "V\n";
    std::cout << "  Acceleration_Ratio: " << std::dec << static_cast<int>(eeprom.Acceleration_Ratio) << "\n";
    std::cout << "  Max_Wheel_Ref_Position: " << std::dec << eeprom.Max_Wheel_Ref_Position << "\n";
    std::cout << "  Max_PWM: " << std::dec << eeprom.Max_PWM << "\n";
    std::cout << "  Overload_Threshold: " << std::dec << eeprom.Overload_Threshold << "\n";
    std::cout << "  Min_Position: " << std::dec << eeprom.Min_Position << "\n";
    std::cout << "  Max_Position: " << std::dec << eeprom.Max_Position << "\n";
    std::cout << "  Position_Kp: " << std::dec << eeprom.Position_Kp << "\n";
    std::cout << "  Position_Kd: " << std::dec << eeprom.Position_Kd << "\n";
    std::cout << "  Position_Ki: " << std::dec << eeprom.Position_Ki << "\n";
    std::cout << "  Close_to_Open_Ref_Position: " << std::dec << eeprom.Close_to_Open_Ref_Position << "\n";
    std::cout << "  Open_to_Close_Ref_Position: " << std::dec << eeprom.Open_to_Close_Ref_Position << "\n";
    std::cout << "  Ramp_Speed: " << std::dec << eeprom.Ramp_Speed << "\n";
    std::cout << "  LED_Blink_Period: " << std::dec << static_cast<int>(eeprom.LED_Blink_Period) << "\n";
    std::cout << "  Packet_Timeout_Detection_Period: " << std::dec << static_cast<int>(eeprom.Packet_Timeout_Detection_Period) << "\n";
    std::cout << "  Overload_Detection_Period: " << std::dec << static_cast<int>(eeprom.Overload_Detection_Period) << "\n";
    std::cout << "  Inposition_Margin: " << std::dec << static_cast<int>(eeprom.Inposition_Margin) << "\n";
    std::cout << "  Over_Voltage_Detection_Period: " << std::dec << static_cast<int>(eeprom.Over_Voltage_Detection_Period) << "\n";
    std::cout << "  Over_Temperature_Detection_Period: " << std::dec << static_cast<int>(eeprom.Over_Temperature_Detection_Period) << "\n";
    std::cout << "  Calibration_Difference: " << std::dec << static_cast<int>(eeprom.Calibration_Difference) << "\n";
  }
}

void readEverything(XYZrobotServo& servo)
{
  // readAndPrintStatus(servo);
  // readAndPrintRAM(servo);
  // readAndPrintEEPROM(servo);
  readCustomStatus(servo);

  std::cout << "\n";
}

void readLoop()
{
  using namespace std::chrono_literals;

  std::this_thread::sleep_for(1s);
  readEverything(servo);
}

const int MinPosition = 250;
const int MaxPosition = 830;

int pos = (MaxPosition - MinPosition) / 2;
const int playtime = 200;
int stepSize = (MaxPosition - MinPosition) / 10;

void setPos()
{
  using namespace std::chrono_literals;

  // readCustomStatus(servo);
  // readAndPrintStatus(servo);

  // XYZrobotServoStatus status = servo.readStatus();
  // std::cout << "Current pos " << status.position << "\n";

  // int pos = -1;
  // if ((status.position + 3) >= MaxPosition) {
  //   pos = MinPosition;
  //   // readCustomStatus(servo);
  // } else if ((status.position - 3) <= MinPosition) {
  //   pos = MaxPosition;
  //   // readCustomStatus(servo);
  // }
  namespace chrono = std::chrono;

  auto startTime = std::chrono::high_resolution_clock::now();
  (void)servo.readStatus();
  auto endTime = std::chrono::high_resolution_clock::now();
  std::cout << "servo.readStatus takes: "
            << chrono::duration_cast<chrono::microseconds>(endTime - startTime).count()
            << " microseconds\n";

  if (servo.isFailed()) {
    std::cout << "error reading status: " << servo.getLastError() << std::endl;
  }

  if (pos == MaxPosition) {
    pos = MinPosition;
    // } else if (pos == MinPosition) {
  } else {
    pos = MaxPosition;
  }

  if (pos != -1) {
    std::cout << "Setting pos " << pos << "\n";
    servo.setPosition(pos, 0);  // playtime, unit 10ms
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(playtime));
}

uint8_t randomByte()
{
  static unsigned int seed = time(nullptr);
  return rand_r(&seed) % 255;
}

void testWrite()
{
  uint8_t byte = randomByte();
  std::cout << "write " << std::dec << byte << "\n";
  servoSerial.writeBytes(&byte, 1);
}

void testRoundtrip()
{
  using namespace std::chrono_literals;

  std::this_thread::sleep_for(500ms);

  uint8_t byte = randomByte();
  std::cout << "write " << std::dec << byte << "\n";
  servoSerial.writeBytes(&byte, 1);

  std::this_thread::sleep_for(10ms);
  servoSerial.readBytes(&byte, 1);
  std::cout << "read " << std::dec << byte << "\n";
}

void signal_callback_handler(int signum)
{
  std::cout << "Caught signal " << signum << std::endl;
  std::exit(signum);
}

int main()
{
  try {
    std::atexit([]() { servo.torqueOff(); });
    signal(SIGINT, signal_callback_handler);
    signal(SIGHUP, signal_callback_handler);

    setup();

    servo.torqueOff();
    if (servo.isFailed()) {
      std::cerr << "Torque OFF failed: " + to_string(servo.getLastError()) << std::endl;
      return 1;
    }
    for (;;) {
      // testRoundtrip();
      // testWrite();
      readLoop();
      // setPos();
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "Unknown exception" << std::endl;
  }
}
