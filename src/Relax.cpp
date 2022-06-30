#include "servo/XYZrobotServo.h"

#include "servo/TcpSerial.h"
#include "servo/UnixSerial.h"

int main() {
  const uint8_t servoId = 5;
  TcpSerial servoSerial("192.168.1.136", 2022);
  XYZrobotServo servo(servoSerial, servoId);

  servo.torqueOff();

  return 0;
}