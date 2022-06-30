#include "servo/XYZrobotServo.h"

#include "servo/TcpSerial.h"
#include "servo/UnixSerial.h"

#include "DrQp.h"

int main() {
  TcpSerial servoSerial("192.168.1.136", 2022);

  for (const auto servoId : servoIdsRange())
  {
    XYZrobotServo servo(servoSerial, servoId);
    servo.torqueOff();
  }

  return 0;
}