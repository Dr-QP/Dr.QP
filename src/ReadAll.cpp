#include "servo/XYZrobotServo.h"

#include "servo/TcpSerial.h"
#include "servo/UnixSerial.h"

#include "DrQp.h"

int main() {
  TcpSerial servoSerial("192.168.1.136", 2022);

  for (const auto servoId : servoIdsRange())
  {
    XYZrobotServo servo(servoSerial, servoId);

    XYZrobotServoStatus status = servo.readStatus();
    if (servo.getLastError())
    {
      std::cerr << "Servo: " << servoId << " error reading status: " << servo.getLastError() << "\n";
    }
    std::cout << "Servo: " << servoId << " position: " << (int)status.position << "\n";
  }

  return 0;
}