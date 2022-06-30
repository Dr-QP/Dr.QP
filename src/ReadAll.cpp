#include "servo/XYZrobotServo.h"

#include "servo/TcpSerial.h"
#include "servo/UnixSerial.h"

#include "DrQp.h"

#include <iostream>

int main() {
  TcpSerial servoSerial("192.168.1.136", 2022);

  for (const auto &leg : kAllLegServoIds) {
    for (const auto servoId : leg) {
      XYZrobotServo servo(servoSerial, servoId);

      XYZrobotServoStatus status = servo.readStatus();
      if (servo.getLastError()) {
        std::cerr << legNameForServo(servoId) << " servo: " << servoId
                  << " error reading status: " << servo.getLastError() << "\n";
      }
      std::cout << legNameForServo(servoId) << "\tservo: " << servoId
                << ":\t" << (int)status.position << "\n";
    }
    std::cout << "\n";
  }

  return 0;
}