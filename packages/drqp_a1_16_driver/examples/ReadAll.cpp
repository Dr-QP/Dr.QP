#include "servo/XYZrobotServo.h"

#include "servo/TcpSerial.h"
#include "servo/UnixSerial.h"

#include "DrQp.h"

#include <iostream>

int main() {
  TcpSerial servoSerial("192.168.1.136", 2022);

  for (const auto &leg : kAllLegServoIds) {
    int servoIndexInLeg = 0;
    for (const auto servoId : leg) {
      XYZrobotServo servo(servoSerial, servoId);

      XYZrobotServoStatus status = servo.readStatus();
      if (servo.getLastError()) {
        std::cerr << legNameForServo(servoId) << " servo: " << (int)servoId
                  << " error reading status: " << servo.getLastError() << "\n";
      } else {
        std::cout << legNameForServo(servoId) << "\tservo: " << (int)servoId << ": "
                  << (int)status.position << "\t neutral: " << kNeutralPose[kServoIdToLeg[servoId]][servoIndexInLeg] << "\n";
      }
      ++servoIndexInLeg;
    }
    std::cout << "\n";
  }

  return 0;
}