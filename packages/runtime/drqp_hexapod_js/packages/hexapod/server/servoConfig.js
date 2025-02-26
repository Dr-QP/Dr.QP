// See configurable parameters at: http://johnny-five.io/api/servo/

const controller = "PCA9685"
const address = "0x41"

const kDirect = 1;
const kInverse = -1;

const kCoxaOffsetAngle = 0;
const kFemurOffsetAngle = 8;
const kTibiaOffsetAngle = -60;

const servoConfig = {
    rightFront: {
        alpha: { id: 2, direction: kDirect, offsetAngle: kCoxaOffsetAngle },
        beta: { id: 4, direction: kDirect, offsetAngle: kFemurOffsetAngle - 3 /* faulty servo calibration */ },
        gamma: { id: 6, direction: kDirect, offsetAngle: kTibiaOffsetAngle + 12 /* faulty servo calibration */ },
    },

    rightMiddle: {
        alpha: { id: 14, direction: kDirect, offsetAngle: kCoxaOffsetAngle },
        beta: { id: 16, direction: kDirect, offsetAngle: kFemurOffsetAngle },
        gamma: { id: 18, direction: kDirect, offsetAngle: kTibiaOffsetAngle },
    },

    rightBack: {
        alpha: { id: 8, direction: kDirect, offsetAngle: kCoxaOffsetAngle },
        beta: { id: 10, direction: kDirect, offsetAngle: kFemurOffsetAngle },
        gamma: { id: 12, direction: kDirect, offsetAngle: kTibiaOffsetAngle },
    },

    leftFront: {
        alpha: { id: 1, direction: kDirect, offsetAngle: kCoxaOffsetAngle },
        beta: { id: 3, direction: kInverse, offsetAngle: kFemurOffsetAngle },
        gamma: { id: 5, direction: kInverse, offsetAngle: kTibiaOffsetAngle },
    },

    leftMiddle: {
        alpha: { id: 13, direction: kDirect, offsetAngle: kCoxaOffsetAngle },
        beta: { id: 15, direction: kInverse, offsetAngle: kFemurOffsetAngle },
        gamma: { id: 17, direction: kInverse, offsetAngle: kTibiaOffsetAngle },
    },

    leftBack: {
        alpha: { id: 7, direction: kDirect, offsetAngle: kCoxaOffsetAngle },
        beta: { id: 9, direction: kInverse, offsetAngle: kFemurOffsetAngle },
        gamma: { id: 11, direction: kInverse, offsetAngle: kTibiaOffsetAngle },
    },
}

module.exports = { servoConfig }
