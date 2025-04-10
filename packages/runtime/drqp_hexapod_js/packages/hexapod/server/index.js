const { servoConfig } = require("./servoConfig")
const { SOCKET_SERVER_PORT, SOCKET_CLIENT_URLS, setServoEvent, newKeyEvent } = require("../src/connectionConfig")

const rclnodejs = require("rclnodejs")
const { randomUUID } = require("crypto")
const app = require("express")()
const http = require("http").Server(app)

const io = require("socket.io")(http, {
    cors: {
        origin: SOCKET_CLIENT_URLS,
    },
})

const LEG_POSITIONS = [
    "leftFront",
    "rightFront",
    "leftMiddle",
    "rightMiddle",
    "leftBack",
    "rightBack",
]

http.listen(SOCKET_SERVER_PORT, function () {
    console.log(`listening on *:${SOCKET_SERVER_PORT}`)
    setupRobot()
})

class Servo {
    constructor(config) {
        this.config = config;
        // console.log('config ', config);
    }

    to(IK_value) {
        // values are -90 to 90
        // 0 - is center position 512 on servo
        // negative values are clockwise
        // this.angle = IK_value * this.config.direction;
        const directed = (IK_value + this.config.offsetAngle) * this.config.direction;
        this.angle = (directed + 180) / 360;
    }

    get id() {
        return this.config.id;
    }

    get position() {
        // const ratio = (this.angle + 180) / 360;
        // const position = Math.floor(ratio * 1023);
        const position = Math.floor(this.angle * 1023);

        // this.angle = Math.max(Math.min(Math.round(directed) + 90, 180), 0);
        // const ratio = (this.angle) / 180;
        // const position = Math.floor(ratio * 1023);
        // console.log('angle=', this.angle, ' == position=', position);
        return position;
    }
}

async function setupRosNode() {
    await rclnodejs.init();
    const node = new rclnodejs.Node('hexa_kinematics');
    const publisher = node.createPublisher('drqp_interfaces/msg/MultiServoPositionGoal', 'servo_goals');

    node.spin();

    return publisher;
}

async function setupRobot()
{
    publisher = await setupRosNode();
    // *************************
    // INITIALIZE SERVOS
    // *************************
    const initServo = (legPosition, angleName) =>
        new Servo(servoConfig[legPosition][angleName])

    let hexapodServos = {}

    for (let leg of LEG_POSITIONS) {
        hexapodServos[leg] = {
            alpha: initServo(leg, "alpha"),
            beta: initServo(leg, "beta"),
            gamma: initServo(leg, "gamma"),
        }

        hexapodServos[leg].alpha.to(90)
        hexapodServos[leg].beta.to(90)
        hexapodServos[leg].gamma.to(90)
    }

    // *************************
    // COMMAND SERVOS
    // *************************
    const setServo = (pose, leg, angle) => {
        const newPose = pose[leg][angle]
        const servo = hexapodServos[leg][angle]
        servo.to(newPose)

        // pose[leg][angle] = [newPose, servo.angle, servo.position] // FOR LOGGING
        return servo
    }

    const setHexapodPose = pose => {
        // console.log("setting pose: ", pose)
        const servos = []
        for (let leg of LEG_POSITIONS) {
            servos.push(setServo(pose, leg, "alpha"));
            servos.push(setServo(pose, leg, "beta"));
            servos.push(setServo(pose, leg, "gamma"));
        }
        // console.log("setting pose: ", pose)

        let poseAsyncMsg = rclnodejs.createMessageObject('drqp_interfaces/msg/MultiServoPositionGoal');

        for (let servo of servos) {
            poseAsyncMsg.goals.push({
                id: servo.id,
                position: servo.position,
                playtime: 5,
            });
        }
        publisher.publish(poseAsyncMsg);
    }

    // *************************
    // LISTEN TO SOCKET
    // *************************
    let clientKey;
    io.on("connection", socket => {
        console.log("client connected.")

        clientKey = randomUUID();
        socket.emit(newKeyEvent, {
          clientKey
        });

        socket.on("disconnect", () => {
            console.log("client disconnected.")
        })

        socket.on(setServoEvent, msg => {
            if (msg.clientKey != clientKey) {
              return;
            }

            if (msg.pose) {
                setHexapodPose(msg.pose)
            }
        })
    })
}
