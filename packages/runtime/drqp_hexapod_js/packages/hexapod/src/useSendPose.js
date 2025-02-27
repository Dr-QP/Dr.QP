import { useState, useEffect, useCallback, useRef } from "react"
import socketIOClient from "socket.io-client"
import {
    SOCKET_SERVER_URL,
    MINIMUM_TIME_BETWEEN_MESSAGES,
    CLIENT_SENDER_NAME,
    setServoEvent,
    newKeyEvent,
} from "./connectionConfig"

const useSendPose = () => {
    const socketRef = useRef()
    const clientKeyRef = useRef()
    const [lastDate, setLastDate] = useState(() => new Date())
    // delta date is the interval between the last two messages sent
    const [deltaDate, setDeltaDate] = useState(0)

    useEffect(() => {
        socketRef.current = socketIOClient(SOCKET_SERVER_URL)

        socketRef.current.on(newKeyEvent, msg => {
          clientKeyRef.current = msg.clientKey
        });
        return () => socketRef.current.disconnect()
    }, [])

    const sendPose = useCallback(
        pose => {
            const currentDate = new Date()
            const newDeltaDate = currentDate - lastDate

            // we shouldn't spam the robot with commands
            if (newDeltaDate < MINIMUM_TIME_BETWEEN_MESSAGES) {
                return
            }

            socketRef.current.emit(setServoEvent, {
                pose,
                sender: CLIENT_SENDER_NAME,
                clientKey: clientKeyRef.current,
                time: currentDate.getTime(),
            })

            setLastDate(currentDate)
            setDeltaDate(newDeltaDate)
        },
        [lastDate]
    )

    return [sendPose, deltaDate]
}

export default useSendPose
