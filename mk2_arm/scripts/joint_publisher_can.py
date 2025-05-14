#!/usr/bin/env python3

import traceback
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import can
import struct
import math

PACKET_ID = {
    "pitch_joint_1": 0x51,
    "roll_joint_2": 0x51,
    "pitch_joint_3": 0x53,
    "roll_joint_4": 0x55,
    "pitch_joint_5": 0x57,
    "roll_joint_6": 0x59
}

MODULE_ID = 0X21

class JointPublisherCan(Node):
    def __init__(self):
        super().__init__("joint_publisher_can")
        self.get_logger().info("joint_publisher_can node started")

        try:
            self.canbus = can.interface.Bus(
                channel='can0',
                bustype='socketcan'
            )
        except OSError as e:
            self.get_logger().fatal(
                f"Error while connecting to CAN bus: {str(e)}\n{traceback.format_exc()}"
            )
            raise

        time.sleep(5)

        self.joint_states = {
            "pitch_joint_1": 0.0,
            "roll_joint_2": 0.0,
            "pitch_joint_3": 0.0,
            "roll_joint_4": 0.0,
            "pitch_joint_5": 0.0,
            "roll_joint_6": 0.0
        }

        self.create_subscription(
            JointState,
            '/joint_states',
            self.send_joint_states,
            10,
        )

    def send_joint_states(self, msg:JointState):
        send_diff = False
        for i in range(0, len(msg.name)):
            if (msg.position[i] != self.joint_states[msg.name[i]]):
                if (msg.name[i] == "pitch_joint_1" or msg.name[i] == "roll_joint_2") and not send_diff:
                    self.joint_states["pitch_joint_1"] = msg.position[0]
                    self.joint_states["roll_joint_2"] = msg.position[1]
                    send_diff = True

                    valueToSend0 = self.scale_angle(msg.position[0])
                    valueToSend1 = self.scale_angle(msg.position[1])

                    theta, phi = self.scale_diff_angles(valueToSend0, valueToSend1)

                    aid = struct.pack('bbbb', 00, PACKET_ID["pitch_joint_1"], MODULE_ID, 0x00)
                    data = struct.pack('ii', theta, phi)

                else:
                    self.joint_states[msg.name[i]] = msg.position[i]
                    valueToSend = self.scale_angle(msg.position[i])

                    aid = struct.pack('bbbb', 00, PACKET_ID[msg.name[i]], MODULE_ID, 0x00)
                    data = struct.pack('i', valueToSend)
                
                can_msg = can.Message(
                    arbitration_id=int.from_bytes(aid, byteorder='big', signed='False'),
                    data=data,
                    is_extended_id=True,
                )

                self.canbus.send(can_msg)
    
    def scale_angle(self, angle: float):
        return int((angle*(4096/(2.0*math.pi))))
        
    def scale_diff_angles(self, theta: float, phi: float):
        return int(-(theta + phi)/2), int((theta - phi)/2)


def main(args=None):
    rclpy.init(args=args)
    try:
        joint_publisher_can = JointPublisherCan()
        rclpy.spin(joint_publisher_can)
    except Exception as err:
        rclpy.logging.get_logger('joint_publisher_can').fatal(
            f'Error in the $node node: {str(err)}\n{traceback.format_exc()}'
        )
        raise err
    else:
        joint_publisher_can.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
