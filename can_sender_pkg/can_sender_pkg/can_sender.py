#!/usr/bin/env python3

import traceback
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import time
import can
import struct

PACKET_ID = {
    "pitch_joint_1": 0x51,
    "roll_joint_2": 0x51,
    "pitch_joint_3": 0x53,
    "roll_joint_4": 0x55,
    "pitch_joint_5": 0x57,
    "roll_joint_6": 0x59
}

MODULE_ID = 0X21

class CanSender(Node):
    def __init__(self):
        super().__init__("can_sender")
        self.get_logger().info("can_sender node started")

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

        self.subs = {}
        for id in PACKET_ID:
            if id != "roll_joint_2":
                if id == "pitch_joint_1":
                    self.subs[id] = self.create_subscription(
                        Float32MultiArray,
                        f'{id}',
                        lambda msg, s=id :self.send_can_frames(msg, s), 
                        10,
                    )
                else:
                    self.subs[id] = self.create_subscription(
                        Float32,
                        f'{id}',
                        lambda msg, s=id :self.send_can_frames(msg, s), 
                        10,
                    )

    def send_can_frames(self, msg, topic_name):
        aid = struct.pack('bbbb', 00, PACKET_ID[topic_name], MODULE_ID, 0x00)
        if topic_name == "pitch_joint_1":
            data = struct.pack('ff', msg.data[0], msg.data[1])
        else:
            data = struct.pack('f', msg.data)
        
        can_msg = can.Message(
            arbitration_id=int.from_bytes(aid, byteorder='big', signed='False'),
            data=data,
            is_extended_id=True,
        )

        self.canbus.send(can_msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        can_sender = CanSender()
        rclpy.spin(can_sender)
    except Exception as err:
        rclpy.logging.get_logger('can_sender').fatal(
            f'Error in the $node node: {str(err)}\n{traceback.format_exc()}'
        )
        raise err
    else:
        can_sender.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
