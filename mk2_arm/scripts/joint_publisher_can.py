#!/usr/bin/env python3

import traceback
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray
import time


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

        self.pubs = {}
        for id in PACKET_ID:
            if id != "roll_joint_2":
                if id == "pitch_joint_1":
                    self.pubs[id] = self.create_publisher(
                        Float32MultiArray,
                        f'{id}',
                        10,
                    )
                else:
                    self.pubs[id] = self.create_publisher(
                        Float32,
                        f'{id}',
                        10,
                    )

    def send_joint_states(self, msg:JointState):
        update_diff = True
        for i in range(0, len(msg.name)):
            if (msg.position[i] != self.joint_states[msg.name[i]]):
                if (msg.name[i] == "pitch_joint_1" or msg.name[i] == "roll_joint_2"):
                    if update_diff:
                        i0 = self.find_index_by_name(msg.name, "pitch_joint_1")
                        i1 = self.find_index_by_name(msg.name, "roll_joint_2")
                        self.joint_states["pitch_joint_1"] = msg.position[i0]
                        self.joint_states["roll_joint_2"] = msg.position[i1]
                        update_diff = False

                        self.pubs["pitch_joint_1"].publish(Float32MultiArray(data = [msg.position[i0], msg.position[i1]]))

                else:
                    self.joint_states[msg.name[i]] = msg.position[i]

                    self.pubs[msg.name[i]].publish(Float32(data = msg.position[i]))
    
    def find_index_by_name(self, joints, joint_name):
        for i in range(0, len(joints)):
            if joints[i] == joint_name:
                return i

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
