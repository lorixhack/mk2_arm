#!/usr/bin/env python3

import traceback
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import serial
import struct

id = {
    "pitch_joint_1": 0x01,
    "roll_joint_2": 0x02,
    "pitch_joint_3": 0x03,
    "roll_joint_4": 0x04,
    "pitch_joint_5": 0x05,
    "roll_joint_6": 0x06
}

class JointPublisherSerial(Node):
    def __init__(self):
        super().__init__("joint_publisher_serial")
        self.get_logger().info("joint_publisher_serial node started")

        self.serial = serial.Serial(port='/dev/ttyACM0', baudrate=115200)
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

        time.sleep(5)

        self.joint_states = {
            "pitch_joint_1": 0.0,
            "roll_joint_2": 0.0,
            "pitch_joint_3": 0.0,
            "roll_joint_4": 0.0,
            "pitch_joint_5": 0.0,
            "roll_joint_6": 0.0
        }

        self.joint_dict = dict()

        self.create_subscription(
            JointState,
            '/joint_states',
            self.send_joint_states,
            10,
        )

    def send_joint_states(self, msg:JointState):
        # self.get_logger().info("Joint positions:")
        send = False
        self.joint_dict.clear()
        for i in range(0, len(msg.name)):
            if msg.position[i] != self.joint_states[msg.name[i]]:
                self.joint_dict[msg.name[i]] = msg.position[i]
                send = True
            self.joint_states[msg.name[i]] = msg.position[i]
        if send:
            # self.send_serial_data()
            self.send_serial_data_alone()


    def send_serial_data(self):
        for joint_name, angle in self.joint_states.items():
            # self.get_logger().info(f"{joint_name} : {angle}")
            packet = struct.pack("<fB", float(angle), id[joint_name])
            # msg = f"{id[joint_name]}{angle}\n"
            # self.get_logger().info(msg)
            self.serial.write(packet)
            # self.get_logger().info(f"{n} bytes written on serial port")
            # self.read_serial_data()
    
    def send_serial_data_alone(self):
        for joint_name, angle in self.joint_dict.items():
            # self.get_logger().info(f"{joint_name} : {angle}")
            packet = struct.pack("<fB", float(angle), id[joint_name])
            self.serial.write(packet)
    
    def read_serial_data(self):
        serial_str = self.serial.readline().decode("utf-8").strip()
        self.get_logger().info(serial_str)

        
def main(args=None):
    rclpy.init(args=args)
    try:
        joint_publisher_serial = JointPublisherSerial()
        rclpy.spin(joint_publisher_serial)
    except Exception as err:
        rclpy.logging.get_logger('joint_publisher_serial').fatal(
            f'Error in the $node node: {str(err)}\n{traceback.format_exc()}'
        )
        raise err
    else:
        joint_publisher_serial.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
