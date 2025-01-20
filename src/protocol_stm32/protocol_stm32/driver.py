#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterValue
from time import sleep

def Create_msg(v):
    param_value_msg = ParameterValue()
    param_value_msg.type = 3  # double тип
    param_value_msg.double_value = v
    return param_value_msg

class Driver_node(Node):
    def __init__(self):
        super().__init__("Driver")
        self.fl_publisher_ = self.create_publisher(ParameterValue, '/fl_cmd_vel', 10)
        self.fr_publisher_ = self.create_publisher(ParameterValue, '/fr_cmd_vel', 10)
        self.bl_publisher_ = self.create_publisher(ParameterValue, '/bl_cmd_vel', 10)
        self.br_publisher_ = self.create_publisher(ParameterValue, '/br_cmd_vel', 10)

    def spin(self):

        while True:
            fl_msg = Create_msg(0.1)
            fr_msg = Create_msg(0.0)
            bl_msg = Create_msg(0.0)
            br_msg = Create_msg(0.0)

            self.fl_publisher_.publish(fl_msg)
            self.get_logger().info('fl_msg published')
            self.fr_publisher_.publish(fr_msg)
            self.get_logger().info('fl_msg published')
            self.bl_publisher_.publish(bl_msg)
            self.get_logger().info('bl_msg published')
            self.br_publisher_.publish(br_msg)
            self.get_logger().info('br_msg published')
            self.get_logger().info("-------------------------")
            sleep(1)



def main(args=None):
    rclpy.init(args=args)
    node = Driver_node()
    node.spin()
    rclpy.shutdown()


# Запуск с терминала (перед сделать source ~/.bashrc)
# ros2 run protocol_stm32 driver
if __name__ == '__main__':
    main()