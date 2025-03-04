#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterValue
from time import sleep, time
from math import pi as Pi

def Create_msg(v):
    """Создает сообщение типа ParameterValue, на вход получает скорость на колесоо"""
    param_value_msg = ParameterValue()
    param_value_msg.type = 3  # double тип
    param_value_msg.double_value = v
    return param_value_msg

def TicPerSec(v):
    """Принимает на вход скорость в метрах в секунду и переводит ее в тики в секунду"""
    D = 0.25 #диаметр колеса в метрах
    # 210 - колиечство тиков на оборот
    return v * (210/(Pi*D))


class Driver_node(Node):
    def __init__(self):
        super().__init__("Driver")
        self.fl_publisher_ = self.create_publisher(ParameterValue, '/fl_cmd_vel', 10)
        self.fr_publisher_ = self.create_publisher(ParameterValue, '/fr_cmd_vel', 10)
        self.bl_publisher_ = self.create_publisher(ParameterValue, '/bl_cmd_vel', 10)
        self.br_publisher_ = self.create_publisher(ParameterValue, '/br_cmd_vel', 10)

    def spin(self):
        start_time = time()
        while True:

            example_speed = TicPerSec(0.785) # метров в секунду

            fl_msg = Create_msg(example_speed)
            fr_msg = Create_msg(example_speed)
            bl_msg = Create_msg(example_speed)
            br_msg = Create_msg(example_speed)  # двигаться вперед значения с минусом

            self.fl_publisher_.publish(fl_msg)
            self.get_logger().info('fl_msg published')
            self.fr_publisher_.publish(fr_msg)
            self.get_logger().info('fl_msg published')
            self.bl_publisher_.publish(bl_msg)
            self.get_logger().info('bl_msg published')
            self.br_publisher_.publish(br_msg)
            self.get_logger().info('br_msg published')
            self.get_logger().info("-------------------------")
            sleep(0.01)

            end_time = time()
            if end_time - start_time >= 5.0:
                break





def main(args=None):
    rclpy.init(args=args)
    node = Driver_node()
    node.spin()
    rclpy.shutdown()


# Запуск с терминала (перед сделать source ~/.bashrc)
# ros2 run protocol_stm32 driver
if __name__ == '__main__':
    main()