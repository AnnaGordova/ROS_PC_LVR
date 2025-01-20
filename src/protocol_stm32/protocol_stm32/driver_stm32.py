#!/usr/bin/env python3
import rclpy
import socket
import struct
from rclpy.node import Node
from rcl_interfaces.msg import ParameterValue
from time import sleep


class Drver_stm32_node(Node):
    def __init__(self):
        super().__init__('Drver_stm32')
        self.fl_subscriber = self.create_subscription(ParameterValue, '/fl_cmd_vel', self.fl_callback, 10)
        self.fr_subscriber = self.create_subscription(ParameterValue, '/fr_cmd_vel', self.fr_callback, 10)
        self.bl_subscriber = self.create_subscription(ParameterValue, '/bl_cmd_vel', self.bl_callback, 10)
        self.br_subscriber = self.create_subscription(ParameterValue, '/br_cmd_vel', self.br_callback, 10)

        self.commands_dict = {}

    def fl_callback(self, msg):
        self.commands_dict['fl'] = msg.double_value

    def fr_callback(self, msg):
        self.commands_dict['fr'] = msg.double_value

    def bl_callback(self, msg):
        self.commands_dict['bl'] = msg.double_value

    def br_callback(self, msg):
        self.commands_dict['br'] = msg.double_value
        if len(self.commands_dict) == 4:
            self.send_message()

    def send_message(self):
        self.get_logger().info(f'Commands: {self.commands_dict}')
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client_socket:
            client_socket.connect(('192.168.5.100', 2007))
            self.get_logger().info("Подключение к серверу STM успешно")
            try:
                format_commands = "=ffff"
                data = struct.pack(format_commands, self.commands_dict['fl'], self.commands_dict['fr'], self.commands_dict['bl'], self.commands_dict['br'])
                head_type = bytes.fromhex('B0')
                byte_string = head_type + data
                client_socket.sendall(byte_string)

                self.get_logger().info("Сообщение отпрвлено")
                self.commands_dict = {}
            finally:
                # Закрываем клиентский сокет
                client_socket.close()


def main(args=None):
    rclpy.init(args=args)
    node = Drver_stm32_node()
    rclpy.spin(node)
    rclpy.shutdown()


# Запуск с терминала (перед сделать source ~/.bashrc)
# ros2 run protocol_stm32 driver_stm32
if __name__ == '__main__':
    main()
