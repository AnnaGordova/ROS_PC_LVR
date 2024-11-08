#!/usr/bin/env python3
import rclpy
import socket
import struct
from rclpy.node import Node

def Telemetry_Request(sock):
    """
    Возвращает ответ телеметрии в виде байтовой строки
    """
    # Отправляем запрос серверу
    request_message = bytes.fromhex('C0')
    sock.sendall(request_message)
    # Получаем ответ от сервера
    response_message = sock.recv(1024).hex() # 1024?
    return response_message


class Protocol_stm32_node(Node):
    def __init__(self):
        super().__init__("Protocol_stm32_node")
        # self.get_logger().info("Hello from ROS2")
        #self.pub_motors = self.create_publisher("....")
        #self.pub_telem = self.create_publisher("....")
        #self.sub_telem_vel = self.create_publisher("....")
        #self.sub_telem_speed = self.create_publisher("....")
        #self.sub_telem_gps = self.create_publisher("....")
        '''
        H - uint8_t
        f - float
        h - int16_t
        '''

        self.host = ''
        self.port = 2007

        self.addr = '192.168.5.100'

        self.format_type = "=H"
        self.format_heartbeat = "=H"
        self.format_motors = "=Hffff"
        self.format_telemetry_request = "=H"
        self.format_telemetry_answer = "=Hfffffffffhhhhf?fffff"
        self.format_unknown_packet = "=H"  

    def spin(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client_socket:
            client_socket.connect(('192.168.5.100', 2007))
            self.get_logger().info("Подключение к серверу STM успешно")
            try:
                byte_string = Telemetry_Request(client_socket)
                self.get_logger().info("Запрос телеметрии")
                self.get_logger().info("Ответ сервера: " + byte_string)
            finally:
                # Закрываем клиентский сокет
                client_socket.close()
            '''
            d = s.recvfrom(1)
            self.get_logger().info("Hello from ROS2")
            d = struct.unpack(self.format_type)

            if d == "0xAA":  # hearbeat?
                d = s.recvfrom(1)
                pass
            elif d == "0xB0":  # motors?
                d = s.recvfrom(5)
                pass
            elif d == "0xC0":  # telem?
                pass
            elif d == "0xFF":  # unknown?
                pass'''


def main(args=None):
    rclpy.init(args=args)
    node = Protocol_stm32_node()
    node.spin()
    rclpy.shutdown()


# Запуск с терминала (перед сделать source ~/.bashrc)
# ros2 run lvr_project protocol_stm32
if __name__ == '__main__':
    main()
