#!/usr/bin/env python3
import rclpy
import socket
import struct
from rclpy.node import Node
import time


def Heartbeat_Response(sock):
    """
    Отправляет heartbeat
    """
    request_message = bytes.fromhex('AA')
    sock.sendall(request_message)


class Heartbeat_node(Node):
    def __init__(self):
        super().__init__("Heartbeat")
        self.seconds = 0.0
        

    def spin(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client_socket:
            client_socket.connect(('192.168.5.100', 2007))
            self.get_logger().info("Подключение к серверу STM успешно")
            while True:
                try:
                    Heartbeat_Response(client_socket)
                    self.get_logger().info(f"heartbeat отправлен -- {str(self.seconds)} секунд")
                    time.sleep(0.5)
                    self.seconds += 0.5
                finally:
                    pass
                  



def main(args=None):
    rclpy.init(args=args)
    node = Heartbeat_node()
    node.spin()
    rclpy.shutdown()


# Запуск с терминала (перед сделать source ~/.bashrc)
# ros2 run protocol_stm32 heartbeat
if __name__ == '__main__':
    main()
