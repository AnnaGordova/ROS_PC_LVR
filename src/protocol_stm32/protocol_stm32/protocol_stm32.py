#!/usr/bin/env python3
import rclpy
import socket
import struct
from rclpy.node import Node

def Telemetry_Request(sock):
    """
    Возвращает ответ телеметрии в виде байтовой строки
    """
    request_message = bytes.fromhex('C0')
    sock.sendall(request_message)
    response_message = sock.recv(82) # 1024?
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

        
        '''
        self.format_type = "=H"
        self.format_heartbeat = "=H"
        self.format_motors = "=Hffff"
        self.format_telemetry_request = "=H"
        self.format_unknown_packet = "=H"  '''
        self.format_telemetry = "=Bfffffffffhhhhhhhhhhf?fffff"

    def spin(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client_socket:
            client_socket.connect(('192.168.5.100', 2007))
            self.get_logger().info("Подключение к серверу STM успешно")
            try:
                byte_string = Telemetry_Request(client_socket)
                self.get_logger().info("Ответ телеметрии получен")
                print(struct.calcsize(self.format_telemetry))
                
                data = struct.unpack(self.format_telemetry, byte_string)

                telemetry_keys = ['type', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'roll', 'yaw', 'pitch',
                                  'fl_cmd', 'fr_cmd', 'bl_cmd', 'br_cmd', 'fl_speed', 'fr_speed', 'bl_speed',
                                  'br_speed', 'boardf_temp', 'boardb_temp', 'bat_value', 'gps_valid',
                                  'latitude', 'longitude', 'speed', 'course', 'variation']
                telemetry_info = {}
                for i in range(len(data)):
                    telemetry_info[telemetry_keys[i]] = data[i]
                print(telemetry_info)
            finally:
                # Закрываем клиентский сокет
                client_socket.close()


def main(args=None):
    rclpy.init(args=args)
    node = Protocol_stm32_node()
    node.spin()
    rclpy.shutdown()


# Запуск с терминала (перед сделать source ~/.bashrc) если что-то не робит
# ros2 run protocol_stm32 protocol_stm32
if __name__ == '__main__':
    main()
