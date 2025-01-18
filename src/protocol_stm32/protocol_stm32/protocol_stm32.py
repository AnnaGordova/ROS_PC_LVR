#!/usr/bin/env python3
import rclpy
import socket
import struct
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
import numpy as np  


def Calculate_quaternion(roll, pitch, yaw):
    """
    Конвертирует углы Эйлера в квартернион

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    Q = Quaternion(x = qx, y = qy, z = qz, w = qw)
    return Q


def Telemetry_Request(sock):
    """
    Возвращает данные телеметрии в виде словаря
    """
    format_telemetry = "=Bfffffffffhhhhhhhhhhf?fffff"
    request_message = bytes.fromhex('C0')
    sock.sendall(request_message)
    byte_string = sock.recv(82)
    data = struct.unpack(format_telemetry, byte_string)
    telemetry_keys = ['type', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'roll', 'yaw', 'pitch',
                      'fl_cmd', 'fr_cmd', 'bl_cmd', 'br_cmd', 'fl_speed', 'fr_speed', 'bl_speed',
                      'br_speed', 'boardf_temp', 'boardb_temp', 'bat_value', 'gps_valid',
                      'latitude', 'longitude', 'speed', 'course', 'variation']
    telemetry_info = {}
    for i in range(len(data)):
        telemetry_info[telemetry_keys[i]] = data[i]
    return telemetry_info



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


    def spin(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client_socket:
            client_socket.connect(('192.168.5.100', 2007))
            self.get_logger().info("Подключение к серверу STM успешно")
            try:
                telemetry_data = Telemetry_Request(client_socket)
                self.get_logger().info("Телеметрия получена")
                q = Calculate_quaternion(telemetry_data['roll'], telemetry_data['pitch'], telemetry_data['yaw'])
                print(telemetry_data)
                print(q)
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
