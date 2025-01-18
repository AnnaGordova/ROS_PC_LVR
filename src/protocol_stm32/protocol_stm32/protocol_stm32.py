#!/usr/bin/env python3
import rclpy
import socket
import struct
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu, BatteryState
import numpy as np
from math import radians
from time import sleep

def rad_per_sec(grad_per_sec):
    """Переводит градусы в секеунду в радианы в секунду"""
    return grad_per_sec*0.01745329

def Calculate_quaternion(r, p, y):
    """
    Конвертирует углы Эйлера в квартернион, предварительно переведя градусы в радианы

    Input
      :param roll: The roll (rotation around x-axis) angle в градусах
      :param pitch: The pitch (rotation around y-axis) angle в градусах
      :param yaw: The yaw (rotation around z-axis) angle в градусах

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    roll, pitch, yaw = radians(r), radians(p), radians(y)
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    Q = Quaternion(x = qx, y = qy, z = qz, w = qw)
    return Q


def Imu_converter(td):
    """ Получает на вход словарь телеметрии и создает Imu.msg"""
    #geometry_msgs/Quaternion and empty_covariance
    orientation = Calculate_quaternion(td['roll'], td['pitch'], td['yaw'])
    orientation_covariance = [0.0] * 9 #ВОЗМОЖНО ПОМЕНЯТЬ
    #geometry_msgs/Vector3 (angular_velocity) and empty_covariance
    angular_velocity = Vector3(x=rad_per_sec(td['gx']), y=rad_per_sec(td['gy']), z=rad_per_sec(td['gz']))
    angular_velocity_covariance = [0.0] * 9 #ВОЗМОЖНО ПОМЕНЯТЬ
    #geometry_msgs/Vector3 (linear_acceleration) and empty_covariance
    linear_acceleration = Vector3(x=td['ax'], y=td['ay'], z=td['az'])
    linear_acceleration_covariance = [0.0] * 9 #ВОЗМОЖНО ПОМЕНЯТЬ

    imu_msg = Imu()
    imu_msg.orientation = orientation
    imu_msg.orientation_covariance = orientation_covariance
    imu_msg.angular_velocity = angular_velocity
    imu_msg.angular_velocity_covariance = angular_velocity_covariance
    imu_msg.linear_acceleration = linear_acceleration
    imu_msg.linear_acceleration_covariance = linear_acceleration_covariance

    return imu_msg
    
def Battery_converter(td):
    """ Получает на вход словарь телеметрии и создает Battery_State.msg"""
    battery_msg = BatteryState()
    battery_msg.voltage = td['bat_value']
    return battery_msg


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

        self.imu_publisher_ = self.create_publisher(Imu, '/imu_topic', 10)
        self.battery_publisher_ = self.create_publisher(BatteryState, '/battery_topic', 10)


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
                print(telemetry_data)
                Imu_msg = Imu_converter(telemetry_data)
                Battery_msg = Battery_converter(telemetry_data)
                print(Battery_msg)
                while True:
                    self.imu_publisher_.publish(Imu_msg)
                    self.get_logger().info('Imu.msg published')

                    self.battery_publisher_.publish(Battery_msg)
                    self.get_logger().info('BatteryState.msg published')
                    sleep(1)
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
