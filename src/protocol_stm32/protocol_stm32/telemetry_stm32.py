#!/usr/bin/env python3
import rclpy
import socket
import struct
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Vector3, PoseWithCovariance, TwistWithCovariance
from sensor_msgs.msg import Imu, BatteryState, NavSatStatus, NavSatFix
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterValue
import numpy as np
from math import radians, cos, sin
from time import sleep, time
from math import pi as Pi

def rad_per_sec(grad_per_sec):
    """Переводит градусы в секеунду в радианы в секунду"""
    return grad_per_sec*0.01745329

def meter_per_second(nodal_speed):
    """Переводит узлы в метры в секунду"""
    return nodal_speed*0.51444444444

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


def Speed_converter(td):
    """ Получает на вход словарь телеметрии и создает ParameterValue.msg для скорости"""
    speed_msg = ParameterValue()
    speed_msg.type = 3 #double type
    speed_msg.double_value = meter_per_second(td['speed'])
    return speed_msg

def Nav_converter(td):
    """ Получает на вход словарь телеметрии и создает NavSatFix.msg"""
    status_msg = NavSatStatus()
    status_msg.status = int(td['gps_valid'])

    nav_sat_fix_msg = NavSatFix()
    nav_sat_fix_msg.latitude = td['latitude']
    nav_sat_fix_msg.longitude = td['longitude']
    nav_sat_fix_msg.status = status_msg

    return nav_sat_fix_msg

def Course_converter(td):
    """ Получает на вход словарь телеметрии и создает ParameterValue.msg для курса"""
    course_msg = ParameterValue()
    course_msg.type = 3 #double type
    course_msg.double_value = td['course']
    return course_msg

def Odometry_converter(td, dt, prev_x, prev_y):
    """ Получает на вход словарь телеметрии, интервал времени и предыдщие значения
    положения в пространстве. Возвращает Odometry.msg и новые координаты в пространстве"""

    def MeterPerSec(v):
        """Принимает на вход скорость в тиках в секунду и переводит ее в тики в секунду"""
        D = 0.25  # диаметр колеса в метрах
        # 210 - колиечство тиков на оборот
        return v * ((Pi * D)/210)

    v_left = td['fl_speed'] #скорость в тиках в секунду
    v_right = td['fr_speed']
    cmd = td['fr_cmd']
    print(f'команда на правое колесо в тиках {cmd}')
    print(f'v_left in tics = {v_left}')
    v_left = MeterPerSec(v_left)
    v_right = MeterPerSec(v_right)
    print(f'v_left in meters = {v_left}')
    yaw = td['yaw']
    v = (v_left + v_right) / 2 

    x = prev_x + v*cos(yaw)*dt
    y = prev_y + v*sin(yaw)*dt

    odometry_msg = Odometry()
    odometry_msg.child_frame_id = "base_link"
    odometry_msg.pose.pose.position.x = x
    odometry_msg.pose.pose.position.y = y

    odometry_msg.pose.pose.orientation = Calculate_quaternion(td['roll'], td['pitch'], td['yaw'])
    odometry_msg.twist.twist.linear.x = v * cos(yaw)
    odometry_msg.twist.twist.linear.y = v * sin(yaw)

    odometry_msg.twist.twist.angular.x = rad_per_sec(td['gx'])
    odometry_msg.twist.twist.angular.y = rad_per_sec(td['gy'])
    odometry_msg.twist.twist.angular.z = rad_per_sec(td['gz'])


    return {'odometry_msg': odometry_msg, 'x_new': x, 'y_new': y}



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

        self.imu_publisher_ = self.create_publisher(Imu, '/imu_topic', 10)
        self.battery_publisher_ = self.create_publisher(BatteryState, '/battery_topic', 10)
        self.speed_publisher_ = self.create_publisher(ParameterValue, '/speed_topic', 10)
        self.nav_publisher_ = self.create_publisher(NavSatFix, '/nav_topic', 10)
        self.course_publisher_ = self.create_publisher(ParameterValue, '/course_topic', 10)
        self.odometry_publisher_ = self.create_publisher(Odometry, '/odometry_topic', 10)

        self.prev_time = time()
        self.curr_time = -1.
        self.dt = 0.

        self.x = 0.
        self.y = 0.
        self.L = 0.58 #58 см - расстояние между колесами

        '''
        H - uint8_t
        f - float
        h - int16_t
        '''

    def spin(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client_socket:
            client_socket.connect(('192.168.5.100', 2007))
            self.get_logger().info("Подключение к серверу STM успешно")
            try:

                while True:
                    telemetry_data = Telemetry_Request(client_socket)
                    self.curr_time = time()
                    self.dt = self.curr_time - self.prev_time
                    self.get_logger().info("Телеметрия получена")
                    self.prev_time = time()
                    print(telemetry_data)

                    Imu_msg = Imu_converter(telemetry_data)
                    Battery_msg = Battery_converter(telemetry_data)
                    Speed_msg = Speed_converter(telemetry_data)
                    NavSatFix_msg = Nav_converter(telemetry_data)
                    Course_msg = Course_converter(telemetry_data)
                    print(f'old_x={self.x}  old_y = {self.y}')
                    odom_dict = Odometry_converter(telemetry_data, self.dt, self.x, self.y)
                    Odometry_msg = odom_dict['odometry_msg']
                    self.x = odom_dict['x_new']
                    self.y = odom_dict['y_new']
                    print(f'new_x={self.x}  new_y = {self.y}')



                    self.imu_publisher_.publish(Imu_msg)
                    self.get_logger().info('Imu.msg published')

                    self.battery_publisher_.publish(Battery_msg)
                    self.get_logger().info('BatteryState.msg published')

                    self.speed_publisher_.publish(Speed_msg)
                    self.get_logger().info('ParameterValue.msg for speed published')

                    self.nav_publisher_.publish(NavSatFix_msg)
                    self.get_logger().info('NavSatFix.msg published')

                    self.course_publisher_.publish(Course_msg)
                    self.get_logger().info('ParameterValue.msg for course published')

                    self.odometry_publisher_.publish(Odometry_msg)
                    self.get_logger().info('Odometry.msg published')

                    self.get_logger().info('-------------------------------------------')

                    sleep(1)
            finally:
                self.get_logger().info("Ошибка")
                # Закрываем клиентский сокет
                client_socket.close()


def main(args=None):
    rclpy.init(args=args)
    node = Protocol_stm32_node()
    node.spin()
    rclpy.shutdown()


# Запуск с терминала (перед сделать source ~/.bashrc) если что-то не робит
# ros2 run protocol_stm32 telemetry_stm32
if __name__ == '__main__':
    main()
