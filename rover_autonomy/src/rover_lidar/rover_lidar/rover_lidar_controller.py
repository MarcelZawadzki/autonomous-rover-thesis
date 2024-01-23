# required for serial
# sudo chmod a+rw /dev/ttyUSB0
# or add user to dialout -> restart
# there is instruction to change usb device symbolics


import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, MagneticField, NavSatFix
from geometry_msgs.msg import Quaternion, Vector3, Twist
import numpy as np
import time
import signal
import functools
import sys
import yaml
from ament_index_python.packages import get_package_prefix
import math

G_ACC = 9.807

rover_params = {"R": 0.0593,        #wheel radius
                "L": 0.344,         #track width
                "D": 0.192,         #wheelbase/2; distance between pivot and axes
                "max_v": 0.230,     #max speed m/s
                "max_ang": 0.347,   #max angle rad
                "max_steering_coef": 4.0}#how many times sharper can a turn be while steering; else: normal diff drive

A_v = [0.0, 0.0, 0.0]
G_v = [0.0, 0.0, 0.0]
M_v = [0.0, 0.0, 0.0]
T_v = 0.0
R_v = 0.0
P_v = 0.0
gyro_calibration = [-1.57096163636364,
                        1.81648942727273,
                        0.0568736636363636]
acc_calibration = [0.040,
                    -0.039,
                    0.023]

#speed @max v, driving *0.23* 0.22 0.23 0.23 0.24 0.25 (m/s)
#max steer angle *0.347* 0.328 0.319 0.348 0.409 0.329(rad)



class ControllerPubSub(Node):

    def __init__(self):
        super().__init__('controller')

        rover_baud = 115200
        self.rover_serial = None

        self.declare_parameter('gps', True)
        self.use_gps = self.get_parameter('gps').get_parameter_value().bool_value

        while not self.rover_serial:
            try:
                self.rover_serial = serial.Serial('/dev/ttyRVR', rover_baud, write_timeout=0.005, timeout=0.005)

                if not self.use_gps:
                    time.sleep(12)
                    self.get_logger().info( f'Set to ignore GNSS' )
                    cmd = f's'.encode(encoding = 'UTF-8')
                    self.rover_serial.write(cmd)
                else:
                    while True:
                        reading = self.rover_serial.readline().decode()[:-2]
                        if len(reading) >= 2 and reading != "GPS_ini":
                            self.get_logger().info( f'GPS connected' )
                            break
                        else:
                            self.get_logger().warn( f'No GPS fix' )
                            time.sleep(1)
                self.get_logger().info( f'Rover initialized' )
            except serial.SerialException:
                self.get_logger().warn( f'Serial not available' )
                time.sleep(1)
            except:
                self.get_logger().warn( f'Unexpected error in rover communication' )
                time.sleep(1)

        self.acc_publisher = self.create_publisher(Imu, 'imu/data_raw', 5)
        self.mag_publisher = self.create_publisher(MagneticField, 'imu/mag', 5)
        if self.use_gps:
            self.gps_publisher = self.create_publisher(NavSatFix, 'gps_rover', 5)
        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 5)

        self.read_timer = self.create_timer(0.01, self.sensor_callback)


    def cmd_vel_callback(self, msg):
        lin_speed = msg.linear.x
        ang_speed = msg.angular.z
        #but when steering it goes double ie. 0 510
        if lin_speed > rover_params['max_v']: lin_speed = rover_params['max_v']
        if lin_speed < -rover_params['max_v']: lin_speed = -rover_params['max_v']
        if ( rover_params['L'] * ang_speed ) / 2.0 > rover_params['max_v']:
            ang_speed = rover_params['max_v'] * 2.0 / rover_params['L']
        if ( rover_params['L'] * ang_speed ) / 2.0 < -rover_params['max_v']:
            ang_speed = -rover_params['max_v'] * 2.0 / rover_params['L']

        #conversion to rads / rads/s
        l_speed = lin_speed / rover_params['R'] - ( rover_params['L'] * ang_speed ) / ( 2.0 * rover_params['R'] )
        r_speed = lin_speed / rover_params['R'] + ( rover_params['L'] * ang_speed ) / ( 2.0 * rover_params['R'] )
        #steering has to be inverted due to servo placement
        steer_ang = -2.0 * math.atan2(rover_params['D'] * ang_speed, lin_speed)  #positive w = positive ang
        if steer_ang < -math.pi: steer_ang += 2 * math.pi
        if steer_ang > math.pi: steer_ang -= 2 * math.pi
        #conversion to internal units
        #speed 0 to 510, ang 0 to 180, ints
        if abs(steer_ang) > rover_params['max_steering_coef'] * rover_params['max_ang']:
            int_steer = 90
        else:
            int_steer = int( steer_ang * 90.0 / rover_params['max_ang'] ) + 90
            if int_steer < 0:
                int_steer = 0
            if int_steer > 180:
                int_steer = 180
        speed_multiplier = rover_params['R'] * 255.0 / rover_params['max_v']
        int_speed_l = int( l_speed * speed_multiplier ) + 255
        int_speed_r = int( r_speed * speed_multiplier ) + 255
        if int_speed_l < 0:
            int_speed_l = 0
        if int_speed_l > 510:
            int_speed_l = 510
        if int_speed_r < 0:
            int_speed_r = 0
        if int_speed_r > 510:
            int_speed_r = 510

        # self.get_logger().info( f'L: {int_speed_l-255}, R: {int_speed_r-255}, Ang: {int_steer-90}' )

        cmd = f'D {int_speed_l} {int_speed_r} {int_steer}'.encode(encoding = 'UTF-8')
        try:
            self.rover_serial.write(cmd)
        except:
            self.get_logger().warn('Could not send cmd vel')


    def publish_imu(self, acceleration, angular_velocity, magnetic_field, gyro_calibration, acc_calibration):
        imu_msg = Imu()
        mag_msg = MagneticField()
        #Header
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="imu_link")
        imu_msg.header = header
        mag_msg.header = header

        #  Linear Acceleration
        imu_msg.linear_acceleration.x = (acceleration[1] - acc_calibration[1])*G_ACC
        imu_msg.linear_acceleration.y = -(acceleration[0] - acc_calibration[0])*G_ACC
        imu_msg.linear_acceleration.z = (acceleration[2] - acc_calibration[2])*G_ACC

        # Angular Velocity, with calibration and convertion to radians
        imu_msg.angular_velocity.x = (angular_velocity[1] - gyro_calibration[1]) * math.pi / 180.0
        imu_msg.angular_velocity.y = -(angular_velocity[0] - gyro_calibration[0]) * math.pi / 180.0
        imu_msg.angular_velocity.z = (angular_velocity[2] - gyro_calibration[2]) * math.pi / 180.0

        # No orientation
        imu_msg.orientation_covariance[0] = -1

        # Covariance for kalman
        imu_msg.linear_acceleration_covariance[0] = 0.1
        imu_msg.linear_acceleration_covariance[1] = 0.1
        imu_msg.linear_acceleration_covariance[2] = 0.1

        # Magnetic Field
        mag_msg.magnetic_field.x = magnetic_field[0]
        mag_msg.magnetic_field.y = -magnetic_field[1]
        mag_msg.magnetic_field.z = -magnetic_field[2]

        self.acc_publisher.publish(imu_msg)
        self.mag_publisher.publish(mag_msg)

    def publish_gps(self, lat, lon, alt, lat_e, lon_e, alt_e):
        gps_msg = NavSatFix()
        #Header
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="tower_link")
        gps_msg.header = header

        gps_msg.status.status = 0   # feed empty when no fix
        gps_msg.status.service = 3  # transmit?

        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = alt

        gps_msg.position_covariance[0] = pow(lon_e, 2)
        gps_msg.position_covariance[4] = pow(lat_e, 2)
        gps_msg.position_covariance[8] = pow(alt_e, 2)
        gps_msg.position_covariance_type = 2

        self.gps_publisher.publish(gps_msg)

    def sensor_callback(self):
        if(self.rover_serial.in_waiting):
            try:
                reading = self.rover_serial.readline().decode()[:-2]
                if reading == 'A':
                    for i in range(3):
                        reading = self.rover_serial.readline().decode()[:-2]
                        A_v[i] = float(reading)
                    for i in range(3):
                        reading = self.rover_serial.readline().decode()[:-2]
                        G_v[i] = float(reading)
                    for i in range(3):
                        reading = self.rover_serial.readline().decode()[:-2]
                        M_v[i] = float(reading)
                    self.publish_imu(A_v, G_v, M_v, gyro_calibration, acc_calibration)
                elif reading == 'G':
                    lat = float(self.rover_serial.readline().decode()[:-2])
                    lon = float(self.rover_serial.readline().decode()[:-2])
                    alt = float(self.rover_serial.readline().decode()[:-2])
                    lat_e = float(self.rover_serial.readline().decode()[:-2])
                    lon_e = float(self.rover_serial.readline().decode()[:-2])
                    alt_e = float(self.rover_serial.readline().decode()[:-2])
                    if self.use_gps:
                        self.publish_gps(lat, lon, alt, lat_e, lon_e, alt_e)
            except Exception as e:
                self.get_logger().warn(f'Corrupted data on rover serial:')
                self.get_logger().warn(e.__str__())



def kill_handler(arg, signum, frame):
    controller_pub_sub = arg
    if controller_pub_sub.rover_serial is not None:
        pass
        #stopping instantly when quit
        cmd = f'D {255} {255} {90}'.encode(encoding = 'UTF-8')
        controller_pub_sub.rover_serial.write(cmd)
        controller_pub_sub.get_logger().info(f'Rover stopped')
    controller_pub_sub.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


def main(args=None):
    rclpy.init(args=args)

    controller_pub_sub = ControllerPubSub()
    rate = controller_pub_sub.create_rate(100)

    #handler to stop when killed
    arg = controller_pub_sub
    signal.signal(signal.SIGINT, functools.partial(kill_handler, arg))

    rclpy.spin(controller_pub_sub)


if __name__ == '__main__':
    main()