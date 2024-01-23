# required for serial
# sudo chmod a+rw /dev/ttyUSB0
# or add user to dialout -> restart
# there is instruction to change usb device symbolics
# update ch340 driver!


import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Header
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import time
import signal
import functools
import sys
import yaml
from ament_index_python.packages import get_package_prefix


class ScanPublisher(Node):

    def __init__(self):
        super().__init__('scanner')
        self.publisher_ = self.create_publisher(PointCloud2, 'lidar_scan', 5)

    def publish_scan(self, scan, scan_params):
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="lidar_link") #frame to be set
        data = []
        height = scan_params["scan_res_v"]
        width = scan_params["scan_res_h"]
        for i in range(height):
            for j in range(width):
                v_angle = scan_params["scan_min_v"] + (height-1-i)*scan_params["scan_step_v"]
                h_angle = j*scan_params["scan_step_h"]
                dist = scan[i][j]/100
                flat_dist = dist*np.cos(v_angle)
                x = float(flat_dist*np.sin(h_angle))
                y = float(flat_dist*np.cos(h_angle))
                z = float(dist*np.sin(v_angle))
                data.append([x, y, z])

        msg = point_cloud2.create_cloud_xyz32(header, data)
        # msg.height = scan_params[0]
        # msg.width = scan_params[1]
        # msg.point_step = len(msg.fields * 4)    #for 4 byte fields
        # msg.row_step = msg.point_step * msg.height * msg.width
        # msg.is_dense = True                     #if all vaild

        self.publisher_.publish(msg)


def kill_handler(arg, signum, frame):
    lidar_serial, scan_publisher = arg
    if lidar_serial is not None:
        lidar_serial.write('S'.encode(encoding = 'UTF-8'))     #stop command
        scan_publisher.get_logger().info(f'Stopped')
    scan_publisher.destroy_node()
    sys.exit(0)


def main(args=None):
    rclpy.init(args=args)

    scan_publisher = ScanPublisher()

    lidar_baud = 115200
    lidar_serial = None
    while not lidar_serial:
        try:
            lidar_serial = serial.Serial('/dev/ttyLID', lidar_baud)
        except serial.SerialException:
            scan_publisher.get_logger().warn(f'Serial not available')
            time.sleep(1)


    #handler to stop the lidar when killed
    arg = lidar_serial, scan_publisher
    signal.signal(signal.SIGINT, functools.partial(kill_handler, arg))


    #read params
    package_path = get_package_prefix('rover_lidar')
    with open(f'{package_path}/../../src/rover_lidar/rover_lidar/rover_lidar_params.yaml', 'r') as file:
        params = yaml.safe_load(file)

    scan_res_v = params["scan_res_v"]
    scan_u_step = params["scan_u_step"]     #1 for full step; 2, 4, 8 for ustepping. ustepping requires current limit I=0.7A
    scan_res_h = 200*scan_u_step
    scan_min_v_deg = params["scan_min_v_deg"]
    scan_min_v = (scan_min_v_deg-90)/180*np.pi
    scan_step_v_deg = params["scan_step_v_deg"]
    scan_step_v = scan_step_v_deg/180*np.pi
    scan_step_h = 2*np.pi/scan_res_h
    scan = np.zeros([scan_res_v, scan_res_h])   #set the type
    scan_cursor_v, scan_cursor_h = 4, 0
    scan_direction = 0

    scan_params = {"scan_res_v": scan_res_v,
                   "scan_res_h": scan_res_h,
                   "scan_step_v": scan_step_v,
                   "scan_step_h": scan_step_h,
                   "scan_min_v": scan_min_v}

    #fake scan for testing
    # while(1):
    #     scan = np.random.randint(300, size=[scan_res_v, scan_res_h])
    #     scan_publisher.get_logger().info(f'Fake scan made')
    #     scan_publisher.publish_scan(scan, scan_params)      #ensure buffer does not overfill
    #     time.sleep(1)

    #scan order inintialize
    time.sleep(5)
    cmd = f'I {scan_u_step} {scan_res_v} {scan_min_v_deg} {scan_step_v_deg}'.encode(encoding = 'UTF-8')
    lidar_serial.write(cmd)
    scan_publisher.get_logger().info(f'Initialize sent')

    #scan receive initialize
    while(True):
        reading = lidar_serial.readline().decode()[:-2]
        if reading == "I":
            reading = lidar_serial.readline().decode()[:-2]
            if reading == "N":
                scan_publisher.get_logger().info(f'Initialized lidar scanner')
                scan_cursor_v, scan_cursor_h = scan_res_v-1, scan_res_h-1
                break

    #scanning
    while(True):
        reading = lidar_serial.readline().decode()[:-2]
        if reading == "T" or reading == "B":
            scan_direction = not scan_direction
            #publish scan
            # scan_publisher.get_logger().info(f'New scan made')
            scan_publisher.publish_scan(scan, scan_params)    #ensure buffer does not overfill
            # print(scan)
        elif reading == "N":
            if not scan_direction:
                scan_cursor_v = scan_cursor_v-1
                scan_cursor_h = scan_res_h-1
            else:
                scan_cursor_v = scan_cursor_v+1
                scan_cursor_h = scan_res_h-1
        elif reading == '0':
            scan[scan_cursor_v, scan_cursor_h] = float('inf')
            scan_cursor_h -= 1
        else:
            scan[scan_cursor_v, scan_cursor_h] = float(reading)
            scan_cursor_h -= 1


if __name__ == '__main__':
    main()