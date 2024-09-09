#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R


class OdometryToTUMTxt:
    def __init__(self, root_path, output_file):
        self.output_file = root_path + output_file
        self.start_time_step = None
        self.cam0_T_body = np.array([
            [0.00875116, -0.99986423, 0.01396015, -0.33908972],
            [-0.00479609, -0.01400249, -0.99989048, 0.74680283],
            [0.99995014, 0.00868325, -0.00491798, -1.09574845],
            [0., 0., 0., 1.]
        ])
        self.odom_subscriber = rospy.Subscriber('/Odometry', Odometry, self.odom_callback)

    def odom_callback(self, odom_msg):
        if self.start_time_step is None:
            self.start_time_step = odom_msg.header.stamp.to_sec()
        timestamp_sec = odom_msg.header.stamp.to_sec() - self.start_time_step
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        z = odom_msg.pose.pose.position.z
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w

        parts = [timestamp_sec, x, y, z, qx, qy, qz, qw]

        position = np.array([parts[1], parts[2], parts[3]])
        quaternion = np.array([parts[4], parts[5], parts[6], parts[7]])

        # 转换位置
        transformed_position = np.dot(self.cam0_T_body[:3, :3], position.T).T + self.cam0_T_body[:3, 3]

        # 转换旋转
        rotation = R.from_quat(quaternion)
        transformed_rotation = R.from_matrix(self.cam0_T_body[:3, :3]) * rotation
        transformed_quaternion = transformed_rotation.as_quat()

        # 合并转换后的数据，并保留四位小数
        transformed_line = np.hstack((transformed_position, transformed_quaternion))
        transformed_line_precise = np.round(transformed_line, 4)

        line = f"{parts[0]:.9f} {parts[1]:.9f} {parts[2]:.9f} {parts[3]:.9f} {parts[4]:.9f} {parts[5]:.9f} {parts[6]:.9f} {parts[7]:.9f}\n"
        with open(self.output_file, 'a') as file:
            file.write(line)


if __name__ == '__main__':

    rospy.init_node('odom2txt', anonymous=True)
    root_path = rospy.get_param('root_path', '/home/lpz/ME5413/catkin_ws/src/hw_2_pkg/result/')
    output_file = "output.txt"
    odom_converter = OdometryToTUMTxt(root_path, output_file)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
