#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import Imu

class IMUReaderNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(IMUReaderNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Get vehicle name from environment and build the IMU topic name
        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'ente')
        self._imu_topic = f"/{self._vehicle_name}/imu_node/imu_data"

        # Temporary data storage for the latest IMU message
        self._latest_imu = None

        # Construct subscriber for the IMU topic
        self.sub = rospy.Subscriber(self._imu_topic, Imu, self.imu_callback)

    def imu_callback(self, data):
        # Log sensor frame info once and store the incoming data
        rospy.loginfo_once(f"IMU sensor frame: {data.header.frame_id}")
        self._latest_imu = data

    def run(self):
        # Publish received IMU data every 0.05 seconds (20 Hz)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self._latest_imu is not None:
                acc = self._latest_imu.linear_acceleration
                gyro = self._latest_imu.angular_velocity
                msg = (f"IMU Linear Acc [x, y, z]: [{acc.x:.2f}, {acc.y:.2f}, {acc.z:.2f}] | "
                       f"Angular Vel [x, y, z]: [{gyro.x:.2f}, {gyro.y:.2f}, {gyro.z:.2f}]")
                rospy.loginfo(msg)
            rate.sleep()

if __name__ == '__main__':
    # Create the node
    node = IMUReaderNode(node_name='imu_reader_node')
    # Run the periodic loop to log IMU data
    node.run()
    # Keep the process alive (note: this line won't be reached because run() is blocking)
    rospy.spin()
