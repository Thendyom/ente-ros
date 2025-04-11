#!/usr/bin/env python3
import os
import math
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

#Occupancy Grid Mapping Class
class OccupancyGridMap:
    def __init__(self, width, height, resolution):
        """
        Initialize a 2D occupancy grid map.
        :param width: Map width in meters.
        :param height: Map height in meters.
        :param resolution: Size of each cell (meters per cell).
        """
        self.resolution = resolution
        self.width = width
        self.height = height
        #Compute number of grid cells in each direction.
        self.nx = int(np.ceil(width / resolution))
        self.ny = int(np.ceil(height / resolution))
        #Initialize the log-odds representation to zero (50% probability).
        self.log_odds = np.zeros((self.ny, self.nx))
        self.l0 = 0  #prior log-odds value corresponding to 0.5 probability.
        #Sensor model parameters (tune as needed).
        self.l_occ = np.log(0.7 / 0.3)  #Evidence for an occupied cell.
        self.l_free = np.log(0.3 / 0.7) #Evidence for a free cell.

    def world_to_map(self, x, y):
        """
        Convert world coordinates (meters) to grid indices.
        Assumes the map is centered at (0, 0).
        """
        ix = int(np.floor((x + self.width / 2.0) / self.resolution))
        iy = int(np.floor((y + self.height / 2.0) / self.resolution))
        return ix, iy

    def update_cell(self, ix, iy, occupied):
        """
        Update a cell's log-odds based on the sensor model.
        :param ix: X index.
        :param iy: Y index.
        :param occupied: Boolean whether the cell is occupied.
        """
        if 0 <= ix < self.nx and 0 <= iy < self.ny:
            if occupied:
                self.log_odds[iy, ix] += self.l_occ - self.l0
            else:
                self.log_odds[iy, ix] += self.l_free - self.l0

    def bresenham_line(self, x0, y0, x1, y1):
        """
        Returns a list of grid indices forming a line between two points using Bresenham's algorithm.
        """
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                cells.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                cells.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        cells.append((x1, y1))
        return cells

    def update_from_sensor(self, pose, sensor_data):
        """
        Update the occupancy grid using sensor measurements.
        :param pose: Robot pose as (x, y, theta) in the world frame.
        :param sensor_data: List of (angle, distance) tuples in the robot's frame.
        """
        x_r, y_r, theta_r = pose
        for angle, distance in sensor_data:
            global_angle = theta_r + angle
            #Compute the measurement hit point in world coordinates.
            x_hit = x_r + distance * math.cos(global_angle)
            y_hit = y_r + distance * math.sin(global_angle)
            #Convert robot and hit point to cell indices.
            start_ix, start_iy = self.world_to_map(x_r, y_r)
            hit_ix, hit_iy = self.world_to_map(x_hit, y_hit)
            #Get the list of grid cells along the sensor ray.
            cells_along_ray = self.bresenham_line(start_ix, start_iy, hit_ix, hit_iy)
            #Update all cells along the ray as free, except the final cell as occupied.
            for (ix, iy) in cells_along_ray[:-1]:
                self.update_cell(ix, iy, occupied=False)
            self.update_cell(hit_ix, hit_iy, occupied=True)

    def get_probability_map(self):
        """
        Convert the log-odds representation back to probabilities.
        """
        odds = np.exp(self.log_odds)
        return odds / (1.0 + odds)

#Street Mapper Node
class StreetMapperNode(DTROS):
    def __init__(self, node_name):
        #Initialize DTROS with a perception node type.
        super(StreetMapperNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'default_vehicle')
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        #Localization output topic. Ensure your KF node publishes a PoseStamped here.
        self.odometry_pose_topic = "/odometry/pose"
        self._bridge = CvBridge()
        self._window = "street_mapper_view"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)

        #Subscribers: camera feed and KF-based pose.
        self.camera_sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.camera_callback)
        self.pose_sub = rospy.Subscriber(self.odometry_pose_topic, PoseStamped, self.pose_callback)
        #Publisher for the occupancy grid (nav_msgs/OccupancyGrid).
        self.map_pub = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size=1)
        
        self.current_pose = None  #Updated via pose_callback.
        #Configure a 2x2 m map with a resolution of 0.02 m (approx. 100 x 100 grid cells).
        self.occupancy_map = OccupancyGridMap(width=2.0, height=2.0, resolution=0.02)

    def pose_callback(self, msg):
        """
        Extracts (x, y, theta) from the PoseStamped message.
        """
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = self.quaternion_to_euler(msg.pose.orientation)
        self.current_pose = (x, y, theta)

    def camera_callback(self, msg):
        """
        Called when a new camera frame is received.
        Converts the image, processes it to detect white street lines,
        and then updates the occupancy grid if a valid pose is available.
        """
        try:
            #Convert the incoming compressed image to an OpenCV BGR image.
            image = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            #Display the raw image.
            cv2.imshow(self._window, image)
            cv2.waitKey(1)
            
            #Process the image to extract sensor-like measurements.
            sensor_data = self.process_image(image)
            
            #Update the map if we have both a valid pose and sensor detections.
            if self.current_pose is not None and sensor_data:
                self.occupancy_map.update_from_sensor(self.current_pose, sensor_data)
                self.publish_occupancy_grid()
                
        except Exception as e:
            rospy.logerr("Error in camera_callback: " + str(e))

    def process_image(self, image):
        """
        Processes the image to detect white lines.
        Uses HSV thresholding and Hough transform to extract line segments.
        For each detected line, computes a sensor measurement (angle, distance)
        relative to the robot's frame.
        :return: List of (angle, distance) tuples.
        """
        #Convert image to HSV.
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #Define thresholds for white color.
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)
        
        #Perform morphological operations to reduce noise.
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)
        
        #Edge detection.
        edges = cv2.Canny(mask, 50, 150, apertureSize=3)
        #Use the probabilistic Hough transform to detect line segments.
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)
        
        sensor_data = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                #Compute the midpoint of the detected line segment.
                mid_x = (x1 + x2) / 2.0
                mid_y = (y1 + y2) / 2.0
                #Draw the detected line and its midpoint for debugging.
                cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(image, (int(mid_x), int(mid_y)), 5, (0, 0, 255), -1)
                
                #Calculate the horizontal offset (dx) from the image center.
                image_center_x = image.shape[1] / 2.0
                dx = mid_x - image_center_x
                #Convert the pixel offset to an angular offset using the horizontal field-of-view.
                horizontal_fov = 62 * np.pi / 180.0  #in radians (example for a Duckiebot camera)
                angle = (dx / image_center_x) * (horizontal_fov / 2.0)
                
                #Estimate distance based on the vertical pixel location.
                distance = self.estimate_distance(mid_y, image.shape[0])
                
                sensor_data.append((angle, distance))
            
            #Display the processed image with detections.
            cv2.imshow("Processed", image)
            cv2.waitKey(1)
        return sensor_data

    def estimate_distance(self, pixel_y, image_height):
        """
        Estimate the distance to a feature based on its vertical coordinate.
        This is a simple linear mapping; adjust or calibrate as needed.
        """
        min_distance = 0.5  #Estimated distance at the bottom of the image (meters)
        max_distance = 5.0  #Estimated distance at the top of the image (meters)
        distance = min_distance + (max_distance - min_distance) * ((image_height - pixel_y) / image_height)
        return distance

    def quaternion_to_euler(self, quat):
        """
        Convert quaternion to a yaw angle (rotation about the Z-axis).
        Assumes the robot operates on a planar surface.
        """
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def publish_occupancy_grid(self):
        """
        Convert the log-odds map to a ROS OccupancyGrid message and publish it.
        Cells with probability greater than 0.5 are marked as occupied (100), else free (0).
        """
        prob_map = self.occupancy_map.get_probability_map()
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"
        grid_msg.info.resolution = self.occupancy_map.resolution
        grid_msg.info.width = self.occupancy_map.nx
        grid_msg.info.height = self.occupancy_map.ny
        #Set the origin so that the map is centered at (0, 0).
        grid_msg.info.origin.position.x = -self.occupancy_map.width / 2.0
        grid_msg.info.origin.position.y = -self.occupancy_map.height / 2.0
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        occupancy_data = []
        prob_threshold = 0.5
        for row in prob_map:
            for p in row:
                occupancy_data.append(100 if p > prob_threshold else 0)
        grid_msg.data = occupancy_data

        self.map_pub.publish(grid_msg)

if __name__ == '__main__':
    try:
        node = StreetMapperNode(node_name='street_mapper_node')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
