#!/usr/bin/env python3

import rospy
import tf.transformations as tf

from geometry_msgs.msg import Point
from std_msgs.msg import String
from lidar_obstacle_avoidance.msg import ObjectPoints
from nav_msgs.msg import Odometry

import numpy as np

class RobotPoints:
    def __init__(self):
        rospy.init_node('publish_robot_points')

        self.robot_density = rospy.get_param("~robot_density", None)
        self.robot_height = rospy.get_param("~robot_height", None)
        self.robot_width = rospy.get_param("~robot_width", None)
        self.x_offset = rospy.get_param("~x_offset", None)
        self.y_offset = rospy.get_param("~y_offset", None)

        self.robot_pose = False
        self.namespace = rospy.get_namespace()

        self.pose_subscriber = rospy.Subscriber(f"/RosAria/pose", 
                                                Odometry, 
                                                self.pose_callback,
                                                queue_size=10)

        self.publish_robot_points = rospy.Publisher(f"{self.namespace}points",
                                                    ObjectPoints,
                                                    queue_size=10)

        rospy.loginfo(f"Publish {self.namespace.strip('/')} points node started")

    def pose_callback(self, pose_data):
        self.robot_pose = pose_data.pose.pose
    
    def _calculate_robot_points(self, robot_shape, robot_position, robot_orientation):
        zeros_column = np.zeros((robot_shape.shape[0], 1))
        ones_column = np.ones((robot_shape.shape[0], 1))
        robot_shape = np.hstack((robot_shape, zeros_column, ones_column))
        rotation_matrix = tf.quaternion_matrix(robot_orientation)

        # Transpose robot_shape before multiplication
        point_in_world = np.dot(rotation_matrix, robot_shape.T).T[:, :3]
        world_x = point_in_world[:, 0] + robot_position[0]
        world_y = point_in_world[:, 1] + robot_position[1]

        return np.array([world_x, world_y]).T

    def loop(self):
        while not rospy.is_shutdown():
            if not self.robot_pose:
                continue

            robot_position = [self.robot_pose.position.x, self.robot_pose.position.y]
            robot_orientation = [
                self.robot_pose.orientation.x,
                self.robot_pose.orientation.y,
                self.robot_pose.orientation.z,
                self.robot_pose.orientation.w
            ]

            thetas = np.linspace(0, 2 * np.pi, self.robot_density)

            x = (self.robot_height/2) * np.cos(thetas) + self.x_offset
            y = (self.robot_width/2) * np.sin(thetas) + self.y_offset

            robot_shape = np.array([x, y]).T
            robot_points = self._calculate_robot_points(robot_shape, robot_position, robot_orientation)

            robot_points_pub = ObjectPoints()
            robot_points_pub.name = String(self.namespace.strip('/'))
            robot_points_pub.central_point = Point(x=self.x_offset + self.robot_pose.position.x, y=self.y_offset + self.robot_pose.position.y)
            robot_points_pub.points = [Point(x=pt[0], y=pt[1]) for pt in robot_points]

            self.publish_robot_points.publish(robot_points_pub)

def main():
    robot_points = RobotPoints()

    try:
        robot_points.loop()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
