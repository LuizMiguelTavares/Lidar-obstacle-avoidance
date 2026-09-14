#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class RoutePublisher:
    def __init__(self):
        rospy.init_node('circular_path_publisher')      
        
        self.frame_id = rospy.get_param('~frame_id', 'world')
        self.x_radius = rospy.get_param('~x_radius', 1.0)
        self.y_radius = rospy.get_param('~y_radius', 1.0)

        self.x_center = rospy.get_param('~x_center', 0.0)
        self.y_center = rospy.get_param('~y_center', 0.0)

        self.z_height = rospy.get_param('~z_height', 0.5)
        self.resolution = rospy.get_param('~resolution', 100)

        self.namespace = rospy.get_namespace()

        self.publisher = rospy.Publisher(f'{self.namespace}route', 
                                         Path, 
                                         queue_size=10)

        self.rate = rospy.Rate(30)

        rospy.loginfo(f"{self.namespace.strip('/')} route publisher node started!")

    def circular_route(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            x_radius = self.x_radius
            y_radius = self.y_radius
            x_center = self.x_center
            y_center = self.y_center
            z_height = self.z_height
            resolution = self.resolution

            radians = np.linspace(0, 2*np.pi, resolution)
            x = x_radius * np.cos(radians) + x_center
            y = y_radius * np.sin(radians) + y_center
            z = np.ones(radians.size) * z_height

            path = Path()
            path.header.stamp = current_time
            path.header.frame_id = self.frame_id

            for xi, yi, zi in zip(x, y, z):
                pose = PoseStamped()
                pose.header.stamp = current_time
                pose.header.frame_id = self.frame_id
                pose.pose.position.x = xi
                pose.pose.position.y = yi
                pose.pose.position.z = zi
                path.poses.append(pose)

            self.publisher.publish(path)

            self.rate.sleep()

def main():
    route = RoutePublisher()
    route.circular_route()

if __name__ == '__main__':
    main()