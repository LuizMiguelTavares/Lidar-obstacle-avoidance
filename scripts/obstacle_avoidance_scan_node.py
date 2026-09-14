#!/usr/bin/env python3

import rospy
import tf.transformations as tf

from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import LaserScan

from aurora_py.obstacle_avoidance_2d import ObstacleAvoidance
from lidar_obstacle_avoidance.msg import ObjectPoints
from scipy.cluster.hierarchy import linkage, fcluster
from scipy.spatial import KDTree
import numpy as np

class ObstacleAvoidanceScan:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_scan')
        self.robot_pose = None
        self.scan_data = None
        self.robot_points = None

        self.max_obstacle_distance = rospy.get_param('~max_obstacle_distance', None)   
        self.density_gain = rospy.get_param('~density_gain', None)
        self.observation_radius = rospy.get_param('~observation_radius', None)
        self.min_threshold = rospy.get_param('~min_threshold', None)
        self.cumulative_distance = rospy.get_param('~cumulative_distance', None)
        self.scan_topic = rospy.get_param('~scan_topic', None)

        obstacle_avoidance = rospy.get_param('~obstacle_avoidance', None)
        n = obstacle_avoidance['n']
        a = obstacle_avoidance['a']
        b = obstacle_avoidance['b']
        k = obstacle_avoidance['k']
        self.obs_avoidance = ObstacleAvoidance(n=n, a=a, b=b, k=k)

        self.namespace = rospy.get_namespace()
        
        self.scan_sub = rospy.Subscriber(self.scan_topic, 
                                            LaserScan, 
                                            self.scan_callback, 
                                            queue_size=10)
        
        self.pose_sub = rospy.Subscriber(f"/natnet_ros{self.namespace}pose",
                                            PoseStamped, 
                                            self.pose_callback, 
                                            queue_size=10)
        
        self.subscribe_robot_points = rospy.Subscriber(f"{self.namespace}points",
                                                    ObjectPoints,
                                                    self.sub_robot_points,
                                                    queue_size=10)
        
        self.potential_publisher = rospy.Publisher(f"{self.namespace}potential",
                                                    Point,
                                                    queue_size=10) 
    
        self.rate = rospy.Rate(30)

        rospy.loginfo(f"{self.namespace.strip('/')} obstacle avoidance node started")

    def scan_callback(self, scan_data):
        self.scan_data = scan_data
        self.num_laser_points = len(scan_data.ranges)
    
    def pose_callback(self, pose_data):
        self.robot_pose = pose_data.pose
    
    def sub_robot_points(self, robot_points_data):
        self.robot_points = np.array([[p.x, p.y] for p in robot_points_data.points])

    def calculate_cartesian_cooerdinates(self, scan_data, robot_position, robot_orientation):
        current_angle = scan_data.angle_min
        x = []
        y = []
        index = []

        rotation_matrix = tf.quaternion_matrix(robot_orientation)

        for idx, r in enumerate(scan_data.ranges):
            if r < scan_data.range_min or r > scan_data.range_max:
                current_angle += scan_data.angle_increment
                continue
            local_x = r * np.cos(current_angle)
            local_y = r * np.sin(current_angle)

            point_in_world = np.dot(rotation_matrix, [-local_x, -local_y, 0, 1])[:3]
            world_x = point_in_world[0] + robot_position[0]
            world_y = point_in_world[1] + robot_position[1]
            
            x.append(world_x)
            y.append(world_y)
            index.append(idx)
            
            current_angle += scan_data.angle_increment
        return np.array([x, y]).T, index

    def select_obstacle_points(self, robot_points, room_points, index, observation_radius, min_threshold):
        robot_center = np.mean(robot_points, axis=0)
        
        filtered_points = [(idx, room_point) for idx, room_point in zip(index, room_points) 
                        if min_threshold < np.linalg.norm(robot_center - room_point) < observation_radius]
        
        if len(filtered_points) == 0:
            return np.array([]), np.array([])
            
        selected_indices, selected_points = zip(*filtered_points)
        
        return np.array(selected_points), np.array(selected_indices)

    def clusterize_obstacles(self, obstacle_points, max_obstacle_distance):
        Z = linkage(obstacle_points, 'single')
        return fcluster(Z, max_obstacle_distance, criterion='distance')

    def calculate_potential(self, robot_points, obstacle_points, clusters, obstacle_idx, cumulative_distance, density_gain):
        if clusters[0] == clusters[-1] and len(set(clusters))>1:
            clusters = list(clusters)
            obstacle_points = list(obstacle_points)
            while clusters[0] == clusters[-1]:
                aux, obs_aux = clusters.pop(-1), obstacle_points.pop(-1)
                clusters.insert(0, aux)
                obstacle_points.insert(0, obs_aux)
            obstacle_points = np.array(obstacle_points)
        elif np.any(obstacle_idx > (self.num_laser_points-self.num_laser_points/4)) and np.any(obstacle_idx < self.num_laser_points/4) and len(set(clusters))==1:
            max_diff = 0
            index_of_max_diff = -1
            
            for i in range(1, len(obstacle_idx)):
                diff = obstacle_idx[i] - obstacle_idx[i-1]
                if diff > max_diff:
                    max_diff = diff
                    index_of_max_diff = i - 1

            obstacle_points = np.concatenate((obstacle_points[index_of_max_diff + 1:], obstacle_points[:index_of_max_diff + 1]), axis=0)
            clusters = np.concatenate((clusters[index_of_max_diff + 1:], clusters[:index_of_max_diff + 1]))

        cluster_dict = {cluster: obstacle_points[np.where(np.array(clusters) == cluster)] for cluster in set(clusters)}

        x_dot, y_dot = 0, 0
        for cluster, points in cluster_dict.items():
            kdtree = KDTree(points)
            smallest_distance = np.inf
            for robot_point in robot_points:
                dist, index = kdtree.query(robot_point)
                if dist < smallest_distance:
                    smallest_distance = dist
                    closest_index = index
                    closest_robot_point = robot_point

            x_p = np.array([points[0][0], points[closest_index][0], points[-1][0]]) 
            y_p = np.array([points[0][1], points[closest_index][1], points[-1][1]]) 

            control_points = self._calculate_control_points(points[closest_index], points, cumulative_distance=cumulative_distance)
            density = int(density_gain*len(control_points)/smallest_distance**2) + 4
            bezier_points = self._bezier_curve(control_points, density=density)
            
            # Pensar na possibilidade de colocar mais pontos do robô e não só o mais próximo
            x_dot_partial, y_dot_partial = self.obs_avoidance.obstacle_avoidance(closest_robot_point, bezier_points)
            x_dot += x_dot_partial
            y_dot += y_dot_partial
        return x_dot, y_dot

    def _calculate_control_points(self, closest_point, points, cumulative_distance=0.15):
        cum_dist = 0 
        control_points = []
        
        for idx, point in enumerate(points):
            if idx>0:
                cum_dist += np.linalg.norm(np.array(point)-np.array(prev_point))
            prev_point = point

            if idx == 0 or idx == len(points)-1 or cum_dist >= cumulative_distance or np.all(point==closest_point):
                control_points.append(point)
                
                cum_dist = 0
        control_points = np.array(control_points)
        return control_points
    
    def _bezier_curve(self, control_points, density):
        control_points = np.array(control_points)
        t_values = np.linspace(0, 1, density)
 
        curve_points = np.zeros((density, 2))
        n = len(control_points) - 1
        for i, point in enumerate(control_points):
            curve_points += np.outer(np.power(1 - t_values, n - i) * np.power(t_values, i) * 
                                    np.math.comb(n, i), point)
        return curve_points
    
    def loop(self):
        while not rospy.is_shutdown():
            if self.robot_points is None or self.robot_pose is None or self.scan_data is None:
                rospy.loginfo('Obstacle_avoidance_scan_node: No pose, points or scan_data found!')
                self.rate.sleep()
                continue
            scan_data = self.scan_data
            robot_position = [self.robot_pose.position.x, self.robot_pose.position.y]
            robot_orientation = [
                self.robot_pose.orientation.x,
                self.robot_pose.orientation.y,
                self.robot_pose.orientation.z,
                self.robot_pose.orientation.w
            ]
            
            room_points, index = self.calculate_cartesian_cooerdinates(scan_data, robot_position, robot_orientation)
            self.room_points = room_points
            
            robot_points = self.robot_points
            observation_radius = self.observation_radius
            min_threshold = self.min_threshold
            obstacle_points, obstacle_idx = self.select_obstacle_points(robot_points, room_points, index, observation_radius, min_threshold)

            if len(obstacle_points) < 2:
                self.rate.sleep()
                continue

            max_obstacle_distance = self.max_obstacle_distance
            clusters = self.clusterize_obstacles(obstacle_points, max_obstacle_distance)

            cumulative_distance = self.cumulative_distance
            density_gain = self.density_gain
            x_dot, y_dot = self.calculate_potential(robot_points, obstacle_points, clusters, obstacle_idx, cumulative_distance, density_gain)

            potential_msg = Point(x=x_dot, y=y_dot)
            self.potential_publisher.publish(potential_msg)
            self.rate.sleep()

def main():
    obs_avoidance = ObstacleAvoidanceScan()
    obs_avoidance.loop()

if __name__ == '__main__':
    main()
