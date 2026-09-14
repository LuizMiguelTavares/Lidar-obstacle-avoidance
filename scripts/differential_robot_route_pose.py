#!/usr/bin/env python3

import rospy
import tf.transformations as tf
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
from sensor_msgs.msg import ChannelFloat32
from nav_msgs.msg import Odometry, Path

from aurora_py.differential_robot_controller import solver_bot_controller, pioneer_controller

import numpy as np

class DifferentialController:
    def __init__(self):
        rospy.init_node('differential_controller')

        # Variables initialization
        self.x_dot, self.y_dot = 0.0, 0.0
        self.btn_emergencia = False
        self.btn_emergencia_is_on = False
        self.robot_path_is_on = False
        self.robot_pose_is_on = False
        
        # Getting Params
        gains = rospy.get_param('~gains', None)
        self.pgains = [gains['linear'], gains['angular']]
        self.line_of_sight_dist = rospy.get_param('~line_of_sight_dist', None)
        self.a = rospy.get_param('~a', 0.15)
        self.robot_type = rospy.get_param('~robot_type', None)

        if self.robot_type is None:
            raise TypeError(f"You need to provide a robot type.")

        if self.robot_type == 'Solverbot':
            self.publisher = rospy.Publisher('/md49_driverCommand',
                                            ChannelFloat32,
                                            queue_size=10)
            
            self.differential_robot_controller = solver_bot_controller
        
        if self.robot_type == 'Pioneer':
            self.publisher = rospy.Publisher('/RosAria/cmd_vel',
                                            Twist,
                                            queue_size=10)
            
            self.differential_robot_controller = pioneer_controller
            
        self.namespace = rospy.get_namespace()

        self.pose_subscriber = rospy.Subscriber(f"/RosAria/pose", 
                                                Odometry, 
                                                self.pose_callback)
        
        self.path_subscriber = rospy.Subscriber(f"{self.namespace}route", 
                                                    Path, 
                                                    self.route_callback)

        self.potential_subscriber = rospy.Subscriber(f"{self.namespace}potential", 
                                                    Point, 
                                                    self.potential_callback)           

        self.emergency_flag_subscriber = rospy.Subscriber('/emergency_flag',
                                            Bool,
                                            self.emergency_button_callback,
                                            queue_size=10)

        self.rate = rospy.Rate(rospy.get_param('~control_frequency', 30))

        rospy.loginfo('Differential robot navigation node started')

    def pose_callback(self, pose_data):
        self.robot_pose_is_on = True
        pose = pose_data.pose.pose
        center_robot_x = pose.position.x
        center_robot_y = pose.position.y
        center_robot_z = pose.position.z

        orientation = pose.orientation
        _, _, self.robot_yaw = tf.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        
        self.robot_x = center_robot_x + self.a*np.cos(self.robot_yaw)
        self.robot_y = center_robot_y + self.a*np.sin(self.robot_yaw)

    def route_callback(self, route_data):
        self.robot_path_is_on = True
        self.route = [[p.pose.position.x, p.pose.position.y] for p in route_data.poses]
    
    def potential_callback(self, potential_data):
        self.x_dot = potential_data.x
        self.y_dot = potential_data.y

    def emergency_button_callback(self, emergency):
        self.btn_emergencia_is_on = True
        if emergency.data:
            self.btn_emergencia = True
            rospy.loginfo('Robot stopping by Emergency')
            rospy.loginfo('Sending emergency stop command')

            if self.robot_type == 'Pioneer':
                for _ in range(10):
                    stop_cmd = Twist()
                    stop_cmd.linear.x = 0.0
                    stop_cmd.linear.y = 0.0
                    stop_cmd.linear.z = 0.0
                    stop_cmd.angular.x = 0.0
                    stop_cmd.angular.y = 0.0
                    stop_cmd.angular.z = 0.0
                    # Publish the Twist message to stop the robot
                    self.publisher.publish(stop_cmd)

                rospy.signal_shutdown("Pioneer Emergency stop")

            if self.robot_type == 'Solverbot':
                for _ in range(10):
                    message = [0.0, 0.0] 
                    stop_cmd = ChannelFloat32(values=message)
                    # Publish the Twist message to stop the robot
                    self.publisher.publish(stop_cmd)

                rospy.signal_shutdown("SolverBot Emergency stop")
    
    def find_line_of_sight(self, robot_pose, route, line_of_sight_dist):
        min_arg = self._closest_point_index(robot_pose, route)
        route = np.array(route)
        cum_dist = 0
        aux_arg = min_arg
        while cum_dist<line_of_sight_dist:
            try:
                cum_dist += np.linalg.norm(route[aux_arg] - route[aux_arg+1])
                aux_arg += 1
            except:
                cum_dist += np.linalg.norm(route[-1] - route[0])
                aux_arg = 0
        
        return route[aux_arg], route[min_arg]
    
    # def cummulative_route(self, route, closest_point):
    #     line_of_sight_dist = 0.2
    #     min_arg = self._closest_point_index(closest_point, route)
    #     route = np.array(route)
    #     cum_dist = 0
    #     aux_arg = min_arg
    #     args = [min_arg]
    #     while cum_dist<line_of_sight_dist:
    #         try:
    #             cum_dist += np.linalg.norm(route[aux_arg] - route[aux_arg+1])
    #             aux_arg += 1
    #             args.append(aux_arg)
    #         except:
    #             cum_dist += np.linalg.norm(route[-1] - route[0])
    #             aux_arg = 0
    #             args.append(aux_arg)
        
    #     aux_arg = min_arg
    #     cum_dist = 0
    #     while cum_dist<line_of_sight_dist:
    #         cum_dist += np.linalg.norm(route[aux_arg] - route[aux_arg-1])
    #         aux_arg -= 1
    #         if aux_arg == -1:
    #             aux_arg = len(route)
    #         args.append(aux_arg)


    def _closest_point_index(self, robot_pose, route):
        route_array = np.array(route)
        differences = route_array - robot_pose
        distances = np.linalg.norm(differences, axis=1)
        
        return np.argmin(distances)

    def control_loop(self):
        while not rospy.is_shutdown():
            
            if self.robot_path_is_on == False or self.btn_emergencia_is_on == False or self.robot_pose_is_on ==False:
                rospy.loginfo('No path or emergency button found')
                self.rate.sleep()
                continue

            self.current_time = rospy.Time.now()
        
            if self.btn_emergencia:
                self.rate.sleep()
                continue

            x_dot, y_dot = self.x_dot, self.y_dot

            robot_pose = np.array([self.robot_x, self.robot_y, self.robot_yaw])

            line_of_sight_dist = self.line_of_sight_dist
            
            route = self.route

            desired_point, closest_point = self.find_line_of_sight([self.robot_x, self.robot_y], route, line_of_sight_dist)

            desired = [desired_point[0]   ,   desired_point[1], 
                            x_dot         ,        y_dot      ]
            
            gains = self.pgains
            a = self.a

            if self.robot_type == 'Solverbot':
                right_wheel, left_wheel = self.differential_robot_controller(robot_pose, desired, gains=gains, a=a)
                # right_wheel = Float32(data=right_wheel)
                # left_wheel = Float32(data=left_wheel)
                message = [right_wheel, left_wheel] 
                ctrl_msg = ChannelFloat32(values=message)
            
            if self.robot_type == 'Pioneer':
                reference_linear_velocity, reference_angular_velocity = self.differential_robot_controller(robot_pose, desired, gains=gains, limits=[0.5, 0.5], a=a)
                ctrl_msg = Twist()
                ctrl_msg.linear.x = reference_linear_velocity
                ctrl_msg.linear.y = 0.0
                ctrl_msg.linear.z = 0.0
                ctrl_msg.angular.x = 0.0
                ctrl_msg.angular.y = 0.0
                ctrl_msg.angular.z = reference_angular_velocity
                # rospy.loginfo('Linear Velocity: ' + str(reference_linear_velocity) + ', Angular Velocity: ' + str(reference_angular_velocity))
            
            self.publisher.publish(ctrl_msg)
            # rospy.loginfo(ctrl_msg)
            self.rate.sleep()

def main():
    try:
        robot = DifferentialController()
        robot.control_loop()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()