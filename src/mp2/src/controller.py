import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
import math
from util import euler_to_quaternion, quaternion_to_euler
import time

class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.prev_vel = 0
        self.L = 1.75 # Wheelbase, can be get from gem_control.py
        self.log_acceleration = True
        self.acc_count = 0
        self.x_log = []
        self.y_log = []
        self.acc_log = []

    def getModelState(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp


    # Tasks 1: Read the documentation https://docs.ros.org/en/fuerte/api/gazebo/html/msg/ModelState.html
    #       and extract yaw, velocity, vehicle_position_x, vehicle_position_y
    # Hint: you may use the the helper function(quaternion_to_euler()) we provide to convert from quaternion to euler
    def extract_vehicle_info(self, currentPose):

        ####################### TODO: Your TASK 1 code starts Here #######################
        pos_x, pos_y, vel, yaw = 0, 0, 0, 0
        #print(currentPose)

        pos_x = currentPose.pose.position.x
        pos_y = currentPose.pose.position.y
        vel =  np.linalg.norm([currentPose.twist.linear.x, currentPose.twist.linear.y, currentPose.twist.linear.z])
        _,_, yaw = quaternion_to_euler(currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z, currentPose.pose.orientation.w)
        # print(vel)

        ####################### TODO: Your Task 1 code ends Here #######################

        return pos_x, pos_y, vel, yaw # note that yaw is in radian

    # Task 2: Longtitudal Controller
    # Based on all unreached waypoints, and your current vehicle state, decide your velocity
    def curvature(self, x1, y1, x2, y2, x3, y3):
        a = abs(x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
        s1 = np.linalg.norm([ x1 - x2, y1- y2], 2)
        s2 = np.linalg.norm([x2 - x3, y2 -y3 ],2)
        s3 = np.linalg.norm([x1-x3, y1-y3],2)

        # print(s1, s2, s3)
        c = (4*a/(s1*s2*s3))*1000
        print("curvature: ", c)
        return c
    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):

        ####################### TODO: Your TASK 2 code starts Here #######################

        if(len(future_unreached_waypoints) >= 3):
            c = self.curvature(future_unreached_waypoints[0][0], future_unreached_waypoints[0][1], future_unreached_waypoints[1][0], future_unreached_waypoints[1][1], future_unreached_waypoints[2][0], future_unreached_waypoints[2][1])
            if(c >20):
                target_velocity = 10
            elif(20> c> 5):
                target_velocity = 14
            else:
                target_velocity = 18
                if abs(target_velocity - curr_vel) > 10:
                    target_velocity = 14
                    if abs(target_velocity - curr_vel) > 9:
                        target_velocity = 6
        else:
            target_velocity = 26

        ####################### TODO: Your TASK 2 code ends Here #######################
        print('target velocity', target_velocity)
        return target_velocity


    # Task 3: Lateral Controller (Pure Pursuit)
    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints):

        ####################### TODO: Your TASK 3 code starts Here #######################
        c = self.curvature(future_unreached_waypoints[0][0], future_unreached_waypoints[0][1], future_unreached_waypoints[1][0], future_unreached_waypoints[1][1], future_unreached_waypoints[2][0], future_unreached_waypoints[2][1])
        if(c >20):
            Lf = 7
        elif(20> c> 5):
            Lf = 12
        else:
            Lf = 15
        max_steering_angle = 0.5

        dx = target_point[0] - curr_x
        dy = target_point[1] - curr_y

        target_yaw = np.arctan2(dy, dx)

        target_steering = np.arctan2(2*self.L*np.sin(target_yaw-curr_yaw), Lf)

        target_steering = max(-max_steering_angle, min(target_steering, max_steering_angle))
        print('target steering ', target_steering)

        ####################### TODO: Your TASK 3 code starts Here #######################
        return target_steering


    def execute(self, currentPose, target_point, future_unreached_waypoints):
        # Compute the control input to the vehicle according to the
        # current and reference pose of the vehicle
        # Input:
        #   currentPose: ModelState, the current state of the vehicle
        #   target_point: [target_x, target_y]
        #   future_unreached_waypoints: a list of future waypoints[[target_x, target_y]]
        # Output: None

        curr_x, curr_y, curr_vel, curr_yaw = self.extract_vehicle_info(currentPose)
        self.x_log.append(curr_x)
        self.y_log.append(curr_y)

        # Acceleration Profile
        if self.log_acceleration:
            print('current velocity ', curr_vel)
            print('prev_vel ', self.prev_vel)
            acceleration = (curr_vel- self.prev_vel) * 100 # Since we are running in 100Hz
            print('acceleration ', acceleration)
            self.prev_vel = curr_vel
            
            if acceleration > 5:
                self.acc_count += 1
            print('acc_count ', self.acc_count)
            self.acc_log.append(acceleration)



        target_velocity = self.longititudal_controller(curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints)
        target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints)



        #Pack computed velocity and steering angle into Ackermann command
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = target_velocity
        newAckermannCmd.steering_angle = target_steering

        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        # plt.plot(self.x_log, self.y_log)
        # plt.plot(np.arange(len(self.acc_log)), self.acc_log)
        np.save('x_pos', self.x_log)
        np.save('y_pos', self.y_log)
        np.save('acc', self.acc_log)
       
