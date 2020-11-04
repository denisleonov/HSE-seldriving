#! /usr/bin/python

import numpy as np
import time
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from rospy import Publisher, Subscriber


class Follower:
    def __init__(self):
        self.pub = Publisher('/turtle_follower/cmd_vel', Twist, queue_size=1)
        Subscriber('/turtle_follower/pose', Pose, self.update_pose)
        Subscriber('/turtle1/pose', Pose, self.follow)
        self.pose = Pose()
        self.tol = 1e-2

        time.sleep(1)
        
    def update_pose(self, new_pose):
        self.pose = new_pose

    def follow(self, pose):
        if np.sqrt((self.pose.x - pose.x) ** 2 + (self.pose.y - pose.y) ** 2) > self.tol:
            return
        
        msg = Twist()
        
        steering_angle = np.arctan2(self.pose.y - pose.y, self.pose.x - pose.x) - self.pose.theta
        
        w = np.sign(steering_angle)
        if steering_angle > np.pi:
            w = -1
        elif steering_angle < -np.pi:
            w = 1
        
        msg.linear.x = 0.5
        msg.angular.z = -w * np.pi / 2
        
        self.pub.publish(msg)
        

rospy.init_node('follower')
Follower()
rospy.spin()