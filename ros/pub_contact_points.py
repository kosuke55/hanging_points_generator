#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import os
import rospy
import sys

from geometry_msgs.msg import PoseArray, Pose
from skrobot.coordinates.math import matrix2quaternion
from std_srvs.srv import SetBool, SetBoolResponse


class ContactPointsPublisher():
    def __init__(self):
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.contact_points_json = rospy.get_param(
            '~contact_points_json', 'save_dir/contact_points.json')
        self.contact_points_json = os.path.join(self.current_dir,
                                                '..',
                                                self.contact_points_json)
        self.gripper_frame = rospy.get_param(
            '~gripper_frame', "/l_gripper_tool_frame")
        self.contact_points_pose_array = PoseArray()
        self.pub_contact_points = rospy.Publisher(
            "~output", PoseArray, queue_size=10)
        self.service()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def service(self):
        self.load_contact_points_service \
            = rospy.Service('load_contact_points',
                            SetBool,
                            self.load_contact_poists)

    def load_contact_poists(self, req):
        self.contact_points_pose_array = PoseArray()
        contact_points_list = json.load(
            open(self.contact_points_json, 'r'))['contact_points']
        for i, cp in enumerate(contact_points_list):
            rotation_quaternion = matrix2quaternion(cp[1:])
            contact_point_pose = Pose()
            contact_point_pose.position.x = cp[0][0]
            contact_point_pose.position.y = cp[0][1]
            contact_point_pose.position.z = cp[0][2]
            contact_point_pose.orientation.x = rotation_quaternion[1]
            contact_point_pose.orientation.y = rotation_quaternion[2]
            contact_point_pose.orientation.z = rotation_quaternion[3]
            contact_point_pose.orientation.w = rotation_quaternion[0]
            self.contact_points_pose_array.poses.append(contact_point_pose)
        print(self.contact_points_pose_array)
        return SetBoolResponse(True, 'load contact points')

    def timer_callback(self, timer):
        self.contact_points_pose_array.header.stamp = rospy.Time.now()
        self.contact_points_pose_array.header.frame_id = self.gripper_frame
        self.pub_contact_points.publish(self.contact_points_pose_array)

    def run(self):
        try:
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                rate.sleep()
        except KeyboardInterrupt:
            sys.exit(0)


if __name__ == '__main__':
    rospy.init_node('contact_points_publisher', anonymous=False)
    contact_points_publiser = ContactPointsPublisher()
    contact_points_publiser.run()
