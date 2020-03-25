#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import rospy
import sys

from geometry_msgs.msg import PoseArray, Pose
from std_srvs.srv import SetBool


class Contact_points_publiser():
    def __init__(self):
        self.contact_points_json = rospy.get_param(
            '~contact_points_json', 'contact_points.json')
        self.gripper_frame = rospy.get_param(
            '~gripper', "/l_gripper_tool_frame")
        self.contact_points_pose_array = PoseArray()
        self.pub_contact_points = rospy.Publisher(
            "/contact_points", PoseArray, queue_size=10)
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
            contact_point_pose = Pose()
            contact_point_pose.position.x = cp[0]
            contact_point_pose.position.y = cp[1]
            contact_point_pose.position.z = cp[2]
            contact_point_pose.orientation.x = 0
            contact_point_pose.orientation.y = 0
            contact_point_pose.orientation.z = 0
            contact_point_pose.orientation.w = 1
            self.contact_points_pose_array.poses.append(contact_point_pose)

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
    contact_points_publiser = Contact_points_publiser()
    contact_points_publiser.run()
