#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os.path as osp
import rospy
from std_srvs.srv import Trigger


class RosbagCallService():
    def __init__(self):
        self.current_dir = osp.dirname(osp.abspath(__file__))
        self.file = osp.join(
            self.current_dir,
            '../create_mesh_sample_rosbag/service_time_list.txt')
        self.read_time()
        self.index = 0
        self.next_time = self.service_times[self.index]
        self.elapsed = [False] * len(self.service_times)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)

    def read_time(self):
        with open(self.file, mode='r') as f:
            times = f.readlines()

        self.service_times = list(
            map(lambda x:
                float(x.replace('\n', '')), times))

    def get_current_time(self):
        self.current_time = rospy.Time.now().to_time()

    def service_call(self, service_name):
        rospy.wait_for_service(service_name, timeout=3)
        try:
            client = rospy.ServiceProxy(service_name, Trigger)
            print(client())
        except rospy.ServiceException as e:
            print('Service call failed: %s' % e)

    def check_elapsed(self):
        if self.next_time < self.current_time:
            rospy.sleep(1)
            # self.service_call('/store_images')
            for _ in range(3):
                self.service_call('/store_images')
                rospy.sleep(0.2)
            self.elapsed[self.index] = True
            if self.index != len(self.service_times) - 1:
                self.index += 1
                self.next_time = self.service_times[self.index]
            else:
                self.service_call('/icp_registration')
                self.service_call('/dbscan')
                self.service_call('/create_mesh_tsdf')
                self.service_call('/create_mesh_voxelize_marcning_cubes')
                self.service_call('/save_images')
                rospy.sleep(5)
                rospy.signal_shutdown('rosbag finished')

    def timer_callback(self, event):
        self.get_current_time()
        self.check_elapsed()
        print(
            self.current_time,
            self.next_time,
            self.next_time - self.current_time,
            self.elapsed)


if __name__ == '__main__':
    rospy.init_node('rosbag_call_service', anonymous=False)
    rosbag_call_service = RosbagCallService()
    rospy.spin()
