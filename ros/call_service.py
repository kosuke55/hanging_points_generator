import rospy
import datetime
from std_srvs.srv import SetBool


class RosbagCallService():
    def __init__(self):
        self.file = 'service_time_list.txt'
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

    def setbool_service_call(self, service_name):
        rospy.wait_for_service(service_name, timeout=3)
        try:
            client = rospy.ServiceProxy(service_name, SetBool)
            print(client(True))
        except rospy.ServiceException as e:
            print('Service call failed: %s' % e)

    def check_elapsed(self):
        if self.next_time < self.current_time:
            rospy.sleep(5)
            for _ in range(3):
                self.setbool_service_call('/integrate_point_cloud')
                rospy.sleep(0.1)
            self.elapsed[self.index] = True
            if self.index != len(self.service_times) - 1:
                self.index += 1
                self.next_time = self.service_times[self.index]
            else:
                self.setbool_service_call('/create_mesh')
                rospy.sleep(5)
                rospy.signal_shutdown('rosbag finished')

    def timer_callback(self, event):
        self.get_current_time()
        self.check_elapsed()
        print(self.current_time, self.next_time, self.next_time - self.current_time, self.elapsed)


if __name__ == '__main__':
    rospy.init_node('rosbag_call_service', anonymous=False)
    rosbag_call_service = RosbagCallService()
    rospy.spin()