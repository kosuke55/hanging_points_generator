#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import os.path as osp
import pathlib2
import subprocess
import sys

import cameramodels
import message_filters
import numpy as np
import open3d as o3d
import rospy
from cv_bridge import CvBridge
from skrobot.interfaces.ros.transform_listener import TransformListener
from skrobot.interfaces.ros.tf_utils import tf_pose_to_coords

from hanging_points_generator import hp_generator
from hanging_points_generator.create_mesh import apply_mask_images
from hanging_points_generator.create_mesh import create_mesh_tsdf
from hanging_points_generator.create_mesh import create_mesh_voxelize_marcing_cubes
from hanging_points_generator.create_mesh import create_urdf
from hanging_points_generator.create_mesh import get_largest_components_mesh
from hanging_points_generator.create_mesh import dbscan
from hanging_points_generator.create_mesh import depths_mean_filter
from hanging_points_generator.create_mesh import get_pcds
from hanging_points_generator.create_mesh import icp_registration
from hanging_points_generator.create_mesh import mask_to_roi
from hanging_points_generator.create_mesh import np_to_o3d_images
from hanging_points_generator.create_mesh import preprocess_masks
from hanging_points_generator.create_mesh import save_camera_poses
from hanging_points_generator.create_mesh import save_images
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger, TriggerResponse


class CreateMesh():
    def __init__(self):
        self.current_dir = os.path.dirname(os.path.abspath(__file__))

        self.camera_frame = rospy.get_param(
            '~camera_frame', 'head_mount_kinect_rgb_link')
        self.world_frame = rospy.get_param(
            '~world_frame', 'l_gripper_tool_frame')

        self.save_raw_img = rospy.get_param(
            '~save_raw_img', True)
        self.save_dir = rospy.get_param(
            '~save_dir', 'save_dir/')
        self.use_tf2 = rospy.get_param(
            '~use_tf2', False)

        self.eps = rospy.get_param(
            '~eps', 0.01)
        self.min_points = rospy.get_param(
            '~min_points', 30)
        self.voxel_length = rospy.get_param(
            '~voxel_length', 0.002)
        self.crop = rospy.get_param(
            '~crop', True)

        self.save_dir = os.path.join(self.current_dir, '..', self.save_dir)
        pathlib2.Path(os.path.join(self.save_dir, 'raw')).mkdir(
            parents=True, exist_ok=True)
        pathlib2.Path(os.path.join(self.save_dir, 'camera_pose')).mkdir(
            parents=True, exist_ok=True)

        self._reset()
        self.load_camera_info()
        self.subscribe()
        self.bridge = CvBridge()

        self.lis = TransformListener(self.use_tf2)
        self.service()

    def _reset(self):
        self.color = None
        self.depth = None
        self.pcd_icp = None
        self.pcds = None
        self.camera_pose = None

        self.color_list = []
        self.depth_list = []
        self.mask_list = []
        self.camera_pose_list = []
        self.intrinsic_list = []
        self.header = None
        self.stanby = False
        self.callback_lock = False

    def reset(self, req):
        self._reset()
        return TriggerResponse(True, 'reset')

    def load_camera_info(self):
        self.camera_info = rospy.wait_for_message(
            '~camera_info', CameraInfo)
        self.camera_model \
            = cameramodels.PinholeCameraModel.from_camera_info(
                self.camera_info)
        self.intrinsic = self.camera_model.open3d_intrinsic
        print('load camera model')
        np.savetxt(os.path.join(self.save_dir, 'camera_pose/intrinsic.txt'),
                   self.intrinsic.intrinsic_matrix)

    def subscribe(self):
        sub_color = message_filters.Subscriber(
            '~input_color', Image, queue_size=1, buff_size=2**24)
        sub_depth = message_filters.Subscriber(
            '~input_depth', Image, queue_size=1, buff_size=2**24)
        sub_mask = message_filters.Subscriber(
            '~input_mask', Image, queue_size=1, buff_size=2**24)
        self.subs = [sub_color, sub_depth, sub_mask]
        sync = message_filters.TimeSynchronizer(
            fs=self.subs, queue_size=100)
        sync.registerCallback(self.callback)

    def callback(self, rgb_msg, depth_msg, mask_msg):
        if self.callback_lock:
            print('callback is locked')
            return
        self.header = rgb_msg.header
        self.mask = self.bridge.imgmsg_to_cv2(mask_msg, 'mono8')
        self.color = self.bridge.imgmsg_to_cv2(rgb_msg, 'rgb8')
        self.depth = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')

        if not self.stanby:
            rospy.loginfo('Stanby!')
            self.stanby = True

    def service(self):
        self.integrate_service = rospy.Service(
            'store_images', Trigger,
            self.store_images)
        self.icp_registration_service = rospy.Service(
            'icp_registration', Trigger,
            self.icp_registration)
        self.create_urdf_service = rospy.Service(
            'create_urdf', Trigger,
            self.create_urdf)
        self.create_mesh_tsdf_service = rospy.Service(
            'create_mesh_tsdf', Trigger,
            self.create_mesh_tsdf)
        self.create_mesh_voxelize_marcning_cubes_service = rospy.Service(
            'create_mesh_voxelize_marcning_cubes', Trigger,
            self.create_mesh_voxelize_marcing_cubes)
        self.dbscan_service = rospy.Service(
            'dbscan', Trigger,
            self.dbscan)
        self.save_service = rospy.Service(
            'save', Trigger,
            self.save)
        self.meshfix_service = rospy.Service(
            'meshfix', Trigger,
            self.meshfix)
        self.reset_service = rospy.Service(
            'reset', Trigger,
            self.reset)
        self.generate_hanging_points = rospy.Service(
            'generate_hanging_points', Trigger,
            self.generate_hanging_points)

    def preprocess_masks(self):
        self.color_mask_list = preprocess_masks(self.mask_list)
        self.depth_mask_list = preprocess_masks(
            self.mask_list, morph_close=False)

    def crop_images(self, preprocess_mask=True):
        if preprocess_mask:
            self.preprocess_masks()
        self.cropped_color_list = apply_mask_images(
            self.color_list, self.mask_list, self.crop)
        self.cropped_depth_list = depths_mean_filter(
            apply_mask_images(self.depth_list, self.mask_list, self.crop))

    def store_images(self, req):
        rospy.loginfo('Store {} images'.format(len(self.color_list)))
        if self.header is None:
            rospy.logwarn('No callback')
            return
        try:
            tf_pose = self.lis.lookup_transform(
                self.world_frame, self.header.frame_id,
                self.header.stamp, rospy.Duration(1))
        except Exception:
            rospy.logwarn('Fail to listeen transform')
            return TriggerResponse(False, 'fail to store image')

        self.camera_pose = None
        if tf_pose:
            self.camera_pose = tf_pose_to_coords(tf_pose)

        if not self.camera_pose:
            rospy.logwarn('No tf')
            return TriggerResponse(False, 'fail to srore image')

        self.callback_lock = True
        self.color_list.append(self.color.copy())
        self.depth_list.append(self.depth.copy())
        self.mask_list.append(self.mask.copy())

        if self.crop:
            self.camera_model.roi = mask_to_roi(self.mask)
        self.intrinsic_list.append(self.camera_model.open3d_intrinsic)

        self.camera_pose_list.append(self.camera_pose.copy_worldcoords())

        self.callback_lock = False
        self.header = None

        return TriggerResponse(True, 'store_images')

    def get_pcds(self):
        self.cropped_color_list = np_to_o3d_images(self.cropped_color_list)
        self.cropped_depth_list = np_to_o3d_images(self.cropped_depth_list)
        self.pcds = get_pcds(
            self.cropped_color_list,
            self.cropped_depth_list,
            self.intrinsic_list)

    def save(self, req):
        self.save_images()
        save_camera_poses(
            osp.join(self.save_dir, 'camera_pose'), 'camera_pose',
            self.camera_pose_list)
        o3d.io.write_point_cloud(
            osp.join(self.save_dir, 'icp_result.pcd'), self.pcd_icp)
        self.mesh_tsdf.export(osp.join(self.save_dir, 'tsdf_obj.ply'))
        self.mesh_voxelize_marching_cubes.export(
            osp.join(self.save_dir, 'voxelized_mc_obj.ply'))
        return TriggerResponse(True, 'save')

    def save_images(self):
        save_images(self.save_dir, 'color_raw', self.color_list, 'bgr')
        save_images(self.save_dir, 'color', self.cropped_color_list)
        save_images(self.save_dir, 'depth_raw', self.depth_list)
        save_images(self.save_dir, 'depth', self.cropped_depth_list)
        save_images(self.save_dir, 'mask', self.mask_list)

    def icp_registration(self, req):
        self.crop_images()
        self.get_pcds()

        self.pcd_icp, self.camera_pose_icp_list, self.obj_poses \
            = icp_registration(self.pcds, self.camera_pose_list,
                               voxel_size=self.voxel_length)
        o3d.visualization.draw_geometries([self.pcd_icp])
        return TriggerResponse(True, 'icp_registration')

    def dbscan(self, req):
        self.pcd_icp = dbscan(self.pcd_icp, self.eps, self.min_points)
        o3d.visualization.draw_geometries([self.pcd_icp])
        return TriggerResponse(True, 'dbscan')

    def create_mesh_tsdf(self, req):
        rospy.loginfo('create_mesh_tsdf')
        self.mesh_tsdf = create_mesh_tsdf(
            self.cropped_color_list, self.cropped_depth_list,
            self.intrinsic_list, self.camera_pose_icp_list,
            connected_components=True)
        self.mesh_tsdf.show()

        return TriggerResponse(True, 'success create mesh')

    def create_mesh_voxelize_marcing_cubes(self, req):
        self.mesh_voxelize_marching_cubes \
            = create_mesh_voxelize_marcing_cubes(
                self.pcd_icp, smoothing_method='laplacian')
        self.mesh_voxelize_marching_cubes.show()
        return TriggerResponse(True, 'create_mesh_voxelize_marcing_cubes')

    def create_urdf(self, req):
        create_urdf(self.mesh_tsdf, osp.join(self.save_dir, 'tsdf_urdf'))
        create_urdf(self.mesh_voxelize_marching_cubes, osp.join(
            self.save_dir, 'mesh_voxelize_marching_cubes_urdf'))
        return TriggerResponse(True, 'create_urdf')

    def meshfix(self, req):
        subprocess.call(
            ['python3',
             os.path.join(
                 self.current_dir,
                 '../hanging_points_generator/meshfix.py'),
             '-i',
             os.path.join(self.save_dir, 'obj.ply'),
             '-o',
             self.save_dir])
        return TriggerResponse(True, 'meshfix')

    def generate_hanging_points(self, req):
        hp_generator.generate(
            urdf_file=os.path.join(self.save_dir, 'base.urdf'),
            required_points_num=1,
            enable_gui='False',
            viz_obj='False',
            save_dir=self.save_dir)
        return TriggerResponse(True, 'success create mesh')

    def run(self):
        try:
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                rate.sleep()
        except KeyboardInterrupt:
            sys.exit(0)


if __name__ == '__main__':
    rospy.init_node('create_mesh', anonymous=False)
    create_mesh = CreateMesh()
    create_mesh.run()
