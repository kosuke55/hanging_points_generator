#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import image_geometry
import message_filters
import numpy as np
import open3d as o3d
import os
import pathlib2
import rospy
import skrobot
import subprocess
import sys
import tf

from cv_bridge import CvBridge
from hanging_points_generator import hanging_points_generator
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import SetBool, SetBoolResponse


class CreateMesh():
    def __init__(self):
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.input_color = rospy.get_param(
            '~input_color',
            '/head_mount_kinect/hd/image_color_rect_repub_desktop')
        self.input_depth = rospy.get_param(
            '~input_depth',
            '/head_mount_kinect/hd/image_depth_rect_repub_desktop')
        self.input_mask = rospy.get_param(
            '~input_mask',
            '/point_indices_to_mask_image_gripper/output')

        self.camera_frame = rospy.get_param(
            '~camera_frame', '/head_mount_kinect_rgb_link')
        self.gripper_frame = rospy.get_param(
            '~gripper_frame', '/l_gripper_tool_frame')

        self.camera_info_msg = rospy.get_param(
            '~camera_info', '/head_mount_kinect/hd/camera_info')

        self.save_raw_img = rospy.get_param(
            '~save_raw_img', True)

        self.save_dir = rospy.get_param(
            '~save_dir', 'save_dir/')

        self.save_dir = os.path.join(self.current_dir, '..', self.save_dir)
        pathlib2.Path(os.path.join(self.save_dir, 'raw')).mkdir(
            parents=True, exist_ok=True)
        pathlib2.Path(os.path.join(self.save_dir, 'camera_pose')).mkdir(
            parents=True, exist_ok=True)

        self.camera_info = None
        self.camera_model = image_geometry.cameramodels.PinholeCameraModel()
        self.color = None
        self.depth = None
        self.header = None
        self.stanby = False
        self.callback_lock = False

        self.load_camera_info()
        self.subscribe()
        self.bridge = CvBridge()

        self.lis = tf.TransformListener()
        self.integrate_count = 0
        self.voxel_length = 0.002
        self.volume = o3d.integration.ScalableTSDFVolume(
            voxel_length=0.0025,
            sdf_trunc=0.01,
            color_type=o3d.integration.TSDFVolumeColorType.RGB8)
        self.service()

    def load_camera_info(self):
        self.camera_info = rospy.wait_for_message(
            self.camera_info_msg, CameraInfo)
        self.camera_model.fromCameraInfo(self.camera_info)
        self.intrinsic = o3d.camera.PinholeCameraIntrinsic()
        self.intrinsic.set_intrinsics(
            self.camera_model.width,
            self.camera_model.height,
            self.camera_model.fx(),
            self.camera_model.fy(),
            self.camera_model.cx(),
            self.camera_model.cy())
        print('load camera model')
        np.savetxt(os.path.join(self.save_dir, 'camera_pose/intrinsic.txt'),
                   self.intrinsic.intrinsic_matrix)

    def subscribe(self):
        sub_color = message_filters.Subscriber(
            self.input_color, Image, queue_size=10)
        sub_depth = message_filters.Subscriber(
            self.input_depth, Image, queue_size=10)
        sub_mask = message_filters.Subscriber(
            self.input_mask, Image, queue_size=10)
        self.subs = [sub_color, sub_depth, sub_mask]
        sync = message_filters.TimeSynchronizer(
            fs=self.subs, queue_size=100)
        sync.registerCallback(self.callback)

    def callback(self, rgb_msg, depth_msg, mask_msg):
        if self.callback_lock:
            return
        self.mask = self.bridge.imgmsg_to_cv2(mask_msg, 'mono8')
        self.color = self.bridge.imgmsg_to_cv2(rgb_msg, 'rgb8')
        self.depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        self.header = rgb_msg.header

        if not self.stanby:
            rospy.loginfo('Stanby!')
            self.stanby = True

    def service(self):
        self.integrate_service = rospy.Service('integrate_point_cloud',
                                               SetBool,
                                               self.integrate_point_cloud)
        self.create_mesh_service = rospy.Service('create_mesh',
                                                 SetBool,
                                                 self.create_mesh)
        self.meshfix_service = rospy.Service('meshfix',
                                             SetBool,
                                             self.meshfix)
        self.reset_volume_service = rospy.Service('reset_volume',
                                                  SetBool,
                                                  self.reset_volume)
        self.generate_hanging_points = rospy.Service(
            'generate_hanging_points',
            SetBool,
            self.generate_hanging_points)

    def integrate_point_cloud(self, req):
        if self.header is None:
            rospy.logwarn('No callback')
            return
        self.callback_lock = True

        self.color_clip = self.color.copy()
        self.depth_clip = self.depth.copy()

        mask_morph_open = cv2.morphologyEx(
            self.mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask_morph_close = cv2.morphologyEx(
            mask_morph_open, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        self.color_clip[mask_morph_close == 0] = [0, 0, 0]
        self.depth_clip[mask_morph_open == 0] = 0

        rospy.loginfo('integrate count: %d', self.integrate_count)

        try:
            self.lis.waitForTransform(
                self.gripper_frame, self.header.frame_id,
                self.header.stamp, rospy.Duration(5))
            trans, rot = self.lis.lookupTransform(
                self.gripper_frame, self.header.frame_id,
                self.header.stamp)

            camera_pose = skrobot.coordinates.Coordinates(
                pos=trans,
                rot=skrobot.coordinates.math.xyzw2wxyz(rot))

            np.savetxt(
                os.path.join(self.save_dir,
                             'camera_pose/camera_pose{:03}.txt'.format(
                                 self.integrate_count)),
                camera_pose.T())

            cv2.imwrite(os.path.join(self.save_dir, 'color{:03}.png'.format(
                self.integrate_count)), cv2.cvtColor(
                    self.color_clip.astype(np.uint8), cv2.COLOR_BGR2RGB))

            cv2.imwrite(os.path.join(self.save_dir, 'depth{:03}.png'.format(
                self.integrate_count)), self.depth_clip.astype(np.uint16))

            if self.save_raw_img:
                cv2.imwrite(os.path.join(
                    self.save_dir,
                    'raw/color_raw{:03}.png'.format(
                        self.integrate_count)),
                            cv2.cvtColor(
                                self.color.astype(np.uint8),
                                cv2.COLOR_BGR2RGB))
                cv2.imwrite(os.path.join(
                    self.save_dir,
                    'raw/depth_raw{:03}.png'.format(
                        self.integrate_count)), self.depth.astype(np.uint16))

            cv2.imwrite(os.path.join(
                self.save_dir,
                'mask{:03}.png'.format(
                    self.integrate_count)),
                        mask_morph_close.astype(np.uint8))

            color = o3d.geometry.Image(self.color_clip.astype(np.uint8))
            depth = o3d.geometry.Image(self.depth_clip.astype(np.uint16))
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color, depth, depth_trunc=4.0,
                convert_rgb_to_intensity=False)
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                rgbd, self.intrinsic)
            pcd = pcd.voxel_down_sample(self.voxel_length)
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=0.1, max_nn=30))
            # o3d.visualization.draw_geometries([pcd])

            if self.integrate_count == 0:
                self.target_pcd = pcd
                self.target_camera_pose = camera_pose
                camera_pose_icp = camera_pose

            else:
                trans_init = self.target_camera_pose.copy_worldcoords(
                ).inverse_transformation().transform(camera_pose)
                result_icp = o3d.registration.registration_icp(
                    pcd, self.target_pcd, 0.01, trans_init.T(),
                    o3d.registration.TransformationEstimationPointToPoint())
                icp_coords = skrobot.coordinates.Coordinates(
                    pos=result_icp.transformation[:3, 3],
                    rot=result_icp.transformation[:3, :3])
                camera_pose_icp = self.target_camera_pose.copy_worldcoords(
                ).transform(icp_coords)
                pcd.transform(result_icp.transformation)
                self.target_pcd += pcd
                self.target_pcd.remove_statistical_outlier(nb_neighbors=100,
                                                           std_ratio=0.001)
                # o3d.visualization.draw_geometries([self.target_pcd])

            np.savetxt(
                os.path.join(
                    self.save_dir,
                    'camera_pose/camera_pose_icp{:03}.txt'.format(
                        self.integrate_count)), camera_pose_icp.T())

            # Save camera pose and intrinsic for texture-mapping
            with open(os.path.join(
                    self.save_dir,
                    'color{:03}.txt'.format(self.integrate_count)), 'w') as f:
                np.savetxt(f, np.concatenate(
                    [camera_pose_icp.T()[:3, 3][None, :],
                     camera_pose_icp.T()[:3, :3]],
                    axis=0))
                np.savetxt(f, [self.camera_model.fx()])
                np.savetxt(f, [self.camera_model.fy()])
                np.savetxt(f, [self.camera_model.cx()])
                np.savetxt(f, [self.camera_model.cy()])
                np.savetxt(f, [self.camera_model.height])
                np.savetxt(f, [self.camera_model.width])

            self.volume.integrate(
                rgbd,
                self.intrinsic,
                np.linalg.inv(camera_pose_icp.T()))

            self.integrate_count += 1
            self.callback_lock = False
            return SetBoolResponse(True, 'success integrate point cloud')

        except Exception:
            self.callback_lock = False
            rospy.logwarn(
                'failed listen transform')
            return SetBoolResponse(False, 'failed listen transform')

    def create_mesh(self, req):
        mesh = self.volume.extract_triangle_mesh()
        mesh.compute_vertex_normals()
        o3d.visualization.draw_geometries([mesh])
        o3d.io.write_triangle_mesh(os.path.join(self.save_dir,
                                                'obj.ply'), mesh)
        return SetBoolResponse(True, 'success create mesh')

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
        return SetBoolResponse(True, 'meshfix')

    def reset_volume(self, req):
        self.volume.reset()
        self.integrate_count = 0
        return SetBoolResponse(True, 'reset volume')

    def generate_hanging_points(self, req):
        hanging_points_generator.generate(
            urdf_file=os.path.join(self.save_dir, 'base.urdf'),
            required_points_num=1,
            enable_gui='False',
            save_dir=self.save_dir)
        return SetBoolResponse(True, 'success create mesh')

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
