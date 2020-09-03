import os
import os.path as osp
import unittest
import shutil

import open3d as o3d

from hanging_points_generator.create_mesh import create_mesh_tsdf
from hanging_points_generator.create_mesh import icp_registration
from hanging_points_generator.create_mesh import create_mesh_voxelize_marcing_cubes
from hanging_points_generator.create_mesh import create_urdf

data_dir = osp.join(osp.abspath(osp.dirname(__file__)), 'data')
test_data_dir = osp.join(osp.abspath(osp.dirname(__file__)), 'test_data')
output_dir = osp.join(test_data_dir, 'output')
shutil.copytree(data_dir, test_data_dir)
os.makedirs(output_dir)


class TestCreateMesh(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.scenes = 2

    def test_0_icp(self):
        camera_poses_icp, pcd = icp_registration(
            test_data_dir, self.scenes)
        o3d.io.write_point_cloud(
            osp.join(test_data_dir, 'icp_result.pcd'), pcd)

    def test_tsdf(self):
        create_mesh_tsdf(test_data_dir, self.scenes)

    def test_voxelize_marching_cubes(self):
        pcd = o3d.io.read_point_cloud(
            osp.join(test_data_dir, 'icp_result.pcd'))
        create_mesh_voxelize_marcing_cubes(
            pcd, voxel_size=0.004)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(test_data_dir)
