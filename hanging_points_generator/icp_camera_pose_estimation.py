#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import copy
import numpy as np
import open3d as o3d
import os
import skrobot


def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--input', '-i', type=str,
                        help='input dir',
                        default='save_dir/')
    args = parser.parse_args()

    width = 1920
    height = 1080
    intrinsic_np = np.loadtxt(os.path.join(
        args.input, 'camera_pose/intrinsic.txt'))

    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    fx = intrinsic_np[0, 0]
    fy = intrinsic_np[1, 1]
    cx = intrinsic_np[0, 2]
    cy = intrinsic_np[1, 2]

    intrinsic.set_intrinsics(
        width, height, fx, fy, cx, cy)

    camera_poses = []
    pcds = []
    voxel_length = 0.002
    image_num = 24

    print('Create point cloud from rgb and depth.')
    for i in range(image_num):
        print('Create {:d}-th point cloud.'.format(i))
        camera_pose = np.loadtxt(
            os.path.join(args.input,
                         'camera_pose/camera_pose{:03}.txt'.format(i)))

        color = o3d.io.read_image(
            os.path.join(args.input,
                         'color{:03}.png'.format(i)))
        depth = o3d.io.read_image(
            os.path.join(args.input,
                         'depth{:03}.png'.format(i)))

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, depth_trunc=4.0, convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd, intrinsic)
        pcd = pcd.voxel_down_sample(voxel_length)
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.1, max_nn=30))

        # pcd.remove_radius_outlier(nb_points=100, radius=0.002)
        pcd.remove_statistical_outlier(nb_neighbors=100,
                                       std_ratio=0.001)

        # o3d.visualization.draw_geometries([pcd])

        c = skrobot.coordinates.Coordinates(
            pos=camera_pose[:3, 3],
            rot=camera_pose[:3, :3])
        camera_poses.append(c)
        pcds.append(pcd)
        # mesh = o3d.geometry.TriangleMesh()
        # mesh.create_from_point_cloud_poisson(pcd)
        # o3d.visualization.draw_geometries([mesh])

    np.savetxt(
        os.path.join(args.input,
                     'camera_pose/camera_pose_icp000.txt'),
        camera_poses[0].T())

    print('ICP registration start.')
    target = pcds[0]
    for i in range(image_num - 1):
        print('ICP registration {:d}-th point cloud.'.format(i+1))
        trans_init = camera_poses[0].copy_worldcoords().inverse_transformation(
        ).transform(camera_poses[i + 1])

        source = pcds[i + 1]

        # draw_registration_result_original_color(source, target,
        #                                         trans_init.T())

        # result_icp = o3d.registration.registration_icp(
        #     source, target, 0.005, trans_init.T(),
        #     o3d.registration.TransformationEstimationPointToPlane())
        result_icp = o3d.registration.registration_icp(
            source, target, 0.01, trans_init.T(),
            o3d.registration.TransformationEstimationPointToPoint())
        # result_icp = o3d.registration.registration_colored_icp(
        #     source, target, 0.01, trans_init.T(),
        #     o3d.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
        #                                             relative_rmse=1e-6,
        #                                             max_iteration=10))
        # print(result_icp.transformation)

        icp_coords = skrobot.coordinates.Coordinates(
            pos=result_icp.transformation[:3, 3],
            rot=result_icp.transformation[:3, :3])
        camera_pose_icp = camera_poses[0].copy_worldcoords(
        ).transform(icp_coords)

        np.savetxt(
            os.path.join(
                args.input,
                'camera_pose/camera_pose_icp{:03}.txt'.format(i+1)),
            camera_pose_icp.T())

        # Save camera pose and intrinsic for texture-mapping
        with open(os.path.join(args.input,
                               'color{:03}.txt'.format(i+1)), 'w') as f:
            np.savetxt(f, np.concatenate(
                [camera_pose_icp.T()[:3, 3][None, :],
                 camera_pose_icp.T()[:3, :3]],
                axis=0))
            np.savetxt(f, [fx])
            np.savetxt(f, [fy])
            np.savetxt(f, [cx])
            np.savetxt(f, [cy])
            np.savetxt(f, [height])
            np.savetxt(f, [width])

        # draw_registration_result_original_color(source, target,
        #                                         result_icp.transformation)

        source.transform(result_icp.transformation)

        target = target + source
        target = target.voxel_down_sample(voxel_length)
        target.remove_statistical_outlier(nb_neighbors=100,
                                          std_ratio=0.001)
        # pcd.remove_radius_outlier(nb_points=100, radius=0.002)
        # o3d.visualization.draw_geometries([target])

    cl, ind = target.remove_radius_outlier(nb_points=100, radius=0.01)
    target = target.select_down_sample(ind)
    o3d.visualization.draw_geometries([target])
    o3d.io.write_point_cloud(os.path.join(
        args.input, 'icp_result.ply'), target)
