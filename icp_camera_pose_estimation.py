#!/usr/bin/env python
# -*- coding: utf-8 -*-


import copy
import numpy as np
import open3d as o3d
import skrobot


def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target])


if __name__ == "__main__":
    intrinsic_np = np.load("savedir/intrinsic.npy")
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(
        1920,
        1080,
        intrinsic_np[0, 0],
        intrinsic_np[1, 1],
        intrinsic_np[0, 2],
        intrinsic_np[1, 2])

    volume = o3d.integration.ScalableTSDFVolume(
        voxel_length=0.005,
        sdf_trunc=0.01,
        color_type=o3d.integration.TSDFVolumeColorType.RGB8)

    camera_poses = []
    pcds = []
    voxel_length = 0.002

    for i in range(8):
        print("Integrate {:d}-th image into the volume.".format(i))
        camera_pose = np.load("savedir/camera_pose{}.npy".format(i))

        color = o3d.io.read_image(
            "savedir/color{}.png".format(i))
        depth = o3d.io.read_image(
            "savedir/depth{}.png".format(i))

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, depth_trunc=4.0, convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd, intrinsic)
        pcd = pcd.voxel_down_sample(voxel_length)
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.1, max_nn=30))

        # pcd.remove_radius_outlier(nb_points=100, radius=0.002)
        # pcd.remove_statistical_outlier(nb_neighbors=100,
        #                                std_ratio=0.001)

        # o3d.visualization.draw_geometries([pcd])

        c = skrobot.coordinates.Coordinates(
            pos=camera_pose[:3, 3],
            rot=camera_pose[:3, :3])
        camera_poses.append(c)
        pcds.append(pcd)
        # mesh = o3d.geometry.TriangleMesh()
        # mesh.create_from_point_cloud_poisson(pcd)
        # o3d.visualization.draw_geometries([mesh])

        # volume.integrate(
        #     rgbd,
        #     intrinsic,
        #     np.linalg.inv(camera_pose))

    np.save("savedir/camera_pose_icp0.npy", camera_poses[0].T())
    target = pcds[0]
    for i in range(7):
        trans_init = camera_poses[0].copy_worldcoords().inverse_transformation(
        ).transform(camera_poses[i+1])

        source = pcds[i+1]

        # draw_registration_result_original_color(source, target,
        #                                         trans_init.T())w

        result_icp = o3d.registration.registration_icp(
            source, target, 0.02, trans_init.T(),
            o3d.registration.TransformationEstimationPointToPlane())
        print(result_icp.transformation)

        icp_coords = skrobot.coordinates.Coordinates(
            pos=result_icp.transformation[:3, 3],
            rot=result_icp.transformation[:3, :3])
        camera_pose_icp = camera_poses[0].copy_worldcoords(
        ).transform(icp_coords)

        np.save("savedir/camera_pose_icp{}.npy".format(i+1),
                camera_pose_icp.T())

        # draw_registration_result_original_color(source, target,
        #                                         result_icp.transformation)

        source.transform(result_icp.transformation)

        target = target + source
        target = target.voxel_down_sample(voxel_length)

    cl, ind = target.remove_radius_outlier(nb_points=100, radius=0.01)
    target = target.select_down_sample(ind)
    o3d.visualization.draw_geometries([target])
    o3d.io.write_point_cloud("icp_result.ply", target)

    # distances = target.compute_nearest_neighbor_distance()
    # avg_dist = np.mean(distances)
    # radius = 1.5 * avg_dist
    # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    #     target,
    #     o3d.utility.DoubleVector([radius, radius * 2]))
    # o3d.visualization.draw_geometries([mesh])

    # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
    #     pcd=target, depth=10, width=0, scale=1.1, linear_fit=True)
    # o3d.visualization.draw_geometries([mesh[0]])
