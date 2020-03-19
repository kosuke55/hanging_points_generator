#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import open3d as o3d
import numpy as np

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--input', '-i', type=str,
                        help='input dir',
                        default='savedir/')
    parser.add_argument('--output', '-o', type=str,
                        help='output file name',
                        default='save_dir/out.ply')
    parser.add_argument('--num', '-n', type=int,
                        help='How many images',
                        default=24)
    args = parser.parse_args()

    intrinsic_np = np.loadtxt(args.input + "camera_pose/intrinsic.txt")
    print(intrinsic_np)
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(
        1920,
        1080,
        intrinsic_np[0, 0],
        intrinsic_np[1, 1],
        intrinsic_np[0, 2],
        intrinsic_np[1, 2])

    volume = o3d.integration.ScalableTSDFVolume(
        voxel_length=0.002,
        sdf_trunc=0.005,
        color_type=o3d.integration.TSDFVolumeColorType.RGB8)

    image_num = args.num
    for i in range(image_num):
        print("Integrate {:d}-th image into the volume.".format(i))

        camera_pose = np.loadtxt(
            args.input +
            "camera_pose/camera_pose_icp{:03}.txt".format(i))
        # camera_pose = np.load("savedir/camera_pose{:03}.npy".format(i))
        color = o3d.io.read_image(
            args.input + "/color{:03}.png".format(i))
        depth = o3d.io.read_image(
            args.input + "/depth{:03}.png".format(i))

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, depth_trunc=4.0, convert_rgb_to_intensity=False)
        # pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        #     rgbd, intrinsic)
        # o3d.visualization.draw_geometries([pcd])

        volume.integrate(
            rgbd,
            intrinsic,
            np.linalg.inv(camera_pose))

    print("Extract a triangle mesh from the volume and visualize it.")
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh])
    o3d.io.write_triangle_mesh(args.output, mesh)
