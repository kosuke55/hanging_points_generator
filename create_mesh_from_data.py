#!/usr/bin/env python
# -*- coding: utf-8 -*-

import open3d as o3d
import numpy as np

if __name__ == "__main__":
    intrinsic_np = np.load("create_mesh_sample_data/intrinsic.npy")
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(
        960,
        540,
        intrinsic_np[0, 0],
        intrinsic_np[1, 1],
        intrinsic_np[0, 2],
        intrinsic_np[1, 2])

    volume = o3d.integration.ScalableTSDFVolume(
        voxel_length=0.001,
        sdf_trunc=0.01,
        color_type=o3d.integration.TSDFVolumeColorType.RGB8)

    for i in range(8):
        print("Integrate {:d}-th image into the volume.".format(i))
        camera_pose = np.load("create_mesh_sample_data/camera_pose{}.npy".format(i))
        color = o3d.io.read_image(
            "create_mesh_sample_data/color{}.png".format(i))
        depth = o3d.io.read_image(
            "create_mesh_sample_data/depth{}.png".format(i))

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, depth_trunc=4.0, convert_rgb_to_intensity=False)
        volume.integrate(
            rgbd,
            intrinsic,
            np.linalg.inv(camera_pose))

    print("Extract a triangle mesh from the volume and visualize it.")
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh])
