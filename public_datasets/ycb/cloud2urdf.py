#!/usr/bin/env python
# -*- coding: utf-8 -*-

import glob
import os

import open3d as o3d
import trimesh

from hanging_points_generator.create_mesh \
    import create_mesh_voxelize_marcing_cubes, create_urdf

# files = glob.glob("ycb/*/*/*")
files = glob.glob("ycb_processed/*/*/*")
out_dir = "ycb_hanging_object_cloud/urdf"
# os.makedirs(out_dir, exist_ok=True)

# http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/
hanging_object_list = [
    "019_pitcher_base",
    "022_windex_bottle",
    "025_mug",
    "033_spatula",
    "035_power_drill",
    "037_scissors",
    "042_adjustable_wrench",
    "048_hammer",
    "050_medium_clamp",
    "051_large_clamp",
    "052_extra_large_clamp"
]

for file in files:
    dirname, filename = os.path.split(file)

    filename_without_ext, ext = os.path.splitext(filename)
    category_name = dirname.split('/')[-2]
    if category_name not in hanging_object_list \
       or filename != "merged_cloud.ply":
        continue

    print(filename)
    # try:
    pc = o3d.io.read_point_cloud(file)
    # mesh = create_mesh_voxelize_marcing_cubes(pc, voxel_size=0.006)
    mesh = create_mesh_voxelize_marcing_cubes(pc, voxel_size=0.005)
    # mesh = trimesh.load(file)
    mesh_invert = mesh.copy()
    mesh_invert.invert()
    mesh += mesh_invert
    mesh.merge_vertices()

    # except Exception:
    #     continue

    if mesh.vertices.shape[0] > 1 and mesh.faces.shape[0] > 1:
        print(file, mesh)
        # os.makedirs(os.path.join(
        #     "ycb_hanging_object/urdf",
        #     category_name, filename_without_ext), exist_ok=True)
        create_urdf(mesh, os.path.join(
            out_dir, category_name, filename_without_ext))
