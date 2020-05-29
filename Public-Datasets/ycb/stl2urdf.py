#!/usr/bin/env python
# -*- coding: utf-8 -*-

import glob
import open3d as o3d
import os
import trimesh

from hanging_points_generator import create_mesh

files = glob.glob("ycb/*/*/*")
os.makedirs("ycb_hanging_object/urdf", exist_ok=True)

# http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/
hanging_object_list = [
    "019_pitcher_base",
    "022_windex_bottle",
    "025_mug",
    "033_spatula",
    "035_power_drill",
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
       or filename != "nontextured.stl":
        continue

    try:
        # mesh = trimesh.load(file)
        mesh = o3d.io.read_triangle_mesh(file)
        mesh = mesh.simplify_vertex_clustering(
            voxel_size=0.01,
            contraction=o3d.geometry.SimplificationContraction.Average)
        o3d.io.write_triangle_mesh("/tmp/mesh_tmp.stl", mesh)
        mesh = trimesh.load("/tmp/mesh_tmp.stl")
        mesh_invert = mesh.copy()
        mesh_invert.invert()
        mesh += mesh_invert
        mesh.merge_vertices()

    except Exception:
        continue

    if mesh.vertices.shape[0] > 1 and mesh.faces.shape[0] > 1:
        print(file, mesh)
        os.makedirs(os.path.join(
            "ycb_hanging_object/urdf",
            category_name, filename_without_ext), exist_ok=True)
        create_mesh.create_urdf(mesh, os.path.join(
            "ycb_hanging_object/urdf", category_name, filename_without_ext))
