#!/usr/bin/env python
# -*- coding: utf-8 -*-

from hanging_points_generator import create_mesh

import glob
import os
import trimesh

files = glob.glob("off/*/*")
os.makedirs("ply", exist_ok=True)
os.makedirs("urdf", exist_ok=True)

scale = 0.1

for file in files:
    dirname, filename = os.path.split(file)
    filename_without_ext, ext = os.path.splitext(filename)
    category_name = dirname.split("/")[1]
    os.makedirs(os.path.join("ply", category_name), exist_ok=True)
    hanging_object_list = ["cap", "cup", "headphone", "helmet",
                           "key", "scissors", "slipper"]
    if 'off' in ext.lower():
        if category_name not in hanging_object_list:
            continue
        try:
            mesh = trimesh.load(file)
            mesh_invert = mesh.copy()
            mesh_invert.invert()
            mesh += mesh_invert
            mesh.merge_vertices()
            mesh.vertices *= scale
        except Exception:
            continue

        if mesh.vertices.shape[0] > 1 and mesh.faces.shape[0] > 1:
            print(file, mesh)
            mesh.export(os.path.join(
                "ply", category_name, filename_without_ext + ".ply"))
            os.makedirs(os.path.join(
                "urdf", category_name, filename_without_ext), exist_ok=True)
            create_mesh.create_urdf(mesh, os.path.join(
                "urdf", category_name, filename_without_ext))
