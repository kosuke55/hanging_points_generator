#!/usr/bin/env python
# -*- coding: utf-8 -*-

import glob
import open3d as o3d
import os
import os.path as osp
import subprocess
import trimesh
import xml.etree.ElementTree as ET

from hanging_points_generator import create_mesh

input_dir = '/media/kosuke55/SANDISK/meshdata/ycb_hanging_object_16'
# example
# /media/kosuke55/SANDISK/meshdata/ycb_hanging_object_16/019_pitcher_base/google_16k/nontextured.stl
files = glob.glob(osp.join(input_dir, '*/*/*'))

init_texture = True
if init_texture:
    save_dir = "/media/kosuke55/SANDISK/meshdata/ycb_hanging_object/textured_urdf"
else:
    save_dir = "/media/kosuke55/SANDISK/meshdata/ycb_hanging_object/urdf"

os.makedirs(save_dir, exist_ok=True)

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
            save_dir,
            category_name,), exist_ok=True)

        subprocess.call(
            ["cp " + dirname + "/* " + os.path.join(save_dir, category_name)],
            shell=True)

        create_mesh.create_urdf(mesh, os.path.join(
            save_dir, category_name), init_texture=init_texture)

        tree = ET.parse(os.path.join(save_dir, category_name, 'base.urdf'))
        root = tree.getroot()
        root[0].find('visual').find('geometry').find(
            'mesh').attrib['filename'] = 'textured.obj'
        tree.write(os.path.join(save_dir, category_name, 'textured.urdf'),
                   encoding='utf-8', xml_declaration=True)
