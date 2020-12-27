#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import glob
import os
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import open3d as o3d
import trimesh

from hanging_points_generator import create_mesh

parser = argparse.ArgumentParser()
parser.add_argument(
    '--input-dir', '-i', type=str,
    default='/media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16',
    help='input ycb mesh directory')
parser.add_argument(
    '--task-type', '-t', type=str,
    default=None,
    help='h -> hanging, p -> pouring. Otherwise, '
    'it will be guessed automatically from the input.')
parser.add_argument(
    '--unable-init-texture', '-uit',
    action='store_true',
    help='unable initializing texture ')
args = parser.parse_args()

input_dir = args.input_dir

if args.task_type == 'p':
    task_type = 'pouring'
elif args.task_type == 'h':
    task_type = 'hanging'
else:
    if 'hanging' in input_dir:
        task_type = 'hanging'
    elif 'pouring' in input_dir:
        task_type = 'pouring'
    else:
        raise ValueError('specify task_type hanging or pouring')

init_texture = not args.unable_init_texture
print('task type: {}'.format(task_type))

if task_type == 'hanging':
    # http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/
    object_list = [
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

elif task_type == 'pouring':
    object_list = [
        "019_pitcher_base",
        "024_bowl",
        "025_mug",
        "027_skillet",
        "029_plate",
    ]

if init_texture:
    save_dir = Path(input_dir) / 'textured_urdf'.format(task_type)  # noqa
    save_dir = '/media/kosuke55/SANDISK/meshdata/ycb_hanging_object/urdf_eval_textured'
else:
    save_dir = Path(input_dir) / 'urdf'.format(task_type)  # noqa

files = list(Path(input_dir).glob('*/*/nontextured.stl'))
os.makedirs(save_dir, exist_ok=True)

for file_path in files:
    file = str(file_path)
    dirname, filename = os.path.split(file)
    category_name = dirname.split('/')[-2]

    if category_name not in object_list:
        continue

    try:
        print(file)
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
        import traceback
        traceback.print_exc()
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
        
        
