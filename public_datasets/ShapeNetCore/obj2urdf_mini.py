#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import glob
import os
import random
import shutil
from pathlib import Path

import numpy as np
import open3d as o3d
from shapenet_utils import hanging_synset
from shapenet_utils import pouring_synset
from shapenet_utils import synset_to_label

from hanging_points_generator.create_mesh import create_urdf
from hanging_points_generator.generator_utils import load_list
from hanging_points_generator.create_mesh import open3d_to_trimesh


parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)

parser.add_argument(
    '--input-dir',
    '-i',
    type=str,
    help='input directory',
    default='/media/kosuke55/SANDISK/meshdata/ShapeNetCore.v2')
parser.add_argument(
    '--save-dir',
    '-s',
    type=str,
    help='save directory',
    # default='/media/kosuke55/SANDISK/meshdata/shapenet_mini_10')
    # default='/media/kosuke55/SANDISK/meshdata/shapenet_mini_hanging_50'
    default='/media/kosuke55/SANDISK/meshdata/shapenet_mini_pouring_50')
parser.add_argument(
    '--num-samples',
    '-n',
    type=int,
    help='number of sampling',
    default=50)
parser.add_argument(
    '--existed-files-list',
    '-efl',
    type=str,
    help='If some files already exist, specify the list name of the files. '
    'Add new objects until the number of total objects becomes num-samples.',
    default=None)
    # '/home/kosuke55/snippets/python/shapnet_mini_10_copy/shapenet_mini_10_hanging_list.txt'  # noqa
parser.add_argument(
    '--task-type', '-t', type=str,
    default=None,
    help='if None, set it automaticaly from outfile')

args = parser.parse_args()

input_dir = args.input_dir
base_save_dir = args.save_dir
num_samples = args.num_samples
os.makedirs(base_save_dir, exist_ok=True)

if args.task_type is not None:
    task_type = args.task_type
else:
    if 'hanging' in base_save_dir:
        task_type = 'hanging'
    elif 'pouring' in base_save_dir:
        task_type = 'pouring'
print('task_type: ', task_type)

if task_type == 'hanging':
    object_list = hanging_synset()
elif task_type == 'pouring':
    object_list = pouring_synset()

print('object_list: ',
      [synset_to_label[o] for o in object_list])

target_length = 0.1

files = []

exist_files = False
if args.existed_files_list is not None:
    exist_files = load_list(args.existed_files_list)


for obj in object_list:
    one_category_paths = list(
        sorted(
            Path(input_dir).glob(
                '{}/*/models/model_normalized.obj'.format(obj))))
    sampled_one_category_files = []
    while len(sampled_one_category_files) < num_samples:
        idx = random.randint(0, len(one_category_paths) - 1)
        if exist_files:
            if True in [f.split('_')[1] in str(one_category_paths[idx])
                        for f in exist_files]:
                continue
        if str(one_category_paths[idx]) not in sampled_one_category_files:
            sampled_one_category_files.append(str(one_category_paths[idx]))

    print(
        synset_to_label[obj],
        len(one_category_paths),
        len(sampled_one_category_files))
    files.extend(sampled_one_category_files)


for file in files:
    dirname, filename = os.path.split(file)
    filename_without_ext, ext = os.path.splitext(filename)
    id_ = dirname.split('/')[-2]
    # id_ = dirname.split('/')[-1]
    synset = dirname.split('/')[-3]
    category_name = synset_to_label[synset]
    save_dir = os.path.join(
        base_save_dir, synset + '_' + id_ + '_' + category_name)

    try:
        mesh = o3d.io.read_triangle_mesh(file)
        mesh = mesh.simplify_vertex_clustering(
            voxel_size=0.01,
            contraction=o3d.geometry.SimplificationContraction.Average)
        mesh = open3d_to_trimesh(mesh)
        mesh_invert = mesh.copy()
        mesh_invert.invert()
        mesh += mesh_invert
        mesh.merge_vertices()
        size = np.array(np.max(mesh.vertices, axis=0)
                        - np.min(mesh.vertices, axis=0))
        length = np.max(size)
        mesh.vertices = mesh.vertices * target_length / length

    except Exception as e:
        print('skip {} {} {}'.format(category_name, synset, id_))
        continue

    if mesh.vertices.shape[0] > 1 and mesh.faces.shape[0] > 1:
        try:
            print(dirname, save_dir)
            shutil.copytree(dirname, save_dir)
        except FileExistsError as e:
            print(e)
        create_urdf(mesh, save_dir, init_texture=True)
    else:
        print('skip {} {} {}'.format(category_name, synset, id_))
