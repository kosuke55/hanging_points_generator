#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import glob
import logging
import os
import os.path as osp
import shutil
from datetime import datetime

import coloredlogs
import numpy as np
import open3d as o3d

from hanging_points_generator.create_mesh import create_urdf
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
    default='/media/kosuke55/SANDISK/meshdata/hanging_object')
args = parser.parse_args()

logfile = '{}_obj2urdf.log'.format(datetime.now().strftime('%Y%m%d_%H%M'))
logging.basicConfig(filename=logfile, level=logging.DEBUG)
logger = logging.getLogger(__name__)
coloredlogs.install(level='DEBUG')

input_dir = args.input_dir
synset_to_label = {'03797390': 'mug'}
label_to_synset = {v: k for k, v in synset_to_label.items()}

ignore_list = [
    # mug
    'b9be7cfe653740eb7633a2dd89cec754',
    '659192a6ba300f1f4293529704725d98',
    'bf2b5e941b43d030138af902bc222a59',
    'b98fa11a567f644344b25d683fe71de',
    'b9004dcda66abf95b99d2a3bbaea842a',
    '127944b6dabee1c9e20e92c5b8147e4a',
    '1038e4eac0e18dcce02ae6d2a21d494a',
    'e71102b6da1d63f3a363b55cbd344baa',
    '27119d9b2167080ec190cb14324769d',
    '5ef0c4f8c0884a24762241154bf230ce',
]

base_save_dir = args.save_dir
os.makedirs(base_save_dir, exist_ok=True)

hanging_object_list = [
    label_to_synset['mug']
]

target_length = 0.1

files = []
for hanging_object in hanging_object_list:
    files.extend(glob.glob(osp.join(
        input_dir, '{}/*/models/model_normalized.obj'.format(hanging_object))))

for file in files:
    dirname, filename = os.path.split(file)
    filename_without_ext, ext = os.path.splitext(filename)
    id_ = dirname.split('/')[-2]
    # id_ = dirname.split('/')[-1]
    if id_ in ignore_list:
        logger.info('ignore {}'.format(file))
        continue
    synset = dirname.split('/')[-3]
    category_name = synset_to_label[synset]
    save_dir = os.path.join(
        base_save_dir, synset + '_' + id_ + '_' + category_name)
    print(save_dir)
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
        logger.warning('skip {} {} {}'.format(category_name, synset, id_))
        logger.warning(e)
        continue

    if mesh.vertices.shape[0] > 1 and mesh.faces.shape[0] > 1:
        try:
            shutil.copytree(dirname, save_dir)
        except FileExistsError as e:
            logger.warning(e)

        create_urdf(mesh, save_dir, init_texture=True)
