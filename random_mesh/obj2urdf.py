#!/usr/bin/env python
# -*- coding: utf-8 -*-

import glob
import os

import trimesh
import tqdm

from hanging_points_generator import create_mesh


files = glob.glob('/media/kosuke/SANDISK/meshdata/ramdom_shape/*/*')

for file in tqdm.tqdm(files, total=len(files)):
    save_dir, filename = os.path.split(file)
    if filename != 'base.obj':
        continue

    try:
        mesh = trimesh.load(file)
        mesh_invert = mesh.copy()
        mesh_invert.invert()
        mesh += mesh_invert
        mesh.merge_vertices()

    except Exception:
        continue

    if mesh.vertices.shape[0] > 1 and mesh.faces.shape[0] > 1:
        create_mesh.create_urdf(mesh, save_dir)
