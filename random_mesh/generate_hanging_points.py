#!/usr/bin/env python
# -*- coding: utf-8 -*-

import glob
import os
import os.path as osp

from hanging_points_generator import hp_generator as hpg

urdf_dir = '/media/kosuke55/SANDISK/meshdata/random_shape_ycb'
files = sorted(
    glob.glob(osp.join(urdf_dir, '*/*')))

for file in files:
    dirname, filename = os.path.split(file)
    filename_without_ext, ext = os.path.splitext(filename)

    if filename == 'base.urdf':
        hpg.generate(urdf_file=file,
                     required_points_num=30,
                     enable_gui=0,
                     viz_obj=0,
                     save_dir=dirname,
                     hook_type='just_bar',
                     render=False)
