#!/usr/bin/env python
# -*- coding: utf-8 -*-

import glob
import os

from hanging_points_generator import hp_generator as hpg

files = glob.glob("/media/kosuke55/SANDISK/meshdata/random_shape_ycb/*/*")

for file in files:
    dirname, filename = os.path.split(file)
    filename_without_ext, ext = os.path.splitext(filename)

    if filename == 'base.urdf':
        print('-----------------------')
        print(file)
        hpg.generate(urdf_file=file,
                     required_points_num=30,
                     enable_gui="true",
                     viz_obj="false",
                     save_dir=dirname,
                     hook_type='just_bar',
                     render=False)
