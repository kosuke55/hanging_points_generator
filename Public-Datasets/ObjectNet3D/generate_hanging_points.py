#!/usr/bin/env python
# -*- coding: utf-8 -*-

from hanging_points_generator import hp_generator as hpg

import glob
import os

files = glob.glob("urdf/*/*/*")

for file in files:
    dirname, filename = os.path.split(file)
    filename_without_ext, ext = os.path.splitext(filename)
    category_name = dirname.split("/")[1]
    if 'urdf' in ext.lower():
        print(file)
        hpg.generate(urdf_file=file,
                     required_points_num=100,
                     enable_gui="false",
                     viz_obj="false",
                     save_dir=dirname,
                     render=False)
