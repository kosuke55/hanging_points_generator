#!/usr/bin/env python
# -*- coding: utf-8 -*-

from hanging_points_generator import hp_generator as hpg

import glob
import os
import os.path as osp

urdf_dir = '/media/kosuke55/SANDISK/meshdata/Hanging-ObjectNet3D-DoubleFaces/CAD/urdf'
files = glob.glob(osp.join(urdf_dir, "*/*/*"))

hanging_object_dict = {
    0: 'cap',
    1: 'cup',
    2: 'headphone',
    3: 'helmet',
    4: 'key',
    5: 'scissors',
    6: 'slipper'
}

for file in files:
    dirname, filename = os.path.split(file)
    filename_without_ext, ext = os.path.splitext(filename)
    category_name = dirname.split("/")[-2]

    # if category_name != hanging_object_dict[5]:
    #     continue

    if 'urdf' in ext.lower():
        print(file)
        hpg.generate(urdf_file=file,
                     required_points_num=5,
                     enable_gui=0,
                     viz_obj=0,
                     save_dir=dirname,
                     render=False)
