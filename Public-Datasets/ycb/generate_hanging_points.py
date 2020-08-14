#!/usr/bin/env python
# -*- coding: utf-8 -*-

import glob
import os
import os.path as osp

from hanging_points_generator import hp_generator as hpg

hanging_object_dict = {
    0: "019_pitcher_base",
    1: "022_windex_bottle",
    2: "025_mug",
    3: "033_spatula",
    4: "035_power_drill",
    5: "037_scissors",
    6: "042_adjustable_wrench",
    7: "048_hammer",
    8: "050_medium_clamp",
    9: "051_large_clamp",
    10: "052_extra_large_clamp"
}

urdf_dir = '/media/kosuke55/SANDISK/meshdata/ycb_hanging_object/urdf_cp'
files = glob.glob(osp.join(urdf_dir, '*/*'))

for file in files:
    dirname, filename = os.path.split(file)
    filename_without_ext, ext = os.path.splitext(filename)
    category_name = dirname.split("/")[-1]

    # if category_name != hanging_object_dict[0]:
    #     continue

    if filename == 'base.urdf':
        print('-----------------------')
        print(file)
        hpg.generate(urdf_file=file,
                     required_points_num=5,
                     enable_gui=0,
                     viz_obj=0,
                     save_dir=dirname,
                     hook_type='just_bar',
                     render=False)
