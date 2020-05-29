#!/usr/bin/env python
# -*- coding: utf-8 -*-

import glob
import os

from hanging_points_generator import hanging_points_generator as hpg

hanging_object_dict = {
    0: "019_pitcher_base",
    1: "022_windex_bottle",
    2: "025_mug",
    3: "033_spatula",
    4: "035_power_drill",
    5: "042_adjustable_wrench",
    6: "048_hammer",
    7: "050_medium_clamp",
    8: "051_large_clamp",
    9: "052_extra_large_clamp"
}

files = glob.glob("urdf/*/*/*")

for file in files:
    dirname, filename = os.path.split(file)
    filename_without_ext, ext = os.path.splitext(filename)
    category_name = dirname.split("/")[1]
    idx = dirname.split("/")[2]

    # if category_name != hanging_object_dict[0]:
    #     continue

    if 'urdf' in ext.lower():
        print('-----------------------')
        print(file)
        hpg.generate(urdf_file=file,
                     required_points_num=30,
                     enable_gui="true",
                     viz_obj="false",
                     save_dir=dirname,
                     hook_type='just_bar')
