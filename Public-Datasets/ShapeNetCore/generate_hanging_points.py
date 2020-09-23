#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import glob
import os.path as osp

from hanging_points_generator import hp_generator as hpg

parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument(
    '--input-dir',
    '-i',
    type=str,
    help='input directory',
    default='/media/kosuke55/SANDISK/meshdata/hanging_object')
parser.add_argument('--required_points_num', '-n', type=int,
                    help='required points number',
                    default=1)
parser.add_argument('--gui', '-g', type=int,
                    help='gui', default=0)
parser.add_argument('--viz_obj', '-v', type=int,
                    help='viz obj with contactpoints',
                    default=0)
parser.add_argument('--hook-type', '-ht', type=str,
                    help='hook type "just_bar" or hook urdf',
                    default='just_bar')

args = parser.parse_args()
input_dir = args.input_dir
files = glob.glob(osp.join(input_dir, '*/base.urdf'))

for file in files:
    dirname, filename = osp.split(file)
    print('-----------------------')
    print(file)
    hpg.generate(urdf_file=file,
                 required_points_num=args.required_points_num,
                 enable_gui=args.gui,
                 viz_obj=args.viz_obj,
                 save_dir=dirname,
                 hook_type=args.hook_type,
                 render=False)
