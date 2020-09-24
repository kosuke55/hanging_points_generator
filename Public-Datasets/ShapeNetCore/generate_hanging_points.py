#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import glob
import os.path as osp

from tqdm import tqdm

from hanging_points_generator import hp_generator as hpg
from hanging_points_generator.generator_utils import load_multiple_contact_points

parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument(
    '--input-dir',
    '-i',
    type=str,
    help='input directory',
    default='/media/kosuke55/SANDISK/meshdata/hanging_object')
parser.add_argument('--required-points-num', '-n', type=int,
                    help='required points number',
                    default=1)
parser.add_argument('--existed-points-num', '-en', type=int,
                    help='required points number',
                    default=100)
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
existed_points_num = args.existed_points_num

for file in tqdm(files):
    dirname, filename = osp.split(file)
    print('-----------------------')
    num_cp = 0
    contact_points_path = osp.join(dirname, 'contact_points')
    if osp.isdir(contact_points_path):
        contact_points = load_multiple_contact_points(contact_points_path)
        if contact_points is not None:
            num_cp = len(contact_points['contact_points'])
    print('Existed points of %s :%d' % (file, num_cp))
    if existed_points_num <= num_cp:
        print('Skipped')
        continue

    hpg.generate(urdf_file=file,
                 required_points_num=args.required_points_num,
                 enable_gui=args.gui,
                 viz_obj=args.viz_obj,
                 save_dir=dirname,
                 hook_type=args.hook_type,
                 render=False)
