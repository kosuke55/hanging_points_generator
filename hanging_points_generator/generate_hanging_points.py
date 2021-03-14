#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import glob
import os.path as osp
from pathlib import Path

from tqdm import tqdm

from hanging_points_generator import hp_generator as hpg
from hanging_points_generator.generator_utils import load_list
from hanging_points_generator.generator_utils \
    import load_multiple_contact_points

parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument(
    '--input-dir',
    '-i',
    type=str,
    help='input directory',
    default='../ycb_eval_data/hanging_eval')
    # default='/media/kosuke55/SANDISK/meshdata/hanging_object')
parser.add_argument(
    '--required-points-num', '-n', type=int,
    help='required points number',
    default=5)
parser.add_argument(
    '--try-num', '-tn', type=int,
    help='number of try', default=1000)
parser.add_argument(
    '--existed-points-num', '-en', type=int,
    help='threshold for the number of points already generated. '
    'Skip more than this number.'
    'If this number is -1, do not skip any file',
    default=-1)
parser.add_argument(
    '--gui', '-g', action='store_true',
    help='gui')
parser.add_argument(
    '--viz_obj', '-v', action='store_true',
    help='viz obj with contactpoints')
parser.add_argument(
    '--hook-type', '-ht', type=str,
    help='hook type "just_bar" or hook urdf',
    default='just_bar')
parser.add_argument(
    '--skip-list', '-s', type=str,
    help='skip file list',
    default='')

args = parser.parse_args()
input_dir = args.input_dir
files = sorted(glob.glob(osp.join(input_dir, '*/base.urdf')))
existed_points_num = args.existed_points_num
skip_file = args.skip_list

skip_list = []
if osp.isfile(skip_file):
    skip_list = load_list(skip_file)
print('skip list: ', skip_list)

for file in tqdm(files):
    dirname, filename = osp.split(file)
    category_name = Path(dirname).name
    print(category_name)
    if category_name in skip_list:
        print('Skipped %s because it is in skip_list' % file)
        continue
    print('-----------------------')
    num_cp = 0
    contact_points_path = osp.join(dirname, 'contact_points')
    if osp.isdir(contact_points_path):
        contact_points = load_multiple_contact_points(contact_points_path)
        if contact_points:
            num_cp = len(contact_points['contact_points'])
    print('Existed points of %s :%d' % (file, num_cp))
    if 0 <= existed_points_num <= num_cp:
        print('Skipped %s ' % file)
        continue

    hpg.generate(urdf_file=file,
                 required_points_num=args.required_points_num,
                 try_num=args.try_num,
                 enable_gui=args.gui,
                 viz_obj=args.viz_obj,
                 save_dir=dirname,
                 hook_type=args.hook_type,
                 render=False)
