#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import glob
import os.path as osp
import random
from pathlib import Path

from tqdm import tqdm

from hanging_points_generator.generator_utils import add_list
from hanging_points_generator.generator_utils import load_list
from hanging_points_generator.generator_utils \
    import load_multiple_contact_points
from hanging_points_generator import pp_generator as ppg

parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument(
    '--input-dir',
    '-i',
    type=str,
    help='input directory',
    default='/media/kosuke55/SANDISK/meshdata/shapenet_mini_10')
parser.add_argument('--required-points-num', '-n', type=int,
                    help='required points number. do not use this option now',
                    default=1)
parser.add_argument(
    '--existed-points-num',
    '-en',
    type=int,
    help='threshold for the number of points already generated.'
    'Skip more than this number.'
    'If this number is -1, do not skip any file',
    default=-1)
parser.add_argument('--gui', '-g', action='store_true',
                    help='gui')
parser.add_argument('--viz_obj', '-v', action='store_true',
                    help='viz obj with contactpoints')
parser.add_argument('--skip-list', '-s', type=str,
                    help='skip file list',
                    default='')
parser.add_argument('--unable-shuffle-files', '-usf', action='store_true',
                    help='unabale shuffle files')
parser.add_argument('--skip', action='store_true',
                    help='use skip file')

args = parser.parse_args()
input_dir = args.input_dir
files = glob.glob(osp.join(input_dir, '*/base.urdf'))
if not args.unable_shuffle_files:
    random.shuffle(files)

existed_points_num = args.existed_points_num
skip_list = []
skip_file = osp.join(
    input_dir, 'finish_list.txt') if args.skip_list == '' else args.skip_list

for file in tqdm(files):
    # if 'bowl' not in file:
    #     continue
    if osp.isfile(skip_file):
        skip_list = load_list(skip_file)
        # print('skip list: ', skip_list)
        print('skip list length : ', len(skip_list))

    dirname, filename = osp.split(file)
    category_name = Path(dirname).name

    if args.skip:
        if category_name in skip_list:
            print('Skipped %s because it is in skip_list' % file)
            continue

    print('-----------------------')
    num_cp = 0
    contact_points_path = osp.join(dirname, 'pouring_points')

    if osp.isdir(contact_points_path):
        contact_points = load_multiple_contact_points(
            contact_points_path, json_name='pouring_points.json')
        if contact_points:
            num_cp = len(contact_points['contact_points'])

    print('Existed points of %s :%d' % (file, num_cp))

    if 0 <= existed_points_num <= num_cp:
        print('Skipped %s ' % file)
        continue

    ppg.generate(urdf_file=file,
                 required_points_num=args.required_points_num,
                 enable_gui=args.gui,
                 viz_obj=args.viz_obj,
                 save_dir=dirname,
                 pattern_spheres=True)
