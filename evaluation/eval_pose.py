#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
from pathlib import Path

import numpy as np

from hanging_points_generator.generator_utils import load_json
from hanging_points_generator.generator_utils import save_json


def two_vectors_angle(v1, v2):
    cos = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    return np.arccos(cos)


parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)

parser.add_argument(
    '--input-dir',
    '-i',
    type=str,
    help='input directory',
    default='/media/kosuke55/SANDISK/meshdata/ycb_hanging_object/urdf_eval/')
parser.add_argument(
    '--ground-truth',
    '-gt',
    type=str,
    help='',
    default='/media/kosuke55/SANDISK/meshdata/ycb_hanging_object/urdf2/annotation_obj')
args = parser.parse_args()

input_dir = args.input_dir
gt_dir = args.ground_truth

input_paths = Path(input_dir).glob('*/filtered_contact_points.json')
for input_path in input_paths:
    print('\n-------')
    category = input_path.parent.name
    gt_path = (Path(gt_dir) / category).with_suffix('.json')
    print(gt_path)
    gt_data = load_json(str(gt_path))
    gt_pos = [p[0] for p in gt_data['contact_points']]
    gt_vec = [np.array(p)[1:, 0] for p in gt_data['contact_points']]
    gt_labels = [label for label in gt_data['labels']]

    thresh_distance = 0.03

    data = load_json(str(input_path))
    labels = data['labels']
    print('total len: %d' % len(labels))
    diff_dict = {}
    diff_dict['-1'] = {}
    diff_dict['-1']['pos_diff'] = []
    diff_dict['-1']['distance'] = []
    diff_dict['-1']['angle'] = []

    for pose, label in zip(data['contact_points'], data['labels']):
        pose = np.array(pose)
        pos_diff = pose[0] - gt_pos
        distances = np.linalg.norm(pos_diff, axis=1)
        min_idx = np.argmin(distances)
        min_distance = np.min(distances)

        if str(gt_labels[min_idx]) not in diff_dict:
            diff_dict[str(gt_labels[min_idx])] = {}
            diff_dict[str(gt_labels[min_idx])]['pos_diff'] = []
            diff_dict[str(gt_labels[min_idx])]['distance'] = []
            diff_dict[str(gt_labels[min_idx])]['angle'] = []

        pos_diff = pos_diff[min_idx].tolist()
        angle = min(two_vectors_angle(pose[1:, 0], gt_vec[min_idx]),
                    two_vectors_angle(pose[1:, 0], -gt_vec[min_idx]))

        if min_distance > thresh_distance:
            diff_dict['-1']['pos_diff'].append(pos_diff)
            diff_dict['-1']['distance'].append(min_distance)
            diff_dict['-1']['angle'].append(angle)
        else:
            diff_dict[str(gt_labels[min_idx])]['pos_diff'].append(pos_diff)
            diff_dict[str(gt_labels[min_idx])]['distance'].append(min_distance)
            diff_dict[str(gt_labels[min_idx])]['angle'].append(angle)

    for key in diff_dict:
        print('----label %s ----' % key)
        print('len: %d' % len(diff_dict[key]['angle']))
        if key == '-1':
            continue
        pos_diff = np.array(diff_dict[key]['pos_diff'])
        if pos_diff.size == 0:
            continue
        diff_dict[key]['pos_diff_mean'] = np.mean(pos_diff, axis=0).tolist()
        diff_dict[key]['pos_diff_max'] = np.max(pos_diff, axis=0).tolist()
        diff_dict[key]['pos_diff_min'] = np.min(pos_diff, axis=0).tolist()

        print('pos_diff_max %f %f %f' % tuple(diff_dict[key]['pos_diff_max']))
        print('pos_diff_mean %f %f %f' % tuple(diff_dict[key]['pos_diff_mean']))
        print('pos_diff_min %f %f %f' % tuple(diff_dict[key]['pos_diff_min']))

        angle = np.array(diff_dict[key]['angle'])
        diff_dict[key]['angle_mean'] = np.mean(angle).tolist()
        diff_dict[key]['angle_max'] = np.max(angle).tolist()
        diff_dict[key]['angle_min'] = np.min(angle).tolist()

        print('angle_max %f' % diff_dict[key]['angle_max'])
        print('angle_mean %f' % diff_dict[key]['angle_mean'])
        print('angle_min %f' % diff_dict[key]['angle_min'])

    save_json(str(input_path.parent / 'eval.json'), diff_dict)