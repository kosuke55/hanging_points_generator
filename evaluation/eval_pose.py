#!/usr/bin/env python
# -*- coding: utf-8 -*-
# flake8: noqa

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
    # default='/media/kosuke55/SANDISK/meshdata/ycb_hanging_object/urdf_eval/')
    default='/media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf')
parser.add_argument(
    '--ground-truth',
    '-gt',
    type=str,
    help='',
    # default='/media/kosuke55/SANDISK/meshdata/ycb_hanging_object/urdf2/annotation_obj')
    default='/media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf/annotation_obj')
parser.add_argument(
    '--print-mode', '-p',
    type=int,
    help='0: dx dy dz angle '
    '1: dx dy dz dl angle '
    '2: dl angle ',
    default=0)

args = parser.parse_args()

input_dir = args.input_dir
gt_dir = args.ground_truth
print_mode = args.print_mode

thresh_distance = 0.03

filtering_result = load_json(str(Path(input_dir) / 'filtering_result.json'))

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
        pos_diff = np.abs(pose[0] - gt_pos)
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

        distance = np.array(diff_dict[key]['distance'])
        diff_dict[key]['distance_mean'] = np.mean(distance).tolist()
        diff_dict[key]['distance_max'] = np.max(distance).tolist()
        diff_dict[key]['distance_min'] = np.min(distance).tolist()
        print('distance_max %f' % diff_dict[key]['distance_max'])
        print('distance_mean %f' % diff_dict[key]['distance_mean'])
        print('distance_min %f' % diff_dict[key]['distance_min'])

        angle = np.array(diff_dict[key]['angle'])
        diff_dict[key]['angle_mean'] = np.mean(angle).tolist()
        diff_dict[key]['angle_max'] = np.max(angle).tolist()
        diff_dict[key]['angle_min'] = np.min(angle).tolist()
        print('angle_max %f' % diff_dict[key]['angle_max'])
        print('angle_mean %f' % diff_dict[key]['angle_mean'])
        print('angle_min %f' % diff_dict[key]['angle_min'])

    eval_json_path = input_path.parent / 'eval.json'
    save_json(str(eval_json_path), diff_dict)

print('\n*** For tex ***')
table_contents = ''
number_table_contents = ''
error_table_contents = ''

paths = list(sorted(Path(input_dir).glob('*/eval.json')))
for path in paths:
    diff_dict = load_json(path)
    noise_num = 0
    for key in diff_dict:
        if key == '-1':
            continue
        category = path.parent.name[4:].replace('_', ' ')
        noise_num = len(diff_dict['-1']['pos_diff'])

        filter_current_category = filtering_result[path.parent.name]
        before_num = filter_current_category['pre_points_num']
        after_num = filter_current_category['post_points_num']

        number_table_contents += '{} &{} &{} &{} \\\\\n'.format(
            category,
            before_num, after_num, noise_num
        )

        if 'pos_diff_max' in diff_dict[key]:
            pos_diff_max = np.array(diff_dict[key]['pos_diff_max']) * 1000
            pos_diff_mean = np.array(diff_dict[key]['pos_diff_mean']) * 1000
            pos_diff_min = np.array(diff_dict[key]['pos_diff_min']) * 1000

            distance_max = np.array(diff_dict[key]['distance_max']) * 1000
            distance_mean = np.array(diff_dict[key]['distance_mean']) * 1000
            distance_min = np.array(diff_dict[key]['distance_min']) * 1000

            angle_max = np.array(diff_dict[key]['angle_max']) * 180 / np.pi
            angle_mean = np.array(diff_dict[key]['angle_mean']) * 180 / np.pi
            angle_min = np.array(diff_dict[key]['angle_min']) * 180 / np.pi

            if print_mode == 0:
                table_contents += '{} &{} &{} &{} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} \\\\\n'.format(
                    category,
                    before_num, after_num, noise_num,
                    abs(pos_diff_max[0]), abs(pos_diff_max[1]), abs(pos_diff_max[2]), angle_max,
                    abs(pos_diff_mean[0]), abs(pos_diff_mean[1]), abs(pos_diff_mean[2]), angle_mean,
                    abs(pos_diff_min[0]), abs(pos_diff_min[1]), abs(pos_diff_min[2]), angle_min
                )

                error_table_contents += '{} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} \\\\\n'.format(
                    category,
                    abs(pos_diff_max[0]), abs(pos_diff_max[1]), abs(pos_diff_max[2]), angle_max,
                    abs(pos_diff_mean[0]), abs(pos_diff_mean[1]), abs(pos_diff_mean[2]), angle_mean,
                    abs(pos_diff_min[0]), abs(pos_diff_min[1]), abs(pos_diff_min[2]), angle_min
                )
            elif print_mode == 1:
                table_contents += '{} &{} &{} &{} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f}&{:.2f} &{:.2f} \\\\\n'.format(
                    category,
                    before_num, after_num, noise_num,
                    abs(pos_diff_max[0]), abs(pos_diff_max[1]), abs(pos_diff_max[2]), distance_max, angle_max,
                    abs(pos_diff_mean[0]), abs(pos_diff_mean[1]), abs(pos_diff_mean[2]), distance_mean, angle_mean,
                    abs(pos_diff_min[0]), abs(pos_diff_min[1]), abs(pos_diff_min[2]), distance_min, angle_min
                )

                error_table_contents += '{} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} \\\\\n'.format(
                    category,
                    abs(pos_diff_max[0]), abs(pos_diff_max[1]), abs(pos_diff_max[2]), distance_max, angle_max,
                    abs(pos_diff_mean[0]), abs(pos_diff_mean[1]), abs(pos_diff_mean[2]), distance_mean, angle_mean,
                    abs(pos_diff_min[0]), abs(pos_diff_min[1]), abs(pos_diff_min[2]), distance_min, angle_min
                )
            elif print_mode == 2:
                table_contents += '{} &{} &{} &{} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} \\\\\n'.format(
                    category,
                    before_num, after_num, noise_num,
                    distance_max, angle_max,
                    distance_mean, angle_mean,
                    distance_min, angle_min
                )

                error_table_contents += '{} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} &{:.2f} \\\\\n'.format(
                    category,
                    distance_max, angle_max,
                    distance_mean, angle_mean,
                    distance_min, angle_min
                )


        else:
            if print_mode == 0:
                table_contents += '{} &{} &{} &{} &- &- &- &- &- &- &- &- &- &- &- &-\\\\\n'.format(
                    category,
                    before_num, after_num, noise_num
                )

                error_table_contents += '{} &- &- &- &- &- &- &- &- &- &- &- &-\\\\\n'.format(category)
            elif print_mode == 1:
                table_contents += '{} &{} &{} &{} &- &- &- &- &- &- &- &- &- &- &- &- &- &- &- \\\\\n'.format(
                    category,
                    before_num, after_num, noise_num
                )

                error_table_contents += '{} &- &- &- &- &- &- &- &- &- &- &- &- &- &- &- \\\\\n'.format(category)

            elif print_mode == 2:
                table_contents += '{} &{} &{} &{} &- &- &- &- &- &- \\\\\n'.format(
                    category,
                    before_num, after_num, noise_num
                )

                error_table_contents += '{} &- &- &- &- &- &- \\\\\n'.format(category)

print('merged table\n')
print(table_contents)

print('number only\n')
print(number_table_contents)

print('error only\n')
print(error_table_contents)
