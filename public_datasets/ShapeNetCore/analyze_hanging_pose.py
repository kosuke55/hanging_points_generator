#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse

import numpy as np
import matplotlib.pyplot as plt

from hanging_points_generator.generator_utils import load_json
from shapenet_utils import synset_to_label
from shapenet_utils import label_to_synset
from shapenet_utils import make_category_filling_rate
from shapenet_utils import filling_rate
from shapenet_utils import manipulation_synset


# def make_graph(points, filename):

#     synset = [k for k, v in points.items()]
#     value = [v for k, v in points.items()]

#     idx = np.argsort(np.array(value)).tolist()
#     value.sort()
#     synset = [synset[i] for i in idx]
#     label = [synset_to_label[s] for s in synset]
#     plt.rcParams["font.size"] = 5
#     plt.barh(label, value, height=0.5)
#     plt.xlabel('num hanging points')
#     plt.ylabel('category')
#     plt.tight_layout()
#     plt.savefig(filename, format='png', dpi=200)
#     plt.close()


def make_graph(data, points, filename):
    if isinstance(data, dict):
        label, value = make_category_filling_rate(data, to_label=True)
    elif isinstance(data, list):
        label, value = data

    plt.rcParams["font.size"] = 5
    fig, ax1 = plt.subplots()
    num_points = []
    for l in label:
        if label_to_synset[l] in points:
            num_points.append(points[label_to_synset[l]])
        else:
            num_points.append(0)

    y1 = np.arange(0, len(num_points), 1) * 1.5
    y2 = y1 + 0.5
    color = 'tab:blue'
    bar1 = ax1.barh(
        y1, value, height=0.5, color=color, label='filling rate')
    ax1.set_xlabel('filling rate')
    ax1.set_ylabel('category')
    for x in [0, 0.2, 0.4, 0.6, 0.8, 1.0]:
        ax1.axvline(x=x, linewidth=0.5, color='black')
    ax2 = ax1.twiny()
    color = 'tab:red'
    bar2 = ax2.barh(
        y2, num_points, height=0.5, color=color, label='number of points')
    ax2.set_xlabel('number of points')
    plt.yticks((y1 + y2) / 2, label)
    ax1.legend(
        (bar1[0], bar2[0]), ['filling rate', 'number of points'],
        loc='lower right')
    plt.tight_layout()
    # plt.show()
    plt.savefig(filename, format='png', dpi=200)
    plt.close()


parser = argparse.ArgumentParser()
parser.add_argument(
    '--file', '-f', type=str,
    default='/media/kosuke55/SANDISK/meshdata/shapenet_mini_10/filtering_result.json',
    # default='/media/kosuke55/SANDISK/meshdata/shapenet_mini_10_copy/filtering_result.json',
    help='filtering result file path')
parser.add_argument(
    '--out-file', '-o', type=str,
    default='shapenet_hanging_points.png',
    help='output graph image file name')
args = parser.parse_args()

filtering_result = load_json(args.file)

points_list = {}
for key in filtering_result:
    synset = key.split('_')[0]
    num_points = filtering_result[key]['post_points_num']
    print(synset, num_points)
    if synset not in points_list:
        points_list[synset] = []
    points_list[synset].append(num_points)

points = {}
# for key in points_list:
target_synsets = manipulation_synset()
for key in target_synsets:
    # if key not in points_list:
    #     print(synset_to_label[key])
    #     continue
    if len(points_list[key]) < 10:
        points_list[key].extend(
            [0] * (10 - len(points_list[key])))
    # points[key] = np.mean(points_list[key])
    points[key] = np.median(points_list[key])

# make_graph(points, args.out_file)
data = filling_rate()
data = dict(filter(lambda item: item[0] in target_synsets, data.items()))
make_graph(data, points, args.out_file)

points_list_label = {}
for key in points_list:
    points_list_label[synset_to_label[key]] = points_list[key]

points_label = {}
for key in points:
    points_label[synset_to_label[key]] = points[key]
