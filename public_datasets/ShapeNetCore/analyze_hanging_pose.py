#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse

import numpy as np
import matplotlib.pyplot as plt

from hanging_points_generator.generator_utils import load_json
from shapenet_utils.shapenet_synset_dict import synset_to_label


def make_graph(points, filename):

    synset = [k for k, v in points.items()]
    value = [v for k, v in points.items()]

    idx = np.argsort(np.array(value)).tolist()
    value.sort()
    synset = [synset[i] for i in idx]
    label = [synset_to_label[s] for s in synset]
    plt.rcParams["font.size"] = 5
    plt.barh(label, value, height=0.5)
    plt.xlabel('num hanging points')
    plt.ylabel('category')
    plt.tight_layout()
    plt.savefig(filename, format='png', dpi=200)
    plt.close()


parser = argparse.ArgumentParser()
parser.add_argument(
    '--file', '-f', type=str,
    default='/media/kosuke55/SANDISK/meshdata/shapenet_mini/filtering_result.json',
    help='filtering result file path')
parser.add_argument(
    '--out-file', '-o', type=str,
    default='shapenet_hanging_points.png',
    help='output graph image file name')


args = parser.parse_args()

filtering_result = load_json(args.file)

points = {}
for key in filtering_result:
    synset = key.split('_')[0]
    num_points = filtering_result[key]['post_points_num']
    print(synset, num_points)
    if synset not in points:
        points[synset] = []
    points[synset].append(num_points)

for key in points:
    points[key] = np.mean(points[key])

make_graph(points, args.out_file)
