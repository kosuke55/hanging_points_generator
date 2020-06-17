#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import os

from hanging_points_generator.hp_generator \
    import check_contact_points

current_dir = os.path.dirname(os.path.abspath(__file__))

parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('--input', '-i', type=str,
                    help='input contact points',
                    default='urdf/019_pitcher_base/')
parser.add_argument('--clustering', '-c', type=int,
                    help='dbscan clustering',
                    default=1)
args = parser.parse_args()

check_contact_points(
    os.path.join(current_dir, args.input, 'contact_points.json'),
    os.path.join(current_dir, args.input, 'textured.urdf'),
    # os.path.join(current_dir, args.input, 'base.urdf'),
    clustering=args.clustering)
