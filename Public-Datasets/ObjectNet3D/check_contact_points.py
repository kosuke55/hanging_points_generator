#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import os

from hanging_points_generator.hanging_points_generator \
    import check_contact_points

current_dir = os.path.dirname(os.path.abspath(__file__))

parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('--input', '-i', type=str,
                    help='input contact points',
                    default='urdf/cap/01')
args = parser.parse_args()

check_contact_points(
    os.path.join(current_dir, args.input, 'contact_points.json'),
    os.path.join(current_dir, args.input, 'base.urdf'))