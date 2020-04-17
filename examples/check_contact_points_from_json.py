#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import os

from hanging_points_generator.hanging_points_generator \
    import check_contact_points


if __name__ == '__main__':
    current_dir = os.path.dirname(os.path.abspath(__file__))

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--input', '-i', type=str,
                        help='input contact points',
                        default='../urdf/610/scissors/contact_points.json')
    parser.add_argument('--urdf', '-u', type=str,
                        help='input urdf',
                        default='../urdf/610/scissors/base.urdf')
    args = parser.parse_args()

    check_contact_points(os.path.join(current_dir, args.input),
                         os.path.join(current_dir, args.urdf))
