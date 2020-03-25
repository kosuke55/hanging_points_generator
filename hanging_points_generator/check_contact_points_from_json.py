#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import json
import os
import skrobot

current_dir = os.path.dirname(os.path.abspath(__file__))

parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('--input', '-i', type=str,
                    help='input contact points',
                    default=current_dir +
                    '/../urdf/610/scissors/contact_points.json')
parser.add_argument('--urdf', '-u', type=str,
                    help='input urdf',
                    default=current_dir
                    + '/../urdf/610/scissors/base.urdf')
args = parser.parse_args()

contact_points_dict = json.load(open(args.input, 'r'))
contact_points = contact_points_dict['contact_points']

current_dir = os.path.dirname(os.path.abspath(__file__))

obj_model = skrobot.models.urdf.RobotModelFromURDF(
    urdf_file=os.path.join(current_dir, args.urdf))

viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
viewer.add(obj_model)
viewer.show()

for i, cp in enumerate(contact_points):
    contact_point_sphere = skrobot.models.Sphere(0.001, color=[255, 0, 0])
    contact_point_sphere.newcoords(
        skrobot.coordinates.Coordinates(pos=cp[0],
                                        rot=cp[1:]))
    viewer.add(contact_point_sphere)
