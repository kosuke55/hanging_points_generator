#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import json
import numpy as np
import os
import pybullet
import pybullet_data
import skrobot
import six
import xml.etree.ElementTree as ET

from distutils.util import strtobool
from math import pi


def reset_pose():
    x = (np.random.rand() - 0.5) * 0.1
    y = (np.random.rand() - 0.5) * 0.1
    z = 1 + (np.random.rand() - 0.5) * 0.1
    roll = np.random.rand() * pi
    pitch = np.random.rand() * pi
    yaw = np.random.rand() * pi
    pybullet.setGravity(0, 0, 0)
    pybullet.resetBasePositionAndOrientation(
        object_id,
        [x, y, z],
        pybullet.getQuaternionFromEuler([roll, pitch, yaw]))


parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('--urdf', '-u', type=str,
                    help='input urdf',
                    default='./urdf/610/scissors/base.urdf')
parser.add_argument('--gui', '-g', type=str,
                    help='gui',
                    default="True")
args = parser.parse_args()

current_dir = os.path.dirname(os.path.abspath(__file__))
urdf_file = os.path.join(current_dir, args.urdf)
contact_points_dict = {'urdf_file': urdf_file, 'contact_points': []}

tree = ET.parse(os.path.join(current_dir, args.urdf))
root = tree.getroot()
center = np.array([float(i) for i in root[0].find(
    "inertial").find("origin").attrib['xyz'].split(' ')])

obj_model = skrobot.models.urdf.RobotModelFromURDF(
    urdf_file=urdf_file)

if strtobool(args.gui):
    physicsClient = pybullet.connect(pybullet.GUI)
    viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
    viewer.add(obj_model)
    viewer.show()
else:
    physicsClient = pybullet.connect(pybullet.DIRECT)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
gravity = -10
pybullet.setGravity(0, 0, gravity)
pybullet.setTimeStep(1 / 240.0)
pybullet.loadURDF("plane.urdf")

StartPos = [0, 0, 0]
StartOrientation = pybullet.getQuaternionFromEuler([0, 0, 0])
object_id = pybullet.loadURDF(urdf_file,
                              StartPos, StartOrientation)

bar = pybullet.createCollisionShape(
    pybullet.GEOM_CYLINDER,
    radius=0.005,
    height=1)

bar_id = pybullet.createMultiBody(
    baseMass=0.,
    baseCollisionShapeIndex=bar,
    basePosition=[0, 0, 1],
    baseOrientation=pybullet.getQuaternionFromEuler(
        [0, 1.5707963267948966, 0]))

loop_num = int(1e10)
find_count = 0

height_thresh = 0.5

for i in six.moves.range(loop_num):
    # emerge object
    pybullet.setGravity(0, 0, 0)
    reset_pose()
    pybullet.stepSimulation()
    contact_points = pybullet.getContactPoints(object_id, bar_id)
    if contact_points:
        continue
    pybullet.setGravity(0, 0, gravity)

    failed = False
    for _ in range(240 * 2):
        pos, rot = pybullet.getBasePositionAndOrientation(object_id)
        if pos[2] < height_thresh:
            failed = True
            break
        pybullet.stepSimulation()
    if failed:
        continue

    contact_points = pybullet.getContactPoints(object_id, bar_id)
    pos, rot = pybullet.getBasePositionAndOrientation(object_id)
    if len(contact_points) == 0:
        continue

    print("Find the hanging part {}".format(find_count))

    obj_coords = skrobot.coordinates.Coordinates(
        pos=pos,
        rot=skrobot.coordinates.math.xyzw2wxyz(rot))

    # Use max height contact_point
    contact_point = np.array(obj_coords.inverse_transform_vector(sorted(
        contact_points, key=lambda x: x[5][2], reverse=True)[0][5]))
    contact_point_sphere = skrobot.models.Sphere(0.001, color=[255, 0, 0])
    contact_point_sphere.newcoords(
        skrobot.coordinates.Coordinates(
            pos=obj_coords.inverse_transform_vector(contact_point + center)))

    if strtobool(args.gui):
        viewer.add(contact_point_sphere)

    contact_points_dict['contact_points'].append(
        (contact_point + center).tolist())

    with open("contact_points.json", "w") as f:
        json.dump(contact_points_dict, f, ensure_ascii=False,
                  indent=4, sort_keys=True, separators=(',', ': '))

    find_count += 1

pybullet.disconnect()
