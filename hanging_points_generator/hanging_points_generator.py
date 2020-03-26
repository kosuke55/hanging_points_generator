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


def reset_pose(object_id):
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


def generate(urdf_file, required_points_num, enable_gui, save_dir):
    current_dir = os.path.dirname(os.path.abspath(__file__))
    save_dir = os.path.join(current_dir, save_dir)
    urdf_file = os.path.join(current_dir, urdf_file)

    contact_points_list = []
    contact_points_dict = {'urdf_file': urdf_file, 'contact_points': []}

    tree = ET.parse(os.path.join(current_dir, urdf_file))
    root = tree.getroot()
    center = np.array([float(i) for i in root[0].find(
        "inertial").find("origin").attrib['xyz'].split(' ')])

    obj_model = skrobot.models.urdf.RobotModelFromURDF(
        urdf_file=urdf_file)

    if strtobool(enable_gui):
        pybullet.connect(pybullet.GUI)
        viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
        viewer.add(obj_model)
        viewer.show()
    else:
        pybullet.connect(pybullet.DIRECT)
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
    bar_pos = [0, 0, 1]
    bar_rot = [-0.0, 0.7071067811865475, -0.0, 0.7071067811865476]
    bar_id = pybullet.createMultiBody(
        baseMass=0.,
        baseCollisionShapeIndex=bar,
        basePosition=bar_pos,
        baseOrientation=bar_rot)

    loop_num = int(1e10)
    find_count = 0

    height_thresh = 0.5

    try:
        for i in six.moves.range(loop_num):
            # emerge object
            pybullet.setGravity(0, 0, 0)
            reset_pose(object_id)
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

            max_height_contact_point = sorted(contact_points, key=lambda x: x[5][2],
                                              reverse=True)[0][5]
            contact_point_to_hole_vector = np.array(
                [max_height_contact_point[0], 0, 1]) - np.array(
                    max_height_contact_point)
            contact_point = skrobot.coordinates.Coordinates(
                pos=max_height_contact_point,
                rot=skrobot.coordinates.math.rotation_matrix_from_axis(
                    x_axis=[1, 0, 0],
                    y_axis=contact_point_to_hole_vector))
            contact_point_obj = obj_coords.inverse_transformation().transform(
                contact_point).translate(center, 'world')

            contact_point_sphere = skrobot.models.Sphere(0.001, color=[255, 0, 0])
            contact_point_sphere.newcoords(
                skrobot.coordinates.Coordinates(
                    pos=contact_point_obj.worldpos(),
                    rot=contact_point_obj.worldrot()))

            if strtobool(enable_gui):
                viewer.add(contact_point_sphere)

            contact_points_list.append(np.concatenate(
                    [contact_point_obj.T()[:3, 3][None, :],
                     contact_point_obj.T()[:3, :3]]).tolist())

            contact_points_dict['contact_points'] = contact_points_list

            with open(os.path.join(save_dir,
                                   'contact_points.json', ), 'w') as f:
                json.dump(contact_points_dict, f, ensure_ascii=False,
                          indent=4, sort_keys=True, separators=(',', ': '))

            find_count += 1
            if find_count == required_points_num:
                break

    except KeyboardInterrupt:
        pass

    pybullet.disconnect()
    return contact_points_list


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--urdf', '-u', type=str,
                        help='input urdf',
                        default='../urdf/610/scissors/base.urdf')
    parser.add_argument('--required_points_num', '-n', type=int,
                        help='required points number',
                        default=1)
    parser.add_argument('--gui', '-g', type=str,
                        help='gui',
                        default="True")
    args = parser.parse_args()

    contact_points_list = generate(args.urdf,
                                   args.required_points_num,
                                   args.gui)
