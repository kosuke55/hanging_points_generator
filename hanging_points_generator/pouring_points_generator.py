#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import json
import os
import time
from pathlib import Path

import numpy as np
import pybullet
import pybullet_data
import skrobot
import xml.etree.ElementTree as ET
from filelock import FileLock


def random_pos(z_offset=0.2):
    pos = [np.random.randn() * 0.05,
           np.random.randn() * 0.05,
           z_offset + np.random.rand() * 0.1]
    return pos


def load_static_urdf(urdf_file, position=[0, 0, 0], orientation=[0, 0, 0, 1]):
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    root[0].find('inertial').find('mass').attrib['value'] = '0'
    urdf_path = Path(urdf_file)
    static_urdf_path = urdf_path.parent / 'static.urdf'
    tree.write(str(static_urdf_path),
               encoding='utf-8', xml_declaration=True)
    object_id = pybullet.loadURDF(str(static_urdf_path),
                                  position, orientation)

    if static_urdf_path.exists():
        static_urdf_path.unlink()

    return object_id


def make_sphere(radius=0.005, use_random_pos=True):
    sphere = pybullet.createCollisionShape(
        pybullet.GEOM_SPHERE, radius=radius)
    sphere_id = pybullet.createMultiBody(
        1, sphere, -1,
        basePosition=random_pos() if use_random_pos else [0, 0, 0])

    return sphere_id


def step(n=1):
    for _ in range(n):
        pybullet.stepSimulation()


def remove_out_sphere(sphere_ids):
    for sphere_id in sphere_ids:
        pos, _ = pybullet.getBasePositionAndOrientation(sphere_id)
        if pos[2] < -0.1:
            pybullet.removeBody(sphere_id)
            sphere_ids.remove(sphere_id)


def save_contact_points(
        save_dir, save_file_name, contact_points_dict):
    if os.path.exists(os.path.join(save_dir, save_file_name)):
        filelock_path = os.path.join(
            save_dir, save_file_name + '.lock')
        with FileLock(filelock_path):
            with open(os.path.join(save_dir, save_file_name), 'r') as f:
                contact_points_dict_existed = json.load(f)
                for c in contact_points_dict['contact_points']:
                    contact_points_dict_existed['contact_points'].append(c)
                # find_count = len(
                #     contact_points_dict_existed['contact_points'])

        filelock_path = os.path.join(
            save_dir, save_file_name + '.lock')
        with FileLock(filelock_path):
            with open(os.path.join(save_dir, save_file_name), 'w') as f:
                json.dump(contact_points_dict_existed, f, ensure_ascii=False,
                          indent=4, sort_keys=True, separators=(',', ': '))
    else:
        with open(os.path.join(save_dir, save_file_name), 'w') as f:
            json.dump(contact_points_dict, f, ensure_ascii=False,
                      indent=4, sort_keys=True, separators=(',', ': '))
        # find_count += 1


def get_urdf_center(urdf_file):
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    center = np.array([float(i) for i in root[0].find(
        "inertial").find("origin").attrib['xyz'].split(' ')])
    return center


def get_contact_points(
        object_id, object_center, sphere_ids,
        x_axis=[1, 0, 0], y_axis=[0, 0, -1], use_min_height=False):
    contact_points_list = []
    object_pos, object_rot = pybullet.getBasePositionAndOrientation(object_id)
    obj_coords = skrobot.coordinates.Coordinates(
        pos=object_pos,
        rot=skrobot.coordinates.math.xyzw2wxyz(object_rot))

    for sphere_id in sphere_ids:
        contact_points = pybullet.getContactPoints(object_id, sphere_id)
        if len(contact_points) == 0:
            continue

        if use_min_height:
            contact_point = sorted(
                contact_points, key=lambda x: x[5][2])[0][5]
        else:
            contact_point = sorted(
                contact_points, key=lambda x: x[5][2], reverse=True)[0][5]

        rot = skrobot.coordinates.math.rotation_matrix_from_axis(
            x_axis=x_axis, y_axis=y_axis)

        contact_point = skrobot.coordinates.Coordinates(
            pos=contact_point, rot=rot)

        contact_point_obj = obj_coords.inverse_transformation().transform(
            contact_point).translate(object_center, 'world')

        pose = np.concatenate(
            [contact_point_obj.T()[:3, 3][None, :],
             contact_point_obj.T()[:3, :3]]).tolist()

        contact_points_list.append(pose)

    return contact_points_list


def generate(urdf_file, required_points_num,
             enable_gui, viz_obj, save_dir):
    """Drop the ball and find the pouring points.

    Parameters
    ----------
    urdf_file : str
        udfr file path
    required_points_num : int
        required points number
    enable_gui : bool
    viz_obj : bool
        viz obj with contactpoints
    save_dir : str
        save dir path
    """

    current_dir = os.path.dirname(os.path.abspath(__file__))

    pouring_points_list = []
    pouring_points_dict = {'urdf_file': urdf_file, 'contact_points': []}

    object_center = get_urdf_center(urdf_file)

    if enable_gui:
        pybullet.connect(pybullet.GUI)
        pybullet.resetDebugVisualizerCamera(
            cameraDistance=0.3,
            cameraYaw=90,
            cameraPitch=0,
            cameraTargetPosition=[0, 0, 0])
    else:
        pybullet.connect(pybullet.DIRECT)

    gravity = -10
    timestep = 240.
    pybullet.setTimeStep(1 / timestep)

    object_id = load_static_urdf(urdf_file, [0, 0, 0], [0, 0, 0, 1])

    try_num = 10000
    find_count = 0
    is_pouring_object = True

    sphere_ids = []
    pybullet.setGravity(0, 0, gravity)

    for _ in range(100):
        sphere_ids.append(make_sphere())
        step(10)
        remove_out_sphere(sphere_ids)

    for f in [[0, 0], [-5, 0], [5, 0], [0, -5], [0, 5], [0, 0]]:
        pybullet.setGravity(f[0], f[1], gravity)
        for _ in range(10):
            step(10)
            remove_out_sphere(sphere_ids)

    pouring_points_list = get_contact_points(
        object_id, object_center, sphere_ids)
    pouring_points_dict['contact_points'] = pouring_points_list

    save_contact_points(
        save_dir, 'pouring_poitns.json', pouring_points_dict)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--urdf', '-u', type=str,
                        help='input urdf',
                        default='/media/kosuke/SANDISK/meshdata/ycb_hanging_object/urdf/025_mug/base.urdf')
    # default='../urdf/610/scissors/base.urdf')
    parser.add_argument('--required_points_num', '-n', type=int,
                        help='required points number',
                        default=1)
    parser.add_argument('--gui', '-g', type=str,
                        help='gui',
                        default="True")
    parser.add_argument('--viz_obj', '-v', type=str,
                        help='viz obj with contactpoints',
                        default="False")
    args = parser.parse_args()

    contact_points_list = generate(args.urdf,
                                   args.required_points_num,
                                   args.gui,
                                   args.viz_obj,
                                   os.path.dirname(args.urdf))
