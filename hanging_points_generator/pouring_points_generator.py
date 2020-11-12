#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import os
import os.path as osp
import sys

import numpy as np
import pybullet
import skrobot
import six

from hanging_points_generator.generator_utils import get_contact_point
from hanging_points_generator.generator_utils import get_urdf_center
from hanging_points_generator.generator_utils import load_static_urdf
from hanging_points_generator.generator_utils import random_pos
from hanging_points_generator.generator_utils import rotate_object
from hanging_points_generator.generator_utils import save_contact_points
from hanging_points_generator.generator_utils import step


def make_sphere(radius=0.005, use_random_pos=True):
    """Make shapere

    Parameters
    ----------
    radius : float, optional
        shapere radius, by default 0.005
    use_random_pos : bool, optional
        use random posisiton, by default True

    Returns
    -------
    shapre_id
    """
    sphere = pybullet.createCollisionShape(
        pybullet.GEOM_SPHERE, radius=radius)
    sphere_id = pybullet.createMultiBody(
        1, sphere, -1,
        basePosition=random_pos() if use_random_pos else [0, 0, 0])

    return sphere_id


def remove_all_sphere(sphere_ids):
    """removeall sphere

    Parameters
    ----------
    sphere_ids : list
        loaded sphere ids list
    """
    for sphere_id in sphere_ids:
        pybullet.removeBody(sphere_id)
    sphere_ids = []


def remove_out_sphere(sphere_ids):
    """remove non pouring points of shapre

    Parameters
    ----------
    sphere_ids : list
        loaded sphere ids list
    """
    for sphere_id in sphere_ids:
        pos, _ = pybullet.getBasePositionAndOrientation(sphere_id)
        if pos[2] < -0.1:
            pybullet.removeBody(sphere_id)
            sphere_ids.remove(sphere_id)


def get_key_rotatins(use_diagonal=True):
    """Get key 6 or 14 rotations.

    Parameters
    ----------
    use_diagonal : bool, optional
        If True retun 14 key rotations, by default True

    Returns
    -------
    key_rotations : list[list[float]]
        list of key rotation
    """
    if use_diagonal:
        key_rotations = [
            pybullet.getQuaternionFromEuler([0, 0, 0]),
            pybullet.getQuaternionFromEuler([np.pi / 4, 0, 0]),
            pybullet.getQuaternionFromEuler([np.pi / 2, 0, 0]),
            pybullet.getQuaternionFromEuler([np.pi / 4 * 3, 0, 0]),
            pybullet.getQuaternionFromEuler([np.pi, 0, 0]),
            pybullet.getQuaternionFromEuler([-np.pi / 4 * 3, 0, 0]),
            pybullet.getQuaternionFromEuler([-np.pi / 2, 0, 0]),
            pybullet.getQuaternionFromEuler([-np.pi / 4, 0, 0]),
            pybullet.getQuaternionFromEuler([0, np.pi / 4, 0]),
            pybullet.getQuaternionFromEuler([0, np.pi / 2, 0]),
            pybullet.getQuaternionFromEuler([0, np.pi / 4 * 3, 0]),
            pybullet.getQuaternionFromEuler([0, -np.pi / 4 * 3, 0]),
            pybullet.getQuaternionFromEuler([0, -np.pi / 2, 0]),
            pybullet.getQuaternionFromEuler([0, -np.pi / 4, 0])
        ]
    else:
        key_rotations = [
            pybullet.getQuaternionFromEuler([0, 0, 0]),
            pybullet.getQuaternionFromEuler([np.pi / 2, 0, 0]),
            pybullet.getQuaternionFromEuler([np.pi, 0, 0]),
            pybullet.getQuaternionFromEuler([-np.pi / 2, 0, 0]),
            pybullet.getQuaternionFromEuler([0, np.pi / 2, 0]),
            pybullet.getQuaternionFromEuler([0, -np.pi / 2, 0])
        ]

    return key_rotations


def get_contact_points(
        object_id, object_center, sphere_ids, contact_points_list=None,
        x_axis=[1, 0, 0], y_axis=[0, 0, -1], use_min_height=False):
    """get contact points between object and spheres

    Parameters
    ----------
    object_id : int
    object_center : list [x, y, z]
    sphere_ids : list of shapre ids
    x_axis : list, optional
        contact pose x_axis direction, by default [1, 0, 0]
    y_axis : list, optional
        contact pose y_axis direction, by default [0, 0, -1]
    use_min_height : bool, optional
        if True use min height contact point, by default False

    Returns
    -------
    contact_points_list
    """
    if contact_points_list is None:
        contact_points_list = []

    object_pos, object_rot = pybullet.getBasePositionAndOrientation(object_id)
    obj_coords = skrobot.coordinates.Coordinates(
        pos=object_pos,
        rot=skrobot.coordinates.math.xyzw2wxyz(object_rot))

    for sphere_id in sphere_ids:
        pose = get_contact_point(
            object_id, object_center, sphere_id, obj_coords,
            x_axis, y_axis, use_min_height)
        if pose is None:
            continue
        # contact_points = pybullet.getContactPoints(object_id, sphere_id)
        # if len(contact_points) == 0:
        #     continue

        # if use_min_height:
        #     contact_point = sorted(
        #         contact_points, key=lambda x: x[5][2])[0][5]
        # else:
        #     contact_point = sorted(
        #         contact_points, key=lambda x: x[5][2], reverse=True)[0][5]

        # rot = skrobot.coordinates.math.rotation_matrix_from_axis(
        #     x_axis, y_axis)

        # contact_point = skrobot.coordinates.Coordinates(
        #     pos=contact_point, rot=rot)

        # contact_point_obj = obj_coords.inverse_transformation().transform(
        #     contact_point).translate(object_center, 'world')

        # pose = np.concatenate(
        #     [contact_point_obj.T()[:3, 3][None, :],
        #      contact_point_obj.T()[:3, :3]]).tolist()

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

    key_rotations = get_key_rotatins()

    try_num = len(key_rotations)
    find_count = 0
    is_pouring_object = True

    try:
        for try_count in six.moves.range(try_num):
            rotate_object(object_id, key_rotations[try_count])
            sphere_ids = []
            pybullet.setGravity(0, 0, gravity)

            for _ in range(30):
                sphere_ids.append(make_sphere())
                step(10)
                remove_out_sphere(sphere_ids)

            for f in [[0, 0], [-5, 0], [5, 0], [0, -5], [0, 5], [0, 0]]:
                pybullet.setGravity(f[0], f[1], gravity)
                for _ in range(10):
                    step(10)
                    remove_out_sphere(sphere_ids)

            pouring_points_list = get_contact_points(
                object_id, object_center, sphere_ids, pouring_points_list)
            pouring_points_dict['contact_points'] = pouring_points_list

            save_contact_points(
                osp.join(save_dir, 'pouring_points.json'), pouring_points_dict)

            remove_all_sphere(sphere_ids)

    except KeyboardInterrupt:
        sys.exit()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--urdf', '-u', type=str,
                        help='input urdf',
                        default='/media/kosuke/SANDISK/meshdata/ycb_hanging_object/urdf/025_mug/base.urdf')
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
