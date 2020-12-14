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
from eos import make_fancy_output_dir

from hanging_points_generator.generator_utils import get_contact_point
from hanging_points_generator.generator_utils import get_urdf_center
from hanging_points_generator.generator_utils import load_static_urdf
from hanging_points_generator.generator_utils import random_pos
from hanging_points_generator.generator_utils import rotate_object
from hanging_points_generator.generator_utils import rotate_local
from hanging_points_generator.generator_utils import save_contact_points
from hanging_points_generator.generator_utils import step


def make_sphere(radius=0.005, pos=[0, 0, 0],
                use_random_pos=False, rgba=None):
    """Make shapere

    Parameters
    ----------
    radius : float, optional
        shapere radius, by default 0.005
    use_random_pos : bool, optional
        use random posisiton, by default True
    rgba : list[float float float float]
        if None, random color, by default None

    Returns
    -------
    shapre_id
    """
    sphere = pybullet.createCollisionShape(
        pybullet.GEOM_SPHERE, radius=radius)
    sphere_id = pybullet.createMultiBody(
        1, sphere, -1,
        basePosition=random_pos() if use_random_pos else pos)
    if rgba is not None:
        pybullet.changeVisualShape(sphere_id, -1, rgbaColor=rgba)

    return sphere_id


def make_2daabb_pattern_spheres(
        object_id, radius=0.005,
        space=0.05, z_space=0.1, rgba=None):
    """Make spheres above ofject bbaa

    Parameters
    ----------
    object_id : int
    radius : float, optional
        by default 0.01
    space : float, optional
        space between two spheres, by default 0.05
    z_space : float, optional
        z space between bbaa and spheres, by default 0.1
    rgba : list[float float float float]
        if None, random color, by default None

    Returns
    -------
    sphere_ids : list[int]
        list of sphere_id
    pos_list : list[list[float float float]]
        list of [x y z]
    """
    aabb = pybullet.getAABB(object_id)
    sphere_ids, pos_list = make_pattern_spheres(
        x_min=aabb[0][0], x_max=aabb[1][0],
        y_min=aabb[0][1], y_max=aabb[1][1],
        z=aabb[1][2] + z_space, radius=radius, space=space, rgba=rgba)

    return sphere_ids, pos_list


def make_pattern_spheres(
        x_min, x_max,
        y_min, y_max,
        z, radius=0.01, space=0.05, rgba=None):
    """Make rect patttern spheres

    Parameters
    ----------
    x_min : float
        x_min of rect
    x_max : float
        x_max of rect
    y_min : float
        y_min of rect
    y_max : float
        y_max of rect
    z : float
        z value of spheres
    radius : float, optional
        by default 0.01
    space : float, optional
        space between two spheres, by default 0.05
    rgba : list[float float float float]
        if None, random color, by default None

    Returns
    -------
    sphere_ids : list[int]
        list of sphere_id
    pos_list : list[list[float float float]]
        list of [x y z]
    """

    interval = radius * 2 + space

    w = x_max - x_min
    num_x = int(np.ceil(w / interval))

    d = y_max - y_min
    num_y = int(np.ceil(d / interval))

    sphere_ids = []
    pos_list = []
    for i in range(num_x):
        for j in range(num_y):
            x = x_min + interval * i
            y = y_min + interval * j
            sphere = pybullet.createCollisionShape(
                pybullet.GEOM_SPHERE, radius=radius)
            sphere_id = pybullet.createMultiBody(
                1, sphere, -1, basePosition=[x, y, z])
            if rgba is not None:
                pybullet.changeVisualShape(sphere_id, -1, rgbaColor=rgba)
            sphere_ids.append(sphere_id)
            pos_list.append([x, y, z])

    return sphere_ids, pos_list


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
    return sphere_ids


def remove_out_sphere(sphere_ids, pos_list=None):
    """remove non pouring points of shapre

    Parameters
    ----------
    sphere_ids : list[int]
        loaded sphere ids list
    pos_list : list[list[float float float]]
        list of [x y z], by default None

    Returns
    -------
    remained_sphere_ids : list[int]
    """
    remained_sphere_ids = []
    pos_in_list = []
    pos_out_list = []
    if pos_list is None:
        for sphere_id in sphere_ids:
            pos, _ = pybullet.getBasePositionAndOrientation(sphere_id)
            if pos[2] < -0.1:
                pybullet.removeBody(sphere_id)
            else:
                remained_sphere_ids.append(sphere_id)
        return remained_sphere_ids

    else:
        for sphere_id, pos_init in zip(sphere_ids, pos_list):
            pos, _ = pybullet.getBasePositionAndOrientation(sphere_id)
            if pos[2] < -0.1:
                pybullet.removeBody(sphere_id)
                pos_out_list.append(pos_init)
            else:
                remained_sphere_ids.append(sphere_id)
                pos_in_list.append(pos_init)
        return remained_sphere_ids, pos_in_list, pos_out_list


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
            pybullet.getQuaternionFromEuler([np.pi / 2, 0, 0]),
            pybullet.getQuaternionFromEuler([np.pi, 0, 0]),
            pybullet.getQuaternionFromEuler([-np.pi / 2, 0, 0]),
            pybullet.getQuaternionFromEuler([0, np.pi / 2, 0]),
            pybullet.getQuaternionFromEuler([0, -np.pi / 2, 0]),

            pybullet.getQuaternionFromEuler([np.pi / 4, 0, 0]),
            pybullet.getQuaternionFromEuler([np.pi / 4 * 3, 0, 0]),
            pybullet.getQuaternionFromEuler([-np.pi / 4 * 3, 0, 0]),
            pybullet.getQuaternionFromEuler([-np.pi / 4, 0, 0]),

            pybullet.getQuaternionFromEuler([0, np.pi / 4, 0]),
            pybullet.getQuaternionFromEuler([0, np.pi / 4 * 3, 0]),
            pybullet.getQuaternionFromEuler([0, -np.pi / 4 * 3, 0]),
            pybullet.getQuaternionFromEuler([0, -np.pi / 4, 0]),

            pybullet.getQuaternionFromEuler([0, 0, np.pi / 4]),
            pybullet.getQuaternionFromEuler([0, 0, np.pi / 4 * 3]),
            pybullet.getQuaternionFromEuler([0, 0, -np.pi / 4 * 3]),
            pybullet.getQuaternionFromEuler([0, 0, -np.pi / 4]),
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
        x_axis=[0, 0, -1], y_axis=[1, 0, 0], use_min_height=False):
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

        contact_points_list.append(pose)

    return contact_points_list


def shake(object_id, base_rotation, shake_step, shake_angle_max):
    """Shake an object around base_rotation

    Parameters
    ----------
    object_id : int
        pybullet object id
    base_rotation : tuple(float float float)
        shake an object around this rotation
    shake_step : int
        How many angles max should be divided
    shake_angle_max : float
        Maximum value of shake tilt angle
    """
    shake_angle_list = np.arange(
        0, shake_angle_max, shake_angle_max / shake_step)
    shake_angle_list = np.hstack((shake_angle_list, shake_angle_list[::-1]))
    shake_angle_list = np.hstack((shake_angle_list, -shake_angle_list))

    for axis in ['x', 'y', 'z']:
        for shake_angle in shake_angle_list:
            rot = rotate_local(base_rotation, shake_angle, axis)
            rotate_object(object_id, rot)
            step(1)


def generate(urdf_file, required_points_num,
             enable_gui, viz_obj, save_dir, radius=0.005,
             pattern_spheres=True, repeat_per_rotation=3, apply_force=False):
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
    radius : float, optional
        shapere radius, by default 0.005
    pattern_spheres : bool, optional
        by default True
    repeat_per_rotation : int, optional
        How many times to pour per rotation, by default 3
    apply_force : bool, optional
        Whether to apply an external force to check the stability.
        by default False
    """

    save_dir = make_fancy_output_dir(osp.join(save_dir, 'pouring_points'),
                                     save_environ=False, save_command=False,
                                     save_git=False, save_gitignore=False,
                                     save_pip=False)

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

    gravity = -9.8
    timestep = 240.
    pybullet.setTimeStep(1 / timestep)

    object_id = load_static_urdf(urdf_file, [0, 0, 0], [0, 0, 0, 1])

    key_rotations = get_key_rotatins()

    try:
        for key_rotation in key_rotations:
            for repeat_idx in range(repeat_per_rotation):
                rotate_object(object_id, key_rotation)
                sphere_ids = []
                pybullet.setGravity(0, 0, gravity)

                if pattern_spheres:
                    sphere_ids, pos_list = make_2daabb_pattern_spheres(
                        object_id, radius=radius, space=0.01, z_space=0.1)
                    step(300)

                    sphere_ids, pos_in_list, pos_out_list = remove_out_sphere(
                        sphere_ids, pos_list)

                    for _ in range(2):
                        for pos in pos_in_list:
                            sphere_ids.append(
                                make_sphere(radius=radius, pos=pos))
                        step(300)
                    shake(object_id, key_rotation,
                          shake_step=100, shake_angle_max=np.pi / 6)
                    step(300)

                    if apply_force:
                        for f in [[0, 0], [-5, 0], [5, 0],
                                  [0, -5], [0, 5], [0, 0]]:

                            pybullet.setGravity(f[0], f[1], gravity)
                            step(100)

                else:
                    for _ in range(30):
                        sphere_ids.append(make_sphere(use_random_pos=True))
                        step(10)
                        remove_out_sphere(sphere_ids)

                    for f in [[0, 0], [-5, 0], [5, 0],
                              [0, -5], [0, 5], [0, 0]]:
                        pybullet.setGravity(f[0], f[1], gravity)
                        for _ in range(10):
                            step(1)
                            remove_out_sphere(sphere_ids)

                sphere_ids = remove_out_sphere(sphere_ids)
                pouring_points_list = get_contact_points(
                    object_id, object_center, sphere_ids, pouring_points_list)
                pouring_points_dict['contact_points'] = pouring_points_list

                save_contact_points(
                    osp.join(
                        save_dir,
                        'pouring_points.json'),
                    pouring_points_dict)

                if len(sphere_ids) == 0:
                    sphere_ids = remove_all_sphere(sphere_ids)
                    break
                sphere_ids = remove_all_sphere(sphere_ids)

    except KeyboardInterrupt:
        sys.exit()

    pybullet.disconnect()
    return pouring_points_list


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--urdf', '-u', type=str,
        help='input urdf',
        default='/media/kosuke/SANDISK/meshdata/ycb_hanging_object/urdf/025_mug/base.urdf')
    parser.add_argument(
        '--required_points_num', '-n', type=int,
        help='required points number',
        default=1)
    parser.add_argument(
        '--gui', '-g', type=str,
        help='gui',
        default="True")
    parser.add_argument(
        '--viz_obj', '-v', type=str,
        help='viz obj with contactpoints',
        default="False")
    parser.add_argument(
        '--pattern', '-p', type=int,
        help='make pattern spheres',
        default=1)

    args = parser.parse_args()

    contact_points_list = generate(args.urdf,
                                   args.required_points_num,
                                   args.gui,
                                   args.viz_obj,
                                   os.path.dirname(args.urdf),
                                   pattern_spheres=args.pattern)
