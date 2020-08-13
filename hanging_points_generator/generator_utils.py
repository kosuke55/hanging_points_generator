#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import os
import os.path as osp
from pathlib import Path

import numpy as np
import pybullet
import skrobot
import xml.etree.ElementTree as ET
from filelock import FileLock


def random_pos(z_offset=0.2):
    """generate random position for dropping sphare

    Parameters
    ----------
    z_offset : float, optional
        z offset, by default 0.2

    Returns
    -------
    pos list [x, y, z]
        random position

    """
    pos = [np.random.randn() * 0.05,
           np.random.randn() * 0.05,
           z_offset + np.random.rand() * 0.1]
    return pos


def random_rot():
    """generate random rotation

    Returns
    -------
    rot list
        quaterenion [x, y, z, w]

    """
    roll = np.random.rand() * np.pi
    pitch = np.random.rand() * np.pi
    yaw = np.random.rand() * np.pi
    rot = pybullet.getQuaternionFromEuler([roll, pitch, yaw])

    return rot


def random_rotate_object(object_id):
    """Rotate to the random roatation

    Parameters
    ----------
    object_id : int

    """
    pos, _ = pybullet.getBasePositionAndOrientation(object_id)
    rot = random_rot()
    pybullet.resetBasePositionAndOrientation(
        object_id, pos, rot)


def load_static_urdf(urdf_file, position=[0, 0, 0], orientation=[0, 0, 0, 1]):
    """load mass=0 urdf by pybullet

    Parameters
    ----------
    urdf_file : str
    position : list, optional
        position, by default [0, 0, 0]
    orientation : list, optional
        quaternion orientation, by default [0, 0, 0, 1]

    Returns
    -------
    object_id ; int
     pybullet object id

    """
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


def step(n=1):
    """pybullet step simulation

    Parameters
    ----------
    n : int, optional
        the number of step, by default 1

    """
    for _ in range(n):
        pybullet.stepSimulation()


def save_contact_points(
        save_dir, save_file_name, contact_points_dict, filelock=False):
    """Save contact points json file with filelock

    Parameters
    ----------
    save_dir : str
    save_file_name : str
    contact_points_dict : dict

    """
    if filelock:
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
    else:
        with open(os.path.join(save_dir, save_file_name), 'w') as f:
            json.dump(contact_points_dict, f, ensure_ascii=False,
                      indent=4, sort_keys=True, separators=(',', ': '))


def get_urdf_center(urdf_file):
    """Get urdf center

    Parameters
    ----------
    urdf_file : str

    Returns
    -------
    ceter : list [x, y ,z]
    """
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    center = np.array([float(i) for i in root[0].find(
        "inertial").find("origin").attrib['xyz'].split(' ')])
    return center


def get_contact_point(
        object_id, object_center,  contact_object_id, obj_coords=None,
        x_axis=[1, 0, 0], y_axis=[0, 0, -1], use_min_height=False):
    contact_points = pybullet.getContactPoints(object_id, contact_object_id)
    if len(contact_points) == 0:
        return None

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

    if obj_coords is None:
        object_pos, object_rot = pybullet.getBasePositionAndOrientation(
            object_id)
        obj_coords = skrobot.coordinates.Coordinates(
            pos=object_pos,
            rot=skrobot.coordinates.math.xyzw2wxyz(object_rot))

    contact_point_obj = obj_coords.inverse_transformation().transform(
        contact_point).translate(object_center, 'world')

    pose = np.concatenate(
        [contact_point_obj.T()[:3, 3][None, :],
         contact_point_obj.T()[:3, :3]]).tolist()

    return pose


def load_multiple_contact_points(
        base_dir, json_name='contact_points.json'):
    """Load multiple contact points

    This is for eos fancy_dir

    Parameters
    ----------
    base_dir : str
        base_dir/*/json_file
    json_name : str, optional
        'contact_points.json' or 'pouring_points.json',
        by default 'contact_points.json'

    Returns
    -------
    base_cp_dict
        merged contact_points dict

    """
    paths = list(sorted(Path(base_dir).glob(osp.join('*', json_name))))

    for i, path in enumerate(paths):
        if i == 0:
            base_cp_file = str(path)
            base_cp_dict = json.load(open(base_cp_file, 'r'))
        else:
            cp_file = str(path)
            cp_dict = json.load(open(cp_file, 'r'))
            for cp in cp_dict['contact_points']:
                base_cp_dict['contact_points'].append(cp)

    return base_cp_dict
