#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import json
import os
import os.path as osp
from pathlib import Path
import sys
from math import pi

import numpy as np
import pybullet
import pybullet_data
import skrobot
import six
import trimesh
import xml.etree.ElementTree as ET
from eos import make_fancy_output_dir
from filelock import FileLock
from sklearn.cluster import DBSCAN

from hanging_points_generator.renderer import Renderer
from hanging_points_generator.generator_utils import add_list
from hanging_points_generator.generator_utils import cluster_contact_points
from hanging_points_generator.generator_utils import filter_penetration
from hanging_points_generator.generator_utils import load_multiple_contact_points
from hanging_points_generator.generator_utils import save_contact_points


def generate(urdf_file, required_points_num, try_num,
             enable_gui, viz_obj, save_dir,
             hook_type='just_bar', render=False):
    current_dir = os.path.dirname(os.path.abspath(__file__))
    base_save_dir = osp.dirname(save_dir)
    category_name = Path(urdf_file).parent.name
    save_dir = make_fancy_output_dir(osp.join(save_dir, 'contact_points'),
                                     save_environ=False, save_command=False,
                                     save_git=False, save_gitignore=False,
                                     save_pip=False)
    pid = os.getpid()

    contact_points_list = []
    contact_points_dict = {'urdf_file': urdf_file, 'contact_points': []}

    tree = ET.parse(urdf_file)
    root = tree.getroot()
    center = np.array([float(i) for i in root[0].find(
        "inertial").find("origin").attrib['xyz'].split(' ')])

    if enable_gui:
        pybullet.connect(pybullet.GUI)
        pybullet.resetDebugVisualizerCamera(
            cameraDistance=0.3,
            cameraYaw=90,
            cameraPitch=0,
            cameraTargetPosition=[0.15, 0, 1])
    else:
        pybullet.connect(pybullet.DIRECT)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    gravity = -9.8
    timestep = 1000.
    pybullet.setTimeStep(1 / timestep)

    # Just bar
    if hook_type == 'just_bar':
        hook_id = pybullet.createMultiBody(
            baseMass=0.,
            baseCollisionShapeIndex=pybullet.createCollisionShape(
                pybullet.GEOM_CYLINDER,
                radius=0.0025,
                height=0.3),
            basePosition=[0, 0, 1],
            baseOrientation=[-0.0, 0.7071067811865475, -0.0, 0.7071067811865476])
        hook_direction = np.array([1., 0, 0])
        pybullet.loadURDF(
            os.path.join(current_dir, 'urdf/hook/plate_visual.urdf'),
            [-0.15, 0, 0.8], [0, 0, 0, 1])

    # hook urdf model
    else:
        hook_id = pybullet.loadURDF(
            os.path.join(current_dir, 'urdf/hook/hook.urdf'),
            [0, 0, 1], [0, 0, 0, 1])
        hook_direction = np.array([1, 0, np.tan(np.pi / 2 - 1.2)])
        pybullet.loadURDF(
            os.path.join(current_dir, 'urdf/hook/plate.urdf'),
            [0, 0, 1], [0, 0, 0, 1])

    hook_direction /= np.linalg.norm(hook_direction)

    if viz_obj:
        obj_model = skrobot.models.urdf.RobotModelFromURDF(
            urdf_file=urdf_file)
        viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
        viewer.add(obj_model)
        viewer.show()

    StartPos = [0, 0, 0]
    StartOrientation = pybullet.getQuaternionFromEuler([0, 0, 0])
    object_id = pybullet.loadURDF(urdf_file,
                                  StartPos, StartOrientation)

    find_count = 0
    is_hanging_object = True

    height_thresh = 0.5

    if render:
        im_w = 512
        im_h = 424
        im_fov = 42.5
        nf = 0.1
        ff = 2.0
        pos = np.array([0.6, 0, 1.0])
        rot = skrobot.coordinates.math.rotation_matrix_from_rpy(
            [np.pi / 2, 0, -np.pi / 2])
        r = Renderer(im_w, im_h, im_fov, nf, ff, pos, rot)

    try:
        for try_count in six.moves.range(try_num):
            bad_flag = False
            if try_count > try_num - 10:
                print('try_count:{}'.format(try_count))
            if find_count >= required_points_num:
                print('break {} find_count:{} try_count:{} require:{}'.format(
                    urdf_file, find_count, try_count, required_points_num))
                break
            if find_count == 0 and try_count > try_num / 2.:
                print('break {} find_count:{} try_count:{} require:{}'.format(
                    urdf_file, find_count, try_count, required_points_num))
                add_list(
                    osp.join(base_save_dir, 'bad_list.txt'), category_name)
                bad_flag = True
                break

            pybullet.setGravity(0, 0, 0)
            failed = False
            if hook_type == 'just_bar':
                reset_pose(object_id, x_offset=0.2,
                           y_offset=0., z_offset=1.)
            else:
                reset_pose(object_id, x_offset=0.2,
                           y_offset=0., z_offset=1.1)
            pybullet.stepSimulation()
            contact_points = pybullet.getContactPoints(object_id, hook_id)
            if contact_points:
                continue

            if hook_type == 'just_bar':
                pybullet.resetBaseVelocity(object_id, [-0.1, 0, 0])
            else:
                pybullet.resetBaseVelocity(object_id, [-0.1, 0, -0.05])

            for _ in range(int(timestep * 0.5)):
                pybullet.stepSimulation()
                if render:
                    r.render()
                    depth = (r.get_depth_metres() * 1000).astype(np.float32)

            pybullet.resetBaseVelocity(object_id, [0, 0, 0])
            pybullet.setGravity(0, 0, gravity)
            for _ in range(int(timestep * 1)):
                pos, rot = pybullet.getBasePositionAndOrientation(object_id)
                if pos[2] < height_thresh:
                    failed = True
                pybullet.stepSimulation()
                if render:
                    r.render()
                    depth = (r.get_depth_metres() * 1000).astype(np.float32)

            if failed:
                continue

            pybullet.setGravity(0, 2, -5)
            for _ in range(int(timestep * 1)):
                pos, rot = pybullet.getBasePositionAndOrientation(object_id)
                if pos[2] < height_thresh:
                    failed = True
                pybullet.stepSimulation()
                if render:
                    r.render()
                    depth = (r.get_depth_metres() * 1000).astype(np.float32)
            if failed:
                continue

            pybullet.setGravity(0, -2, -5)
            for _ in range(int(timestep * 1)):
                pos, rot = pybullet.getBasePositionAndOrientation(object_id)
                if pos[2] < height_thresh:
                    failed = True
                pybullet.stepSimulation()
                if render:
                    r.render()
                    depth = (r.get_depth_metres() * 1000).astype(np.float32)
            if failed:
                continue

            pybullet.setGravity(0, 0, gravity)
            for _ in range(int(timestep * 1)):
                pos, rot = pybullet.getBasePositionAndOrientation(object_id)
                if pos[2] < height_thresh:
                    failed = True
                pybullet.stepSimulation()
            if failed:
                continue

            contact_points = pybullet.getContactPoints(object_id, hook_id)
            pos, rot = pybullet.getBasePositionAndOrientation(object_id)

            if len(contact_points) == 0:
                continue

            obj_coords = skrobot.coordinates.Coordinates(
                pos=pos,
                rot=skrobot.coordinates.math.xyzw2wxyz(rot))

            min_height_contact_point = sorted(
                contact_points, key=lambda x: x[5][2])[0][5]

            contact_point = skrobot.coordinates.Coordinates(
                pos=min_height_contact_point,
                rot=skrobot.coordinates.math.rotation_matrix_from_axis(
                    hook_direction, [0, 0, -1]))

            contact_point_obj = obj_coords.inverse_transformation().transform(
                contact_point).translate(center, 'world')

            contact_point_marker = skrobot.models.Sphere(
                0.001, color=[255, 0, 0])
            contact_point_marker.newcoords(
                skrobot.coordinates.Coordinates(
                    pos=contact_point_obj.worldpos(),
                    rot=contact_point_obj.worldrot()))

            if viz_obj:
                viewer.add(contact_point_marker)

            pose = np.concatenate(
                [contact_point_obj.T()[:3, 3][None, :],
                 contact_point_obj.T()[:3, :3]]).tolist()
            contact_points_list.append(pose)

            contact_points_dict['contact_points'] = contact_points_list

            save_contact_points(
                osp.join(save_dir, 'contact_points.json'), contact_points_dict)

            find_count += 1
            print('{}({}) find:{}'.format(urdf_file, pid, find_count))

        print('finish {}'.format(urdf_file))
        if not bad_flag:
            add_list(osp.join(base_save_dir, 'good_list.txt'), category_name)
    except KeyboardInterrupt:
        sys.exit()

    pybullet.disconnect()
    return contact_points_list


def reset_pose(object_id, x_offset=0.2, y_offset=0., z_offset=1.):
    x = (np.random.rand() - 0.5) * 0.1 + x_offset
    y = (np.random.rand() - 0.5) * 0.4 + y_offset
    z = (np.random.rand() - 0.5) * 0.4 + z_offset

    roll = np.random.rand() * pi * 2
    pitch = np.random.rand() * pi * 2
    yaw = np.random.rand() * pi * 2
    pybullet.setGravity(0, 0, 0)
    pybullet.resetBasePositionAndOrientation(
        object_id,
        [x, y, z],
        pybullet.getQuaternionFromEuler([roll, pitch, yaw]))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--urdf', '-u', type=str,
                        help='input urdf',
                        default='../urdf/610/scissors/base.urdf')
    parser.add_argument('--required_points_num', '-n', type=int,
                        help='required points number',
                        default=1)
    parser.add_argument('--try-num', '-tn', type=int,
                        help='number of try', default=1000)
    parser.add_argument('--gui', '-g', type=int,
                        help='gui',
                        default=0)
    parser.add_argument('--viz-obj', '-v', type=int,
                        help='viz obj with contactpoints',
                        default=0)
    parser.add_argument('--hook-type', '-ht', type=str,
                        help='hook type "just_bar" or hook urdf',
                        default='just_bar')
    args = parser.parse_args()

    contact_points_list = generate(args.urdf,
                                   args.required_points_num,
                                   args.try_num,
                                   args.gui,
                                   args.viz_obj,
                                   os.path.dirname(args.urdf),
                                   hook_type=args.hook_type)
