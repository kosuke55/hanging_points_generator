#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import json
import os
import os.path as osp
import sys
from math import pi

import numpy as np
import pybullet
import pybullet_data
import skrobot
import six
import trimesh
import xml.etree.ElementTree as ET
from distutils.util import strtobool
from eos import make_fancy_output_dir
from filelock import FileLock
from sklearn.cluster import DBSCAN

from hanging_points_generator.renderer import Renderer
from hanging_points_generator.generator_utils import load_multiple_contact_points
from hanging_points_generator.generator_utils import save_contact_points


def check_contact_points(
        contact_points_path, urdf_file, json_name='contact_points.json',
        use_clustering=True, use_filter_penetration=True,
        inf_penetration_check=True):
    """Chaeck contact poitns with urdf

    Parameters
    ----------
    contact_points_path : str
        file or dir path
        if dir load multiple contact_points
    urdf_file : str
    json_name : str, optional
        'contact_points.json' or 'pouring_points.json',
        by default 'contact_points.json'
    use_clustering : bool, optional
        by default True
    use_filter_penetration : bool, optional
        by default True
    inf_penetration_check : bool, optional
        by default True

    """
    if osp.isdir(contact_points_path):
        contact_points_dict = load_multiple_contact_points(
            contact_points_path, json_name)
    else:
        contact_points_dict = json.load(open(contact_points_path, 'r'))

    contact_points = contact_points_dict['contact_points']
    if use_clustering:
        contact_points = cluster_hanging_points(
            contact_points, eps=0.005, min_samples=2)
    if use_filter_penetration:
        if inf_penetration_check:
            contact_points, _ = filter_penetration(
                urdf_file, contact_points, box_size=[100, 0.0001, 0.0001])
        else:
            contact_points, _ = filter_penetration(
                urdf_file, contact_points, box_size=[0.1, 0.0001, 0.0001])

    obj_model = skrobot.models.urdf.RobotModelFromURDF(
        urdf_file=osp.abspath(urdf_file))

    viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
    viewer.add(obj_model)

    for i, cp in enumerate(contact_points):
        contact_point_sphere = skrobot.models.Sphere(0.001, color=[255, 0, 0])
        contact_point_sphere.newcoords(
            skrobot.coordinates.Coordinates(pos=cp[0],
                                            rot=cp[1:]))
        viewer.add(contact_point_sphere)

    viewer._init_and_start_app()


def cluster_hanging_points(hanging_points, eps=0.03,
                           min_samples=1, merge_clusters=True):
    points = [c[0] for c in hanging_points]
    dbscan = DBSCAN(
        eps=eps, min_samples=min_samples).fit(points)
    clustered_hanging_points = []

    for label in range(np.max(dbscan.labels_) + 1):
        if np.count_nonzero(dbscan.labels_ == label) <= 1:
            continue
        for idx, hp in enumerate(hanging_points):
            if dbscan.labels_[idx] == label:
                clustered_hanging_points.append(hp)

    return clustered_hanging_points


def filter_penetration(obj_file, hanging_points,
                       box_size=[0.1, 0.0001, 0.0001]):
    """Filter the penetrating hanging points

    Parameters
    ----------
    obj_file : srt
        obj file path (urdf or stl)
    hanging_points : list
        list of hanging points(=contact points)
    box_size : list
        penetration check box of size [length, width, width]

    Returns
    -------
    penetrating_hanging_points: list
    filtered_hanging_points: list
    """

    filtered_hanging_points = []
    penetrating_hanging_points = []

    path_without_ext, ext = osp.splitext(obj_file)
    if ext == '.urdf':
        obj_file = path_without_ext + '.stl'
        if not osp.isfile(obj_file):
            obj_file = path_without_ext + '.obj'
    obj = skrobot.models.MeshLink(obj_file)

    collision_manager = trimesh.collision.CollisionManager()
    collision_manager.add_object('obj', obj.visual_mesh)

    for hp in hanging_points:
        penetration_check_box = skrobot.models.Box(
            box_size,
            face_colors=[255, 0, 0],
            pos=hp[0], rot=hp[1:])
        penetration_check_box.translate([0, 0.005, 0])

        penetration = collision_manager.in_collision_single(
            penetration_check_box.visual_mesh, penetration_check_box.T())

        if penetration:
            penetrating_hanging_points.append(hp)
        else:
            filtered_hanging_points.append(hp)

    return filtered_hanging_points, penetrating_hanging_points


def generate(urdf_file, required_points_num,
             enable_gui, viz_obj, save_dir,
             hook_type='just_bar', render=False):
    current_dir = os.path.dirname(os.path.abspath(__file__))
    save_dir = make_fancy_output_dir(osp.join(save_dir, 'contact_points'))
    pid = os.getpid()

    contact_points_list = []
    contact_points_dict = {'urdf_file': urdf_file, 'contact_points': []}

    tree = ET.parse(urdf_file)
    root = tree.getroot()
    center = np.array([float(i) for i in root[0].find(
        "inertial").find("origin").attrib['xyz'].split(' ')])

    if strtobool(enable_gui):
        pybullet.connect(pybullet.GUI)
        pybullet.resetDebugVisualizerCamera(
            cameraDistance=0.3,
            cameraYaw=90,
            cameraPitch=0,
            cameraTargetPosition=[0.15, 0, 1])
    else:
        pybullet.connect(pybullet.DIRECT)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    gravity = -10
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
            os.path.join(current_dir, '../urdf/hook/plate_visual.urdf'),
            [-0.15, 0, 0.8], [0, 0, 0, 1])

    # hook urdf model
    else:
        hook_id = pybullet.loadURDF(
            os.path.join(current_dir, '../urdf/hook/hook.urdf'),
            [0, 0, 1], [0, 0, 0, 1])
        hook_direction = np.array([1, 0, np.tan(np.pi / 2 - 1.2)])
        pybullet.loadURDF(
            os.path.join(current_dir, '../urdf/hook/plate.urdf'),
            [0, 0, 1], [0, 0, 0, 1])

    hook_direction /= np.linalg.norm(hook_direction)

    if strtobool(viz_obj):
        obj_model = skrobot.models.urdf.RobotModelFromURDF(
            urdf_file=urdf_file)
        viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
        viewer.add(obj_model)
        viewer.show()

    StartPos = [0, 0, 0]
    StartOrientation = pybullet.getQuaternionFromEuler([0, 0, 0])
    object_id = pybullet.loadURDF(urdf_file,
                                  StartPos, StartOrientation)

    try_num = 10000
    find_count = 0
    is_hanging_object = True

    # if os.path.exists(os.path.join(save_dir, 'contact_points.json')):
    #     filelock_path = os.path.join(
    #         save_dir, 'contact_points.json.lock')
    #     with FileLock(filelock_path):
    #         with open(os.path.join(save_dir, 'contact_points.json'), 'r') as f:
    #             contact_points_dict_existed = json.load(f)
    #             find_count = len(
    #                 contact_points_dict_existed['contact_points'])
    #     if find_count == 0:
    #         print(urdf_file, 'Not hanging object')
    #         is_hanging_object = False
    # print("{}: num hanging points: {}".format(urdf_file, find_count))

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
            # if os.path.exists(os.path.join(save_dir, 'contact_points.json')):
            #     filelock_path = os.path.join(
            #         save_dir, 'contact_points.json.lock')
            #     with FileLock(filelock_path):
            #         with open(os.path.join(save_dir, 'contact_points.json'), 'r') as f:
            #             contact_points_dict_existed = json.load(f)
            #             find_count = len(
            #                 contact_points_dict_existed['contact_points'])
            #     if find_count == 0:
            #         print(urdf_file, 'Not hanging object')
            #         is_hanging_object = False

            if try_count > try_num - 10:
                print('try_count:{}'.format(try_count))
            # if np.mod(try_count, 500) == 0:
            #     print("try count:{}".format(try_count))

            # if find_count >= required_points_num or not is_hanging_object:
            #     print('break {} find_count:{} require:{} hanging_object:{}'.format(
            #         urdf_file, find_count, required_points_num, is_hanging_object))
            #     break
            if find_count >= required_points_num or \
                    (find_count == 0 and try_count > 2000):
                print('break {} find_count:{} try_count:{} require:{}'.format(
                    urdf_file, find_count, try_count, required_points_num))
                break

            # if find_count == 0 and try_count > 10000:
            #     print("Not find hanging points")
            #     filelock_path = os.path.join(
            #         save_dir, 'contact_points.json.lock')
            #     with FileLock(filelock_path):
            #         with open(os.path.join(
            #                 save_dir, 'contact_points.json'), 'r') as f:
            #             json.dump(
            #                 contact_points_dict, f, ensure_ascii=False,
            #                 indent=4, sort_keys=True, separators=(',', ': '))

            #     print('break {} find_count{} try_count:{}'.format(
            #         urdf_file, find_count, try_count))
            #     break

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

            pybullet.setGravity(0, 5, -5)
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

            pybullet.setGravity(0, -5, -5)
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
                # if render:
                # r.render()
                # depth = (r.get_depth_metres() * 1000).astype(np.float32)
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

            # contact_point_to_hole_vector = np.array(
            #     [min_height_contact_point[0], 0, 1]) - np.array(
            #         min_height_contact_point)
            contact_point = skrobot.coordinates.Coordinates(
                pos=min_height_contact_point,
                rot=skrobot.coordinates.math.rotation_matrix_from_axis(
                    hook_direction, [0, 0, -1]))

            # y_axis=contact_point_to_hole_vector))

            contact_point_obj = obj_coords.inverse_transformation().transform(
                contact_point).translate(center, 'world')

            contact_point_sphere = skrobot.models.Sphere(
                0.001, color=[255, 0, 0])
            contact_point_sphere.newcoords(
                skrobot.coordinates.Coordinates(
                    pos=contact_point_obj.worldpos(),
                    rot=contact_point_obj.worldrot()))

            if strtobool(viz_obj):
                viewer.add(contact_point_sphere)

            pose = np.concatenate(
                [contact_point_obj.T()[:3, 3][None, :],
                 contact_point_obj.T()[:3, :3]]).tolist()
            contact_points_list.append(pose)

            contact_points_dict['contact_points'] = contact_points_list

            save_contact_points(
                save_dir, 'contact_points.json', contact_points_dict)

            find_count += 1
            print('{}({}) find:{}'.format(urdf_file, pid, find_count))

            # if os.path.exists(os.path.join(save_dir, 'contact_points.json')):
            #     filelock_path = os.path.join(
            #         save_dir, 'contact_points.json.lock')
            #     with FileLock(filelock_path):
            #         with open(os.path.join(
            #                 save_dir, 'contact_points.json'), 'r') as f:
            #             contact_points_dict_existed = json.load(f)
            #             contact_points_dict_existed['contact_points'].append(
            #                 pose)
            #             find_count = len(
            #                 contact_points_dict_existed['contact_points'])

            #     filelock_path = os.path.join(
            #         save_dir, 'contact_points.json.lock')
            #     with FileLock(filelock_path):
            #         with open(os.path.join(save_dir,
            #                                'contact_points.json', ), 'w') as f:
            #             json.dump(
            #                 contact_points_dict_existed, f, ensure_ascii=False,
            #                 indent=4, sort_keys=True, separators=(',', ': '))

            # else:
            #     with open(os.path.join(save_dir,
            #                            'contact_points.json', ), 'w') as f:
            #         json.dump(contact_points_dict, f, ensure_ascii=False,
            #                   indent=4, sort_keys=True, separators=(',', ': '))
            #     find_count += 1
            # print("{}: Find the hanging point {}".format(urdf_file, find_count))

        print('finish {}'.format(urdf_file))
    except KeyboardInterrupt:
        sys.exit()

    pybullet.disconnect()
    return contact_points_list


def reset_pose(object_id, x_offset=0.2, y_offset=0., z_offset=1.):
    x = (np.random.rand() - 0.5) * 0.1 + x_offset
    y = (np.random.rand() - 0.5) * 0.4 + y_offset
    z = (np.random.rand() - 0.5) * 0.4 + z_offset

    roll = np.random.rand() * pi
    pitch = np.random.rand() * pi
    yaw = np.random.rand() * pi
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
                                   os.path.dirname(args.urdf),
                                   hook_type='just_bar')