#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import json
import os
import time
from math import pi

import numpy as np
import pybullet
import pybullet_data
import skrobot
import six
import xml.etree.ElementTree as ET
from distutils.util import strtobool
from sklearn.cluster import DBSCAN


def check_contact_points(contact_points_file, urdf_file, clustering=True):
    contact_points_dict = json.load(open(contact_points_file, 'r'))
    contact_points = contact_points_dict['contact_points']
    if clustering:
        contact_points = cluster_hanging_points(
            contact_points, eps=0.005, min_samples=2)

    obj_model = skrobot.models.urdf.RobotModelFromURDF(
        urdf_file=urdf_file)

    viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
    viewer.add(obj_model)
    viewer.show()

    for i, cp in enumerate(contact_points):
        contact_point_sphere = skrobot.models.Sphere(0.001, color=[255, 0, 0])
        contact_point_sphere.newcoords(
            skrobot.coordinates.Coordinates(pos=cp[0],
                                            rot=cp[1:]))
        viewer.add(contact_point_sphere)


def cluster_hanging_points(hanging_points, eps=0.03, min_samples=1):
    points = [c[0] for c in hanging_points]
    dbscan = DBSCAN(
        eps=eps, min_samples=min_samples).fit(points)
    clustered_hanging_points = []
    print('--- dbscan ---')
    print(points)
    print(dbscan.labels_)
    for label in range(np.max(dbscan.labels_) + 1):
        if np.count_nonzero(dbscan.labels_ == label) <= 1:
            continue
        for idx, hp in enumerate(hanging_points):
            if dbscan.labels_[idx] == label:
                clustered_hanging_points.append(hp)

    return clustered_hanging_points


def generate(urdf_file, required_points_num,
             enable_gui, viz_obj, save_dir, hook_type='just_bar'):
    start_time = time.time()
    finding_times = []
    finding_times.append(start_time)
    current_dir = os.path.dirname(os.path.abspath(__file__))

    contact_points_list = []
    contact_points_dict = {'urdf_file': urdf_file, 'contact_points': []}

    tree = ET.parse(urdf_file)
    root = tree.getroot()
    center = np.array([float(i) for i in root[0].find(
        "inertial").find("origin").attrib['xyz'].split(' ')])

    if strtobool(enable_gui):
        pybullet.connect(pybullet.GUI)
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

    height_thresh = 0.5

    try:
        for try_count in six.moves.range(try_num):
            if np.mod(try_count, 50) == 0:
                print("try count:{}".format(try_count))
            if find_count == 0 and try_count > 5000:
                print("Not find hanging points")
                with open(os.path.join(save_dir,
                                       'contact_points.json', ), 'w') as f:
                    json.dump(contact_points_dict, f, ensure_ascii=False,
                              indent=4, sort_keys=True, separators=(',', ': '))
                return contact_points_list

            # emerge object
            pybullet.setGravity(0, 0, 0)
            if hook_type == 'just_bar':
                reset_pose(object_id, x_offset=0.2, y_offset=0., z_offset=1.)
            else:
                reset_pose(object_id, x_offset=0.2, y_offset=0., z_offset=1.1)
            pybullet.stepSimulation()
            contact_points = pybullet.getContactPoints(object_id, hook_id)
            if contact_points:
                continue

            if hook_type == 'just_bar':
                pybullet.resetBaseVelocity(object_id, [-0.05, 0, 0])
            else:
                pybullet.resetBaseVelocity(object_id, [-0.1, 0, -0.05])

            for _ in range(int(timestep * 2)):
                pybullet.stepSimulation()

            pybullet.resetBaseVelocity(object_id, [0, 0.05, 0])
            for _ in range(int(timestep * 2)):

                pos, rot = pybullet.getBasePositionAndOrientation(object_id)
                if pos[2] < height_thresh:
                    failed = True
                    break
                pybullet.stepSimulation()

            pybullet.resetBaseVelocity(object_id, [0, -0.05, 0])
            for _ in range(int(timestep * 2)):
                pos, rot = pybullet.getBasePositionAndOrientation(object_id)
                if pos[2] < height_thresh:
                    failed = True
                    break

            pybullet.setGravity(0, 0, gravity)
            failed = False
            for _ in range(int(timestep * 2)):
                pos, rot = pybullet.getBasePositionAndOrientation(object_id)
                if pos[2] < height_thresh:
                    failed = True
                    break
                pybullet.stepSimulation()

            if failed:
                continue

            contact_points = pybullet.getContactPoints(object_id, hook_id)
            pos, rot = pybullet.getBasePositionAndOrientation(object_id)

            if len(contact_points) == 0:
                continue

            finding_times.append(time.time())
            print("Find the hanging point {}   time {}  total time {}".format(
                find_count,
                finding_times[len(finding_times) - 1]
                - finding_times[len(finding_times) - 2],
                finding_times[len(finding_times) - 1] - start_time))

            obj_coords = skrobot.coordinates.Coordinates(
                pos=pos,
                rot=skrobot.coordinates.math.xyzw2wxyz(rot))

            min_height_contact_point = sorted(
                contact_points, key=lambda x: x[5][2])[0][5]

            contact_point_to_hole_vector = np.array(
                [min_height_contact_point[0], 0, 1]) - np.array(
                    min_height_contact_point)
            contact_point = skrobot.coordinates.Coordinates(
                pos=min_height_contact_point,
                rot=skrobot.coordinates.math.rotation_matrix_from_axis(
                    x_axis=hook_direction,
                    y_axis=contact_point_to_hole_vector))
            contact_point_obj = obj_coords.inverse_transformation().transform(
                contact_point).translate(center, 'world')

            contact_point_sphere = skrobot.models.Sphere(0.001, color=[255, 0, 0])
            contact_point_sphere.newcoords(
                skrobot.coordinates.Coordinates(
                    pos=contact_point_obj.worldpos(),
                    rot=contact_point_obj.worldrot()))

            if strtobool(viz_obj):
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


def reset_pose(object_id, x_offset=0.2, y_offset=0., z_offset=1.):
    x = (np.random.rand() - 0.5) * 0.1 + x_offset
    y = (np.random.rand() - 0.5) * 0.2 + y_offset
    z = (np.random.rand() - 0.5) * 0.2 + z_offset

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
