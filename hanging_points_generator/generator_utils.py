#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import copy
import json
import os
import os.path as osp
import random
from operator import itemgetter
from pathlib import Path

import numpy as np
import numpy.matlib as npm
import pybullet
import skrobot
import trimesh
import xml.etree.ElementTree as ET
from filelock import FileLock
from skrobot import coordinates
from sklearn.cluster import DBSCAN


def check_contact_points(
        contact_points_path, urdf_file='', json_name='contact_points.json',
        cluster_min_points=2, eps=0.03, use_filter_penetration=True,
        inf_penetration_check=True, align=True, average=True,
        average_pos=False, _test=False, image_name=None, large_axis=False,
        just_check_num_points=False):
    """Chaeck contact points with urdf

    Parameters
    ----------
    contact_points_path : str
        file or dir path
        if dir load multiple contact_points
    urdf_file : str
    json_name : str, optional
        'contact_points.json' or 'pouring_points.json',
        by default 'contact_points.json'
    cluster_min_points : int, optional
        by default 2
    eps : float, optional
        eps paramerter of sklearn dbscan, by default 0.03
    use_filter_penetration : bool, optional
        by default True
    inf_penetration_check : bool, optional
        by default True
    align : bool, optional
        by default True
    average : bool, optional
        by default True
    average_pos : bool, optional
        by default False
    average_pos : bool, optional
        by default False
    _test : bool, optional
        test mode, by default False
    image_name, optional
        if set this, save image with the name, by default None
    large_axis, by default False
        visualize points with large axis, by default False
    just_check_num_points : bool, optional
        by default False

    """
    if osp.isdir(contact_points_path):
        contact_points_dict = load_multiple_contact_points(
            contact_points_path, json_name)
    else:
        contact_points_dict = load_json(contact_points_path)
    if not contact_points_dict:
        print('No points file')
        return False

    contact_points = contact_points_dict['contact_points']
    print('Load %d points' % len(contact_points))
    if just_check_num_points:
        return True
    urdf_file = str(urdf_file or contact_points_dict['urdf_file'])

    if use_filter_penetration or inf_penetration_check:
        if inf_penetration_check:
            contact_points, _ = filter_penetration(
                urdf_file, contact_points,
                box_size=[100, 0.0001, 0.0001])
        else:
            contact_points, _ = filter_penetration(
                urdf_file, contact_points,
                box_size=[0.1, 0.0001, 0.0001])
        print('%d points are left after penetration check' % len(
            contact_points))

    if cluster_min_points or cluster_min_points == -1:
        contact_points = cluster_contact_points(
            contact_points, min_samples=cluster_min_points, eps=eps)
        print('%d points are left after clustering' % len(contact_points))

    obj_model = skrobot.models.urdf.RobotModelFromURDF(
        urdf_file=osp.abspath(urdf_file))

    if len(contact_points) == 0:
        print('No points')
        return False

    if align or average:
        contact_points_coords = make_contact_points_coords(contact_points)
        dbscan = dbscan_coords(contact_points_coords, eps=eps)
        contact_points_coords, labels = get_dbscan_core_coords(
            contact_points_coords, dbscan)
    if align and labels:
        contact_points_coords \
            = align_coords(contact_points_coords, labels)
        contact_points = coords_to_dict(contact_points_coords,
                                        urdf_file)['contact_points']
    if average and labels:
        contact_points_coords = make_average_coords_list(
            contact_points_coords, labels, average_pos=average_pos)
        contact_points \
            = coords_to_dict(contact_points_coords,
                             urdf_file)['contact_points']

    if not _test:
        contact_point_marker_list = []
        for i, cp in enumerate(contact_points):
            if large_axis:
                contact_point_marker = skrobot.model.Axis(0.003, 0.05)
            else:
                contact_point_marker = skrobot.model.Sphere(
                    0.001, color=[255, 0, 0])
            contact_point_marker.newcoords(
                skrobot.coordinates.Coordinates(pos=cp[0], rot=cp[1:]))
            contact_point_marker_list.append(contact_point_marker)

        viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 640))
        viewer.add(obj_model)
        for contact_point_marker in contact_point_marker_list:
            viewer.add(contact_point_marker)

        viewer._init_and_start_app()

        if image_name is not None:
            print('Save %s' % image_name)
            from PIL import Image
            loop = True
            while loop:
                try:
                    data = viewer.scene.save_image(visible=True)
                    rendered = Image.open(trimesh.util.wrap_as_stream(data))
                    rendered.save(image_name)
                    loop = False
                except AttributeError:
                    print('Fail to save. Try again.')

    return True


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


def rotate_object(object_id, rot):
    """Rotate to the specified roatation
    Parameters
    ----------
    object_id : str
    rot : list[float float float float]
        quarternion
    """
    pos, _ = pybullet.getBasePositionAndOrientation(object_id)
    pybullet.resetBasePositionAndOrientation(object_id, pos, rot)


def rotate_local(rot, theta, axis):
    rot = coordinates.math.xyzw2wxyz(rot)
    c = coordinates.Coordinates(rot=rot)
    c.rotate(theta, axis, 'local')
    rot = coordinates.math.wxyz2xyzw(c.quaternion)
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


def save_json(save_file, data, sort_key=True):
    """Save json

    Parameters
    ----------
    save_file : str
    data : list or dict
    sort_key : bool, optional
        by default True
    """
    with open(save_file, 'w') as f:
        json.dump(data, f, ensure_ascii=False,
                  indent=4, sort_keys=sort_key, separators=(',', ': '))


def load_json(file):
    """Load json file

    Parameters
    ----------
    file : str
        input json file

    Returns
    -------
    data : dict
    """
    with open(file, 'r') as f:
        data = json.load(f)
    return data


def save_contact_points(
        save_file, contact_points_dict, filelock=False):
    """Save contact points json file with filelock

    Parameters
    ----------
    save_file : str
    contact_points_dict : dict

    """
    if filelock:
        if os.path.exists(save_file):
            filelock_path = save_file + '.lock'
            with FileLock(filelock_path):
                contact_points_dict_existed = load_json(save_file)
                for c in contact_points_dict['contact_points']:
                    contact_points_dict_existed['contact_points'].append(c)

            filelock_path = save_file + '.lock'
            with FileLock(filelock_path):
                save_json(save_file, contact_points_dict_existed)
        else:
            save_json(save_file, contact_points_dict)
    else:
        save_json(save_file, contact_points_dict)


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
        object_id, object_center, contact_object_id, obj_coords=None,
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

    rot = coordinates.math.rotation_matrix_from_axis(
        x_axis, y_axis)

    contact_point = coordinates.Coordinates(
        pos=contact_point, rot=rot)

    if obj_coords is None:
        object_pos, object_rot = pybullet.getBasePositionAndOrientation(
            object_id)
        obj_coords = coordinates.Coordinates(
            pos=object_pos,
            rot=coordinates.math.xyzw2wxyz(object_rot))

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
        base_dir/**/json_file
    json_name : str, optional
        'contact_points.json' or 'pouring_points.json',
        by default 'contact_points.json'

    Returns
    -------
    base_cp_dict
        merged contact_points dict. If not file retunr False

    """
    base_cp_dict = False
    paths = list(sorted(Path(base_dir).glob(osp.join('**', json_name))))
    for i, path in enumerate(paths):
        if i == 0:
            base_cp_dict = load_json(str(path))
        else:
            cp_dict = load_json(str(path))
            for cp in cp_dict['contact_points']:
                base_cp_dict['contact_points'].append(cp)

    return base_cp_dict


def make_contact_points_coords(contact_points):
    """Make contact points coords

    Parameters
    ----------
    contact_points : list[list[list[float], list[float]]]

    Returns
    -------
    contact_points_coords:  list[skrobot.coordinates.Coordinates]
    """
    contact_points_coords = []
    for cp in contact_points:
        contact_point_coords = coordinates.Coordinates(
            pos=cp[0], rot=cp[1:])
        contact_points_coords.append(contact_point_coords)
    return contact_points_coords


def dbscan_coords(coords_list, eps=0.03, min_sample=2):
    """Dbscan cluster coords list

    Parameters
    ----------
    coords_list : list[skrobot.coordinates.Coordinates]
    eps : float, optional
        [description], by default 0.03
    min_sample : int, optional
        cluster min sample, by default 2

    Returns
    -------
    dbscan :

    """
    dbscan = DBSCAN(eps=eps, min_samples=min_sample).fit(
        [coords.worldpos() for coords in coords_list])
    return dbscan


def cluster_contact_points(points, eps=0.01, min_samples=-1):
    """Clustering points

    Parameters
    ----------
    points : list[list[list[float], list[float]]]

    eps : float, optional
        eps paramerter of sklearn dbscan, by default 0.01
    min_samples : int, optional
        clustering min samples, if -1 set 1/5 of the whole, by default -1

    Returns
    -------
    clustered points
        clustered hanging points
    """
    if min_samples == -1:
        min_samples = len(points) // 5

    pos = [c[0] for c in points]
    dbscan = DBSCAN(
        eps=eps, min_samples=min_samples).fit(pos)

    clustered_points = []

    for label in range(np.max(dbscan.labels_) + 1):
        if np.count_nonzero(dbscan.labels_ == label) <= 1:
            continue
        for idx, p in enumerate(points):
            if dbscan.labels_[idx] == label:
                clustered_points.append(p)

    return clustered_points


def get_dbscan_core_coords(coords_list, dbscan):
    idx = tuple(dbscan.core_sample_indices_)
    if len(idx) == 0:
        print('No core sample')
        return coords_list, False
    core_labels = [int(x) for x in list(
        filter(lambda x: x != -1, dbscan.labels_))]
    core_coords_list = itemgetter(*idx)(coords_list)
    return core_coords_list, core_labels


def get_dbscan_label_coords(coords_list, dbscan, label):
    idx = dbscan.labels_ == label
    label_coords_list = np.array(coords_list)[(np.array(idx))].tolist()
    return label_coords_list


def align_coords(coords_list, labels, angle_thresh=90., copy_list=True):
    """Align the x-axis of coords

    invert coordinates above the threshold.
    If you do not align, the average value of the
    rotation map will be incorrect.

    Parameters
    ----------
    coords_list : list[skrobot.coordinates.base.Coordinates]
    lables : numpy.ndarray
    angle_thresh : float, optional
        invert coordinates above the threshold, by default 135.0
    copy_list ; bool, optional
        If True copy coords_list, by default True

    Returns
    -------
    coords_list : list[skrobot.coordinates.base.Coordinates]
    """
    aligned_coords_list = []
    if copy_list:
        coords_list = copy.copy(coords_list)

    for label in range(np.max(labels) + 1):
        q_base = None
        for idx, coords in enumerate(coords_list):
            if labels[idx] == label:
                if q_base is None:
                    q_base = coords.quaternion
                q_distance \
                    = coordinates.math.quaternion_distance(
                        q_base, coords.quaternion)

                if np.rad2deg(q_distance) > angle_thresh:
                    # coords_list[idx].rotate(np.pi, 'y')
                    aligned_coords_list.append(
                        coords_list[idx].copy_worldcoords().rotate(
                            np.pi, 'y'))
                else:
                    aligned_coords_list.append(
                        coords_list[idx].copy_worldcoords())

    return aligned_coords_list


def split_label_coords(coords_list, labels):
    """Split coords based on label

    Parameters
    ----------
    coords_list : list[skrobot.coordinates.Coordinates]

    Returns
    -------
    coords_clusters :list[list[skrobot.coordinates.Coordinates]]
    """
    coords_clusters = []
    labels = np.array(labels)
    for label in range(np.max(labels) + 1):
        idx = tuple(np.where(labels == label)[0])
        coords_cluster = itemgetter(*idx)(coords_list)

        coords_clusters.append(coords_cluster)
    return coords_clusters


def make_average_coords_list(coords_list, labels, average_pos=True):
    """Make average orientation coords list

    Parameters
    ----------
    coords_list : list[skrobot.coordinates.Coordinates]

    Returns
    -------
    coords_list : list[skrobot.coordinates.Coordinates]
    """
    average_coords_list = []
    coords_clusters = split_label_coords(coords_list, labels)
    for coords_cluster in coords_clusters:
        coords_average = average_coords(coords_cluster)
        if average_pos:
            average_coords_list.append(coords_average)
        else:
            for coords in coords_cluster:
                coords.rotation = coords_average.rotation
                average_coords_list.append(coords)

    return average_coords_list


def averageQuaternions(Q):
    """Calculate average quaternion

    https://github.com/christophhagen/averaging-quaternions/blob/master/LICENSE
    Q is a Nx4 numpy matrix and contains the quaternions to average in the rows.
    The quaternions are arranged as (w,x,y,z), with w being the scalar
    The result will be the average quaternion of the input. Note that the signs
    of the output quaternion can be reversed, since q and -q describe the same orientation

    Parameters
    ----------
    Q : numpy.ndarray or list[float]

    Returns
    -------
    average quaternion : numpy.ndarray
    """

    Q = np.array(Q)
    M = Q.shape[0]
    A = npm.zeros(shape=(4, 4))

    for i in range(0, M):
        q = Q[i, :]
        A = np.outer(q, q) + A

    A = (1.0 / M) * A
    eigenValues, eigenVectors = np.linalg.eig(A)
    eigenVectors = eigenVectors[:, eigenValues.argsort()[::-1]]

    # return the real part of the largest eigenvector (has only real part)
    return np.real(eigenVectors[:, 0].A1)


def average_coords(coords_list):
    """Calculate average coords

    Parameters
    ----------
    coords_list : list[skrobot.coordinates.Coordinates]

    Returns
    -------
    coords_average : skrobot.coordinates.Coordinates
    """
    q_list = [c.quaternion for c in coords_list]
    q_average = averageQuaternions(q_list)
    pos_average = np.mean([c.worldpos() for c in coords_list], axis=0)
    coords_average = coordinates.Coordinates(pos_average, q_average)
    return coords_average


def coords_to_dict(coords_list, urdf_file, labels=None):
    """Cover coords list to dict for json

    Parameters
    ----------
    coords_list : list[skrobot.coordinates.Coordinates]
    labels : list[int]

    Returns
    -------
    contact_points_dict : dict
    """
    contact_points_list = []
    contact_points_dict = {
        'urdf_file': urdf_file,
        'contact_points': [],
        'labels': []}
    for coords in coords_list:
        pose = np.concatenate(
            [coords.T()[:3, 3][None, :],
                coords.T()[:3, :3]]).tolist()
        contact_points_list.append(pose)
    contact_points_dict['contact_points'] = contact_points_list
    if labels is not None:
        contact_points_dict['labels'] = labels
    return contact_points_dict


def make_aligned_contact_points_coords(contact_points_coords, eps=0.01):
    """Aligned contact points coords

    Parameters
    ----------
    contact_points_coords : list[skrobot.coordinates.Coordinates]
        [description]
    eps : float, optional
        eps paramerter of sklearn dbscan, by default 0.01

    Returns
    -------
    [type]
        [description]
    """
    dbscan = dbscan_coords(contact_points_coords, eps=eps)
    contact_points_coords, labels = get_dbscan_core_coords(
        contact_points_coords, dbscan)
    if not labels:
        return False
    aligned_contact_points_coords \
        = align_coords(contact_points_coords, labels)
    return aligned_contact_points_coords


def make_aligned_contact_points(contact_points_dict):
    contact_points = contact_points_dict['contact_points']
    urdf_file = str(contact_points_dict['urdf_file'])
    contact_points_coords = make_contact_points_coords(contact_points)
    aligned_contact_points_coords \
        = make_aligned_contact_points_coords(contact_points_coords)
    if not aligned_contact_points_coords:
        return False
    aligned_contact_points_dict = coords_to_dict(
        aligned_contact_points_coords, urdf_file)
    return aligned_contact_points_dict


def filter_penetration(obj_file, hanging_points,
                       box_size=[0.1, 0.0001, 0.0001],
                       translate=[0, 0.005, 0]):
    """Filter the penetrating hanging points

    Parameters
    ----------
    obj_file : srt
        obj file path (urdf, stl or obj)
    hanging_points : list[list[list[float], list[float]]]
        list of hanging points(=contact points)
    box_size : list[float]
        penetration check box of size [length, width, width] order
    translate : list, optional
        translate a box for penetration check,
        by default [0, 0.005, 0]

    Returns
    -------
    penetrating_hanging_points: list[list[list[float], list[float]]]
    filtered_hanging_points: list[list[list[float], list[float]]]
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
        penetration_check_box.translate(translate)

        penetration = collision_manager.in_collision_single(
            penetration_check_box.visual_mesh, penetration_check_box.T())

        if penetration:
            penetrating_hanging_points.append(hp)
        else:
            filtered_hanging_points.append(hp)

    return filtered_hanging_points, penetrating_hanging_points


def sample_contact_points(contact_points, num_samples):
    """Sampling contact points for the specified number

    Parameters
    ----------
    contact_points : list[list[list[float], list[float]]]
    num_samples : int

    Returns
    -------
    contact_points : list[list[list[float], list[float]]]
    """
    if num_samples > len(contact_points):
        num_samples = len(contact_points)
    idx = random.sample(range(0, len(contact_points)), num_samples)
    return [contact_points[i] for i in idx]


def set_contact_points_urdf_path(contact_points_path):
    """Set contact points urdf path

    Set base.urdf in same directory

    Parameters
    ----------
    contact_points_path : str
    """
    urdf_file = osp.join(osp.dirname(contact_points_path), 'base.urdf')
    contact_points_dict = load_json(contact_points_path)
    contact_points_dict['urdf_file'] = urdf_file
    save_contact_points(contact_points_path, contact_points_dict)


def filter_contact_points(
        contact_points_dict, cluster_min_points=-1, eps=0.03, num_samples=30,
        use_filter_penetration=True, inf_penetration_check=True,
        translate=[0, 0.005, 0]):
    """Filter contact points by clustering, aligning, averageing

    Parameters
    ----------
    contact_points_dict :
        dict{'contact_points' : list[list[list[float], list[float]]]
             'urdf_file' : str}
    cluster_min_points : int, optional
        by default -1
    eps : float, optional
        eps paramerter of sklearn dbscan, by default 0.03
    num_samples : int, optional
        sampling contact points with this value.
        if -1 remain all points.
    use_filter_penetration : bool, optional
        by default True
    inf_penetration_check : bool, optional
        by default True
    translate : list, optional
        translate a box for penetration check,
        by default [0, 0.005, 0]

    Returns
    -------
    average_aligned_contact_points_coord_dict :
        dict{'contact_points' : list[list[list[float], list[float]]]
             'urdf_file' : str}
    """
    urdf_file = str(contact_points_dict['urdf_file'])
    contact_points = contact_points_dict['contact_points']
    if len(contact_points) == 0:
        print('No points')
        return False

    if use_filter_penetration or inf_penetration_check:
        if inf_penetration_check:
            contact_points, _ = filter_penetration(
                urdf_file, contact_points,
                box_size=[100, 0.0001, 0.0001],
                translate=translate)
        else:
            contact_points, _ = filter_penetration(
                urdf_file, contact_points,
                box_size=[0.1, 0.0001, 0.0001],
                translate=translate)
        print('penetration contact_points :%d' % len(contact_points))
        if len(contact_points) == 0:
            print('No points after penetration check')
            return False
    if cluster_min_points or cluster_min_points == -1:
        contact_points = cluster_contact_points(
            contact_points, min_samples=cluster_min_points, eps=eps)
        print('clusterring contact_points :%d' % len(contact_points))
        if len(contact_points) == 0:
            print('No points after clustering')
            return False

    contact_points_coords = make_contact_points_coords(contact_points)
    dbscan = dbscan_coords(contact_points_coords, eps=eps)
    contact_points_coords, labels = get_dbscan_core_coords(
        contact_points_coords, dbscan)
    if not labels:
        return False

    aligned_contact_points_coords \
        = align_coords(contact_points_coords, labels)
    average_aligned_contact_points_coords = make_average_coords_list(
        aligned_contact_points_coords, labels, average_pos=False)
    if num_samples > 0:
        average_aligned_contact_points_coords = sample_contact_points(
            average_aligned_contact_points_coords, num_samples=num_samples)

    average_aligned_contact_points_coord_dict \
        = coords_to_dict(
            average_aligned_contact_points_coords, urdf_file, labels)

    return average_aligned_contact_points_coord_dict


def filter_contact_points_dir(
        input_dir, cluster_min_points=-1, eps=0.03,
        rate_thresh=0.1, num_samples=30,
        use_filter_penetration=True, inf_penetration_check=True,
        points_path_name='contact_points', suffix=''):
    """Filter all contact points in the directory

    Parameters
    ----------
    input_dir : str
        hanging_object of hanging_object/category/contact_points/<fancy_dir>/contact_points.json # noqa
    cluster_min_points : int, optional
        by default -1
    eps : float, optional
        eps paramerter of sklearn dbscan, by default 0.03
    rate_thresh : float
        Objects whose rate of the number of remaining points is greater than this value are skipped.
    num_samples : int, optional
        sampling contact points with this value
        if -1 remain all points.
    use_filter_penetration : bool, optional
        by default True
    inf_penetration_check : bool, optional
        by default True
    suffix : str, optional
        if '_pouring' output file name is filtered_contact_points_pouring.json etc.
        by default ''
    """
    if points_path_name == 'p' or points_path_name == 'pouring':
        points_path_name = 'pouring_points'
        json_name = 'pouring_points.json'

    contact_points_path_list = list(
        Path(input_dir).glob('*/{}'.format(points_path_name)))
    skip_list_file = osp.join(input_dir, 'filter_skip_list{}.txt'.format(suffix))
    remain_list_file = osp.join(input_dir, 'filter_remain_list{}.txt'.format(suffix))
    result_dict = {}
    for contact_points_path in contact_points_path_list:
        print('-----')
        contact_points = load_multiple_contact_points(
            str(contact_points_path), json_name=json_name)
        category_name = contact_points_path.parent.name
        print(category_name)
        if not contact_points:
            print('Load no points. Skip %s' % str(contact_points_path))
            add_list(skip_list_file, category_name)
            continue
        pre_points_num = len(contact_points['contact_points'])
        print('contact points :%d' % pre_points_num)
        filtered_contact_points \
            = filter_contact_points(
                contact_points,
                cluster_min_points=cluster_min_points,
                num_samples=num_samples,
                use_filter_penetration=use_filter_penetration,
                inf_penetration_check=inf_penetration_check)
        if not filtered_contact_points:
            print('Skip %s ' % str(contact_points_path))
            add_list(skip_list_file, category_name)
            result_dict[category_name] = {'pre_points_num': pre_points_num,
                                          'post_points_num': 0,
                                          'rate': 0.}
            continue
        post_points_num = len(filtered_contact_points['contact_points'])
        rate = post_points_num / pre_points_num
        print('Filtering %d -> %d  remaning-rate : %f' % (
            pre_points_num, post_points_num, rate))
        if rate < rate_thresh:
            print('Skip %s because of low remaning rate %f' % (
                str(contact_points_path), rate))
            add_list(skip_list_file, category_name)
            continue
        add_list(remain_list_file, category_name)
        save_contact_points(
            str(contact_points_path.parent / 'filtered_contact_points{}.json'.format(suffix)),
            filtered_contact_points)
        result_dict[category_name] = {'pre_points_num': pre_points_num,
                                      'post_points_num': post_points_num,
                                      'rate': rate}
    save_json(
        osp.join(input_dir,
                 'filtering_result{}.json'.format(suffix)), result_dict)


def add_list(path, item):
    """Add object to list and save file

    Parameters
    ----------
    path : str
        ist txt file
    item : str
        item to add
    """
    filelock_path = path + '.lock'
    with FileLock(filelock_path):
        if osp.isfile(path):
            with open(path) as f:
                bad_list = [s.strip() for s in f.readlines()]

            if item not in bad_list:
                with open(path, mode='a') as f:
                    f.writelines(item + '\n')
        else:
            with open(path, mode='a') as f:
                f.writelines(item + '\n')
    os.remove(filelock_path)


def load_list(path):
    """Load list txt

    Parameters
    ----------
    path : str
        list txt file
    """
    filelock_path = path + '.lock'
    list_ = []
    with FileLock(filelock_path):
        if osp.isfile(path):
            with open(path) as f:
                list_ = [s.strip() for s in f.readlines()]
    os.remove(filelock_path)

    return list_
