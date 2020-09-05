#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import open3d as o3d
import os
import pathlib2
import skrobot
import trimesh
import xml.etree.ElementTree as ET


def create_mesh_tsdf(input_dir, scenes,
                     voxel_length=0.002, sdf_trunc=0.005):
    """Create mesh using tsdf.

    TODO: Making input image list is better.

    Parameters
    ----------
    input_dir : str
      Input directry which include color{:03}.png and depth{:03}.png.
    output_dir : str
      Ouput directry where output mesh saved
    scences : int
      How many scenes were shot.

    Returns
    -------
    mesh : open3d.open3d.geometry.TriangleMesh
      Mesh created by tsdf
    """

    intrinsic_np = np.loadtxt(os.path.join(input_dir,
                                           "camera_pose/intrinsic.txt"))
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    image_shape = np.array(o3d.io.read_image(
        os.path.join(input_dir, "color000.png"))).shape
    width = image_shape[1]
    height = image_shape[0]
    intrinsic.set_intrinsics(
        width,
        height,
        intrinsic_np[0, 0],
        intrinsic_np[1, 1],
        intrinsic_np[0, 2],
        intrinsic_np[1, 2])

    volume = o3d.integration.ScalableTSDFVolume(
        voxel_length=voxel_length,
        sdf_trunc=sdf_trunc,
        color_type=o3d.integration.TSDFVolumeColorType.RGB8,
        volume_unit_resolution=16,
        depth_sampling_stride=1)

    for i in range(scenes):
        print("Integrate {:d}-th image into the volume.".format(i))

        camera_pose = np.loadtxt(
            os.path.join(input_dir,
                         "camera_pose/camera_pose_icp{:03}.txt".format(i)))
        color = o3d.io.read_image(
            os.path.join(input_dir, "color{:03}.png".format(i)))
        depth = o3d.io.read_image(
            os.path.join(input_dir, "depth{:03}.png".format(i)))

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, depth_trunc=4.0, convert_rgb_to_intensity=False)
        # pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        #     rgbd, intrinsic)
        # o3d.visualization.draw_geometries([pcd])

        volume.integrate(
            rgbd,
            intrinsic,
            np.linalg.inv(camera_pose))

    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()

    return mesh


def create_mesh_voxelize(pcd, voxel_size=0.002):
    """Create voxelized mesh from pcd

    Parameters
    ----------
    pcd : open3d.open3d.geometry.PointCloud
      Input pcd data
    voxel_size : float

    Returns
    -------
    mesh : trimesh.base.Trimesh
      Voxelized mesh
    """

    voxel_grid \
        = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)
    mesh = trimesh.Trimesh()
    vertices = []
    faces = []

    for voxel_index, voxel in enumerate(voxel_grid.get_voxels()):
        grid_index = voxel.grid_index

        voxel_origin \
            = grid_index * voxel_size + voxel_grid.origin

        vertices.append(voxel_origin)
        vertices.append(voxel_origin + [0, 0, voxel_size])
        vertices.append(voxel_origin + [0, voxel_size, 0])
        vertices.append(voxel_origin + [0, voxel_size, voxel_size])
        vertices.append(voxel_origin + [voxel_size, 0, 0])
        vertices.append(voxel_origin + [voxel_size, 0, voxel_size])
        vertices.append(voxel_origin + [voxel_size, voxel_size, 0])
        vertices.append(voxel_origin + [voxel_size, voxel_size, voxel_size])

        faces.append((np.array([1, 3, 0]) + voxel_index * 8).tolist())
        faces.append((np.array([4, 1, 0]) + voxel_index * 8).tolist())
        faces.append((np.array([0, 3, 2]) + voxel_index * 8).tolist())
        faces.append((np.array([2, 4, 0]) + voxel_index * 8).tolist())
        faces.append((np.array([1, 7, 3]) + voxel_index * 8).tolist())
        faces.append((np.array([5, 1, 4]) + voxel_index * 8).tolist())
        faces.append((np.array([5, 7, 1]) + voxel_index * 8).tolist())
        faces.append((np.array([3, 7, 2]) + voxel_index * 8).tolist())
        faces.append((np.array([6, 4, 2]) + voxel_index * 8).tolist())
        faces.append((np.array([2, 7, 6]) + voxel_index * 8).tolist())
        faces.append((np.array([6, 5, 4]) + voxel_index * 8).tolist())
        faces.append((np.array([7, 5, 6]) + voxel_index * 8).tolist())

    mesh.vertices = trimesh.caching.tracked_array(vertices)
    mesh.faces = trimesh.caching.tracked_array(faces)
    mesh.merge_vertices()

    return mesh


def create_mesh_voxelize_marcing_cubes(pcd, voxel_size=0.004):
    """Voxelize point cloud and apply marching cubes

    Parameters
    ----------
    pcd : open3d.open3d.geometry.PointCloud
      Input pcd data
    voxel_size : float

    Returns
    -------
    mesh : trimesh.base.Trimesh
    Mesh with voxelization and marching cubes applied
    """

    voxel_grid \
        = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)
    mesh = trimesh.Trimesh()
    grid_indices = []

    for voxel_index, voxel in enumerate(voxel_grid.get_voxels()):
        grid_indices.append(voxel.grid_index.tolist())

    x = max(grid_indices, key=lambda x: x[0])[0]
    y = max(grid_indices, key=lambda x: x[1])[1]
    z = max(grid_indices, key=lambda x: x[2])[2]

    occupacy_grid = np.zeros([x + 1, y + 1, z + 1], dtype=np.bool)

    for voxel_index, voxel in enumerate(voxel_grid.get_voxels()):
        occupacy_grid[voxel.grid_index[0],
                      voxel.grid_index[1],
                      voxel.grid_index[2]] = True

    mesh = trimesh.voxel.ops.matrix_to_marching_cubes(
        matrix=occupacy_grid,
        pitch=voxel_size)

    mesh.merge_vertices()
    # mesh.remove_duplicate_faces()

    return mesh


def create_urdf(mesh, output_dir):
    """Create urdf from mesh

    Parameters
    ----------
    mesh : trimesh.base.Trimesh
        input mesh
    output_dir : str
        Ouput directry where output mesh saved
    """

    current_dir = os.path.dirname(os.path.abspath(__file__))
    tree = ET.parse(os.path.join(current_dir, '../urdf/base/base.urdf'))
    root = tree.getroot()
    center = ''.join(str(i) + ' ' for i in mesh.centroid.tolist()).strip()
    root[0].find('inertial').find('origin').attrib['xyz'] = center
    # os.makedirs(os.path.join(output_dir), exist_ok=True)
    pathlib2.Path(os.path.join(output_dir)).mkdir(parents=True,
                                                  exist_ok=True)
    mesh.export(os.path.join(output_dir, 'base.stl'), "stl")
    tree.write(os.path.join(output_dir, 'base.urdf'),
               encoding='utf-8', xml_declaration=True)


def create_point_cloud(
        color, depth, intrinsic, voxel_size=0.002,
        estimate_normals=True, remove_outlier='statistical'):
    """Create point cloud

    Parameters
    ----------
    color : open3d.open3d.geometry.Image
        [description]
    depth : open3d.open3d.geometry.Image
        [description]
    intrinsic : open3d.open3d.camera.PinholeCameraIntrinsic
        [description]
    voxel_size : float, optional
        down sample voxel size, by default 0.002
    estimate_normals : bool, optional
        If True, estimate normals, by default True
    remove_outlier : str, optional
        remove_outlier method 'statistical' ot 'radius',
        by default 'statistical'

    Returns
    -------
    pcd : open3d.open3d.geometry.PointCloud
    """

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color, depth, depth_trunc=4.0, convert_rgb_to_intensity=False)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd, intrinsic)

    if voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size)

    if estimate_normals:
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.1, max_nn=30))

    if remove_outlier == 'statistical':
        pcd.remove_statistical_outlier(
            nb_neighbors=100, std_ratio=0.001)
    elif remove_outlier == 'radius':
        pcd.remove_radius_outlier(nb_points=100, radius=0.002)

    return pcd


def icp_registration(input_dir, scenes, voxel_size=0.002):
    """Estimate camera pose and create integrated point cloud

    TODO: Making input point cloud list is better.

    Parameters
    ----------
    input_dir : str
        Input directry which include color{:03}.png and depth{:03}.png.
        [description]
    scenes : int
        How many scenes were shot.
    voxel_size : float, optional
        voxel size, by default 0.002

    Returns
    -------
    camera_poses_icp : List of skrobot.coordinates.base.Coordinates
    pcd : open3d.open3d.geometry.PointCloud
    """

    image_shape = np.array(o3d.io.read_image(
        os.path.join(input_dir, "color000.png"))).shape
    width = image_shape[1]
    height = image_shape[0]
    intrinsic_np = np.loadtxt(os.path.join(
        input_dir, 'camera_pose/intrinsic.txt'))
    fx = intrinsic_np[0, 0]
    fy = intrinsic_np[1, 1]
    cx = intrinsic_np[0, 2]
    cy = intrinsic_np[1, 2]

    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(
        width, height, fx, fy, cx, cy)

    camera_poses = []
    camera_poses_icp = []
    pcds = []

    print('Create point cloud from rgb and depth.')
    for i in range(scenes):
        print('Create {:d}-th point cloud.'.format(i))
        camera_pose = np.loadtxt(
            os.path.join(input_dir,
                         'camera_pose/camera_pose{:03}.txt'.format(i)))

        color = o3d.io.read_image(
            os.path.join(input_dir,
                         'color{:03}.png'.format(i)))
        depth = o3d.io.read_image(
            os.path.join(input_dir,
                         'depth{:03}.png'.format(i)))

        pcd = create_point_cloud(color, depth, intrinsic)
        # o3d.visualization.draw_geometries([pcd])

        c = skrobot.coordinates.Coordinates(
            pos=camera_pose[:3, 3],
            rot=camera_pose[:3, :3])
        camera_poses.append(c)
        pcds.append(pcd)
        # mesh = o3d.geometry.TriangleMesh()
        # mesh.create_from_point_cloud_poisson(pcd)
        # o3d.visualization.draw_geometries([mesh])

    np.savetxt(
        os.path.join(input_dir,
                     'camera_pose/camera_pose_icp000.txt'),
        camera_poses[0].T())

    print('ICP registration start.')
    target = pcds[0]
    for i in range(scenes - 1):
        print('ICP registration {:d}-th point cloud.'.format(i + 1))
        trans_init = camera_poses[0].copy_worldcoords().inverse_transformation(
        ).transform(camera_poses[i + 1])

        source = pcds[i + 1]

        result_icp = o3d.registration.registration_icp(
            source, target, 0.02, trans_init.T(),
            o3d.registration.TransformationEstimationPointToPoint())

        icp_coords = skrobot.coordinates.Coordinates(
            pos=result_icp.transformation[:3, 3],
            rot=result_icp.transformation[:3, :3])

        camera_pose_icp = camera_poses[0].copy_worldcoords(
        ).transform(icp_coords)
        camera_poses_icp.append(camera_pose_icp)

        obj_transformation = camera_poses[0].copy_worldcoords().transform(
            camera_pose_icp.inverse_transformation())
        print('{}   {} '.format(obj_transformation.rpy_angle(),
                                obj_transformation.translation))
        np.savetxt(
            os.path.join(
                input_dir,
                'camera_pose/camera_pose_icp{:03}.txt'.format(i + 1)),
            camera_pose_icp.T())
        np.savetxt(
            os.path.join(
                input_dir,
                'camera_pose/obj_coords{:03}.txt'.format(i + 1)),
            obj_transformation.T())

        # Save camera pose and intrinsic for texture-mapping
        with open(os.path.join(input_dir,
                               'color{:03}.txt'.format(i + 1)), 'w') as f:
            np.savetxt(f, np.concatenate(
                [camera_pose_icp.T()[:3, 3][None, :],
                 camera_pose_icp.T()[:3, :3]],
                axis=0))
            np.savetxt(f, [fx])
            np.savetxt(f, [fy])
            np.savetxt(f, [cx])
            np.savetxt(f, [cy])
            np.savetxt(f, [height])
            np.savetxt(f, [width])

        # draw_registration_result_original_color(source, target,
        #                                         result_icp.transformation)

        source.transform(result_icp.transformation)

        target = target + source
        if voxel_size > 0:
            target = target.voxel_down_sample(voxel_size)
        target.remove_statistical_outlier(nb_neighbors=100,
                                          std_ratio=0.001)
        # pcd.remove_radius_outlier(nb_points=100, radius=0.002)
        # o3d.visualization.draw_geometries([target])

    target.remove_radius_outlier(nb_points=100, radius=0.002)
    # o3d.visualization.draw_geometries([target])
    # o3d.io.write_point_cloud(os.path.join(
    #     input_dir, 'icp_result.pcd'), target)

    return camera_poses_icp, target


def smoothing_mesh(mesh, method='humphrey'):
    if method == 'humphrey':
        trimesh.smoothing.filter_humphrey(mesh, alpha=0.1, beta=0.5,
                                          iterations=10,
                                          laplacian_operator=None)
    if method == 'laplacian':
        trimesh.smoothing.filter_laplacian(mesh, lamb=0.3, iterations=10,
                                           implicit_time_integration=False,
                                           volume_constraint=True,
                                           laplacian_operator=None)
    if method == 'taubin':
        trimesh.smoothing.filter_taubin(mesh, lamb=0.5, nu=0.5,
                                        iterations=10, laplacian_operator=None)
    if method == 'laplacian_calculation':
        trimesh.smoothing.laplacian_calculation(mesh, equal_weight=True)

    return mesh
