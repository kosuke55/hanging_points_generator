import argparse
import numpy as np
import open3d as o3d
import os
import trimesh


def create_voxel_mesh_from_pcd(pcd, voxel_size=0.002):
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

    return mesh


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--input', '-i', type=str,
                        help='input pcd file',
                        default=os.path.join(
                            os.path.dirname(os.path.abspath(__file__)),
                            '../scissors_realsense_work/obj.pcd'))

    args = parser.parse_args()

    pcd = o3d.io.read_point_cloud(args.input)
    mesh = create_voxel_mesh_from_pcd(pcd, voxel_size=0.002)
    mesh.show()
