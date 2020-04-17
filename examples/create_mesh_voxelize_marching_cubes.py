import argparse
import open3d as o3d
import os

from hanging_points_generator.create_mesh \
    import create_mesh_voxelize_marcing_cubes, create_urdf

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--input', '-i', type=str,
                        help='input pcd file',
                        default=os.path.join(
                            os.path.dirname(os.path.abspath(__file__)),
                            '../save_dir/obj.pcd'))
    parser.add_argument('--output', '-o', type=str,
                        help='output file name',
                        default=os.path.join(
                            os.path.dirname(os.path.abspath(__file__)),
                            '../save_dir/voxelized_mc_obj.ply'))
    args = parser.parse_args()

    pcd = o3d.io.read_point_cloud(args.input)
    mesh = create_mesh_voxelize_marcing_cubes(pcd, voxel_size=0.004)
    mesh.show()
    mesh.export(args.output)
    create_urdf(mesh, os.path.dirname(args.output))
