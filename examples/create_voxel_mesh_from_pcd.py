import argparse
import open3d as o3d
import os

from hanging_points_generator.create_mesh import create_voxelized_mesh

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
                            '../save_dir/voxelized_obj.ply'))
    args = parser.parse_args()

    pcd = o3d.io.read_point_cloud(args.input)
    mesh = create_voxelized_mesh(pcd, voxel_size=0.002)
    mesh.show()
    mesh.export(args.output)
