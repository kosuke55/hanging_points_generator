import argparse
import os
import trimesh

from hanging_points_generator.create_mesh import smoothing_mesh

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--input', '-i', type=str,
                        help='input ply file',
                        default=os.path.join(
                            os.path.dirname(os.path.abspath(__file__)),
                            '../save_dir/voxelized_obj.ply'))
    args = parser.parse_args()

    mesh = trimesh.load(args.input)
    mesh = smoothing_mesh(mesh, method='humphrey')
    mesh.show()
