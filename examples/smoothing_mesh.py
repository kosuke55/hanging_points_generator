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
                            '../sample_data/voxelized_obj.ply'))
    parser.add_argument('--method', '-m', type=str,
                        help='smoothing method "humphrey", "laplacian" \
                        + "taubin" or "laplacian_calculation"',
                        default='humphrey')
    args = parser.parse_args()

    mesh = trimesh.load(args.input)
    mesh = smoothing_mesh(mesh, method=args.method)
    mesh.show()
