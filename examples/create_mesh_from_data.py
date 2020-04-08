#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import open3d as o3d
import os

from hanging_points_generator.create_mesh import create_mesh_tsdf


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--input', '-i', type=str,
                        help='input dir',
                        default=os.path.join(
                            os.path.dirname(os.path.abspath(__file__)),
                            '../save_dir/'))
    parser.add_argument('--output', '-o', type=str,
                        help='output file name',
                        default=os.path.join(
                            os.path.dirname(os.path.abspath(__file__)),
                            '../save_dir/out.ply'))
    parser.add_argument('--scenes', '-s', type=int,
                        help='How many scenes were shot.',
                        default=4)
    parser.add_argument('--voxel_length', '-vl', type=float,
                        help='voxel_length',
                        default=0.002)
    parser.add_argument('--sdf_trunc', '-st', type=float,
                        help='sdf_trunc',
                        default=0.002)
    args = parser.parse_args()

    mesh = create_mesh_tsdf(args.input,
                            args.output,
                            args.scenes,
                            args.voxel_length)
    o3d.visualization.draw_geometries([mesh])
