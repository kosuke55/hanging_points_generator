#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import open3d as o3d
import os


from hanging_points_generator.create_mesh import icp_registration


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--input', '-i', type=str,
                        help='input dir',
                        default=os.path.join(
                            os.path.dirname(os.path.abspath(__file__)),
                            '../save_dir/'))
    parser.add_argument('--scenes', '-s', type=int,
                        help='How many scenes were shot.',
                        default=2)
    parser.add_argument('--voxel_size', '-v', type=float,
                        help='voxel length for down sampling.',
                        default=0.001)
    args = parser.parse_args()

    camera_poses_icp, pcd = icp_registration(args.input,
                                             args.scenes,
                                             args.voxel_size)
    o3d.visualization.draw_geometries([pcd])
