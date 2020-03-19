#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import trimesh

from pymeshfix import _meshfix


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--input', '-i', type=str,
                        help='input mesh file',
                        default='save_dir/obj.ply')
    parser.add_argument('--output', '-o', type=str,
                        help='output mesh file',
                        default='save_dir/obj_fix.ply')
    args = parser.parse_args()

    mesh = trimesh.load(args.input)
    mesh.export(args.input)

    # _meshfix.clean_from_file(args.input, args.output)

    tin = _meshfix.PyTMesh()
    tin.load_file(args.input)
    tin.fill_small_boundaries()
    tin.clean(max_iters=10, inner_loops=3)
    tin.save_file(args.output)
