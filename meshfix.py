#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import os
import trimesh
import xml.etree.ElementTree as ET

from pymeshfix import _meshfix


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--input', '-i', type=str,
                        help='input mesh file',
                        default='save_dir/obj.ply')
    parser.add_argument('--output_dir', '-o', type=str,
                        help='output mesh file',
                        default='save_dir/')
    args = parser.parse_args()

    mesh = trimesh.load(args.input)
    mesh.export(args.input)

    # _meshfix.clean_from_file(args.input, args.output)

    tin = _meshfix.PyTMesh()
    tin.load_file(args.input)
    tin.fill_small_boundaries()
    tin.clean(max_iters=10, inner_loops=3)
    tin.save_file(args.input)

    mesh = trimesh.load(args.input)
    current_dir = os.path.dirname(os.path.abspath(__file__))
    tree = ET.parse(os.path.join(current_dir, 'urdf/base/base.urdf'))
    root = tree.getroot()
    center = ''.join(str(i)+' ' for i in mesh.centroid.tolist()).strip()
    root[0].find('inertial').find('origin').attrib['xyz'] = center
    mesh.export(args.output_dir + 'base.stl', "stl")
    tree.write(args.output_dir + 'base.urdf',
               encoding='utf-8', xml_declaration=True)
