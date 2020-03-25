#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
python3
Generate the completed mesh(urdf) from the missing mesh(ply).
'''

import argparse
import os
import trimesh
import xml.etree.ElementTree as ET

from pymeshfix import _meshfix


def fix(input_mesh, output_dir):
    mesh = trimesh.load(input_mesh)
    mesh.export(input_mesh)

    tin = _meshfix.PyTMesh()
    tin.load_file(input_mesh)
    tin.fill_small_boundaries()
    tin.clean(max_iters=10, inner_loops=3)
    tin.save_file(input_mesh)

    mesh = trimesh.load(input_mesh)
    current_dir = os.path.dirname(os.path.abspath(__file__))
    tree = ET.parse(os.path.join(current_dir, 'urdf/base/base.urdf'))
    root = tree.getroot()
    center = ''.join(str(i) + ' ' for i in mesh.centroid.tolist()).strip()
    root[0].find('inertial').find('origin').attrib['xyz'] = center
    os.makedirs(os.path.join(output_dir), exist_ok=True)
    mesh.export(output_dir + 'base.stl', "stl")
    tree.write(output_dir + 'base.urdf',
               encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--input_mesh', '-i', type=str,
                        help='input mesh file',
                        default='save_dir/obj.ply')
    parser.add_argument('--output_dir', '-o', type=str,
                        help='output mesh file',
                        default='save_dir/')
    args = parser.parse_args()

    fix(args.input_mesh, args.output_dir)
