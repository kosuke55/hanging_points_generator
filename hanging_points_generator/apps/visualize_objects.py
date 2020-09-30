import argparse
import os.path as osp
from pathlib import Path
from six.moves import input

import skrobot


def visualize_objects_dir(input_dir, filename='base.urdf'):
    paths = Path(input_dir).glob(osp.join('*', filename))
    viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
    init = True
    for path in paths:
        print(path)
        root, ext = osp.splitext(str(path))
        if not init:
            viewer.delete(obj)  # noqa

        if 'obj' in ext.lower() or 'off' in ext.lower() \
                or 'ply' in ext.lower() or 'stl' in ext.lower():
            obj = skrobot.models.MeshLink(str(path))

        elif 'urdf' in ext.lower():
            obj = skrobot.models.urdf.RobotModelFromURDF(
                urdf_file=osp.abspath(str(path)))

        viewer.add(obj)
        if init:
            viewer.show()
            init = False
        input('')


def main():
    # current_dir = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--input', '-i', type=str,
                        help='input dir', required=True)
    parser.add_argument('--filename', '-f', type=str,
                        help='filename', default='base.urdf')
    args = parser.parse_args()
    visualize_objects_dir(args.input, args.filename)


if __name__ == '__main__':
    main()
