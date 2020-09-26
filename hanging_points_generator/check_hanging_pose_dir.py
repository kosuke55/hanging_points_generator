import argparse
import os.path as osp
from pathlib import Path

from hanging_points_generator.generator_utils \
    import check_contact_points
from hanging_points_generator.generator_utils import load_bad_list


parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)

parser.add_argument(
    '--input-dir', '-i', type=str,
    help='input urdf',
    default='/media/kosuke55/SANDISK/meshdata/hanging_object')
parser.add_argument(
    '--idx', type=int,
    help='data idx',
    default=0)

parser.add_argument('--bad-list', '-b', type=int,
                    help='skip file in bad list',
                    default=0)

parser.add_argument('--clustering', '-c', type=int,
                    help='dbscan clustering min points', default=0)
parser.add_argument('--eps', '-e', type=float,
                    help='dbscan eps params', default=0.01)
parser.add_argument('--filter-penetration', '-f', type=int,
                    help='filter penetration', default=0)
parser.add_argument('--inf-penetration-check', '-ipc', type=int,
                    help='infinity penetration check ', default=1)
parser.add_argument('--align', type=int,
                    help='align coords', default=0)
parser.add_argument('--average', type=int,
                    help='average coords rot', default=0)
parser.add_argument('--average-pos', type=int,
                    help='average coords pos', default=0)
args = parser.parse_args()
base_dir = args.input_dir
pose_path = list(Path(base_dir).glob('*/contact_points'))
start_idx = args.idx

bad_list_file = str(Path(base_dir) / 'bad_list.txt')
if osp.isfile(bad_list_file):
    bad_list = load_bad_list(osp.join(base_dir, 'bad_list.txt'))

try:
    idx = -1
    for path in pose_path:
        print('-----------------------')
        print('%s : %d' % (str(path), idx))
        idx += 1
        if idx < start_idx:
            continue
        category_name = path.parent.name
        if category_name in bad_list:
            print('Skipped %s because it is in bad_list' % category_name)
            continue
        pose = str(path)
        urdf = str(path.parent / 'base.urdf')
        check_contact_points(pose, urdf,
                             cluster_min_points=args.clustering,
                             eps=args.eps,
                             use_filter_penetration=args.filter_penetration,
                             inf_penetration_check=args.inf_penetration_check,
                             align=args.align, average=args.average,
                             average_pos=args.average_pos)

except KeyboardInterrupt:
    pass
