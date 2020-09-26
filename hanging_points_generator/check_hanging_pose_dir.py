import argparse
from pathlib import Path

from hanging_points_generator.generator_utils \
    import check_contact_points


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

parser.add_argument('--clustering', '-c', type=int,
                    help='dbscan clustering min points', default=0)
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
base_path = list(Path(args.input_dir).glob('*/contact_points'))
start_idx = args.idx

try:
    idx = -1
    for path in base_path:
        idx += 1
        if idx < start_idx:
            continue
        print(str(path), idx)
        pose = str(path)
        urdf = str(path.parent / 'base.urdf')
        check_contact_points(pose, urdf,
                             cluster_min_points=args.clustering,
                             use_filter_penetration=args.filter_penetration,
                             inf_penetration_check=args.inf_penetration_check,
                             align=args.align, average=args.average,
                             average_pos=args.average_pos)

except KeyboardInterrupt:
    pass
