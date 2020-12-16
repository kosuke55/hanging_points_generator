import argparse
import os.path as osp
from pathlib import Path

from hanging_points_generator.generator_utils \
    import check_contact_points
from hanging_points_generator.generator_utils \
    import load_list


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--input', '-i', type=str,
                        help='input urdf',
                        default='')
    parser.add_argument('--input-file-name', '-ifn', type=str,
                        help='input object file name. '
                        + 'base.urdf or textured.urdf',
                        default='base.urdf')
    parser.add_argument('--idx', type=int,
                        help='data idx',
                        default=0)
    parser.add_argument('--pose', '-p', type=str,
                        help='input pose',
                        default='')
    parser.add_argument('--pose-file-name', '-pfn', type=str,
                        help='input pose file name. '
                        + 'This is needed only when input is directory'
                        + 'ex)contact_points, filtered_contact_points.json',
                        default='contact_points')
    parser.add_argument('--clustering', '-c', type=int,
                        help='dbscan clustering min points', default=0)
    parser.add_argument('--eps', '-e', type=float,
                        help='dbscan eps params', default=0.01)
    parser.add_argument('--filter-penetration', '-f', type=int,
                        help='filter penetration', default=0)
    parser.add_argument('--inf-penetration-check', '-ipc', action='store_true',
                        help='infinity penetration check ')
    parser.add_argument('--align', action='store_true',
                        help='align coords')
    parser.add_argument('--average', action='store_true',
                        help='average coords rot')
    parser.add_argument('--average-pos', action='store_true',
                        help='average coords pos')
    parser.add_argument('--skip-list-file', '-slf', type=str,
                        help='slip obect list file ',
                        default='skip-list-file')
    parser.add_argument('--large-axis', '-la', action='store_true',
                        help='use large axis as visulaizing marker')
    parser.add_argument('--just-check-num-points', '-jcnp',
                        action='store_true',
                        help='just check nuber of points without visualiziong')

    args = parser.parse_args()
    input_file_name = args.input_file_name
    pose_file_name = args.pose_file_name
    skip_list_file = args.skip_list_file
    if input_file_name == 'b':
        input_file_name = 'base.urdf'
    elif input_file_name == 't':
        input_file_name = 'textured.urdf'

    if pose_file_name == 'c':
        pose_file_name = 'contact_points'
    if pose_file_name == 'p':
        pose_file_name = 'pouring_points'
    elif pose_file_name == 'f':
        pose_file_name = 'filtered_contact_points.json'

    if osp.isfile(args.input):
        json_name = Path(pose_file_name).with_suffix('.json').name
        check_contact_points(args.pose, args.input, json_name=json_name,
                             cluster_min_points=args.clustering,
                             use_filter_penetration=args.filter_penetration,
                             inf_penetration_check=args.inf_penetration_check,
                             align=args.align, average=args.average,
                             average_pos=args.average_pos, large_axis=args.large_axis,
                             just_check_num_points=args.just_check_num_points)
    else:
        base_dir = args.input
        pose_path = list(Path(base_dir).glob('*/%s' % pose_file_name))
        start_idx = args.idx
        skip_list_file = str(Path(base_dir) / skip_list_file)
        skip_list = []
        if osp.isfile(skip_list_file):
            skip_list = load_list(osp.join(base_dir, skip_list_file))
        try:
            idx = -1
            for path in pose_path:
                idx += 1
                print('-----------------------')
                print('%s : %d' % (str(path), idx))
                if idx < start_idx:
                    continue
                category_name = path.parent.name
                if category_name in skip_list:
                    print(
                        'Skipped %s because it is in skip_list' %
                        category_name)
                    continue
                pose = str(path)
                urdf = str(path.parent / input_file_name)
                check_contact_points(
                    pose,
                    urdf,
                    cluster_min_points=args.clustering,
                    eps=args.eps,
                    use_filter_penetration=args.filter_penetration,
                    inf_penetration_check=args.inf_penetration_check,
                    align=args.align,
                    average=args.average,
                    average_pos=args.average_pos,
                    large_axis=args.large_axis,
                    just_check_num_points=args.just_check_num_points)
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
