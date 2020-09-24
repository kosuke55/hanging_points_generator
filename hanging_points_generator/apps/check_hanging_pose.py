import argparse
import os

from hanging_points_generator.generator_utils \
    import check_contact_points


def main():
    # current_dir = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--input', '-i', type=str,
                        help='input urdf',
                        default='')
    parser.add_argument('--pose', '-p', type=str,
                        help='input pose',
                        required=True)
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

    check_contact_points(args.pose, args.input,
                         cluster_min_points=args.clustering,
                         use_filter_penetration=args.filter_penetration,
                         inf_penetration_check=args.inf_penetration_check,
                         align=args.align, average=args.average,
                         average_pos=args.average_pos)

if __name__ == '__main__':
    main()
