import argparse
import os

from hanging_points_generator.hp_generator \
    import check_contact_points


def main():
    # current_dir = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--input', '-i', type=str,
                        help='input urdf',
                        required=True)
    parser.add_argument('--pose', '-p', type=str,
                        help='input pose',
                        required=True)
    parser.add_argument('--clustering', '-c', type=int,
                        help='dbscan clustering',
                        default=1)
    parser.add_argument('--filter_penetration', '-f', type=int,
                        help='filter penetration',
                        default=0)
    args = parser.parse_args()

    check_contact_points(args.pose, args.input,
                         use_clustering=args.clustering,
                         use_filter_penetration=args.filter_penetration)


if __name__ == '__main__':
    main()
