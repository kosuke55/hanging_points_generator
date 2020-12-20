import argparse

from hanging_points_generator.generator_utils \
    import filter_contact_points_dir


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--input', '-i', type=str,
                        help='input directory',
                        default='')
    parser.add_argument('--clustering', '-c', type=int,
                        help='dbscan clustering min points', default=0)
    parser.add_argument('--eps', '-e', type=int,
                        help='dbscan clustering eps', default=0.03)
    parser.add_argument('--rate-thresh', '-r', type=float,
                        help='remaining rate threshold',
                        default=0.1)
    parser.add_argument('--num-samples', '-n', type=int,
                        help='nuber of sampling ponts after processing,'
                        + 'If -1 remain all points',
                        default=30)
    parser.add_argument('--filter-penetration', '-f',
                        action='store_true',
                        help='filter penetration')
    parser.add_argument('--inf-penetration-check', '-ipc', action='store_true',
                        help='infinity penetration check ')
    parser.add_argument('--half-inf-penetration-check', '-hipc',
                        action='store_true',
                        help='half infinity penetration check ')
    parser.add_argument('--points-path-name', '-ppn', type=str,
                        help='points path name. contact_points is for hanging.'
                        'contact_points is for hanging.'
                        'pouring_points is for pouring.'
                        'if you give nothing, use default value hanging'
                        'else if you give p, it becanes pouring',
                        default='contact_points')
    parser.add_argument('--suffix', '-s', type=str,
                        help='output file suffix',
                        default='')
    args = parser.parse_args()

    filter_contact_points_dir(
        args.input, args.clustering, args.eps,
        args.rate_thresh, args.num_samples,
        use_filter_penetration=args.filter_penetration,
        inf_penetration_check=args.inf_penetration_check,
        half_inf_penetration_check=args.half_inf_penetration_check,
        points_path_name=args.points_path_name,
        suffix=args.suffix)


if __name__ == '__main__':
    main()
