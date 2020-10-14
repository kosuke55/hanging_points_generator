import argparse

from hanging_points_generator.generator_utils \
    import filter_contact_points_dir


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--input', '-i', type=str,
                        help='input directory',
                        default='')
    parser.add_argument('--rate-thresh', '-r', type=float,
                        help='remaining rate threshold',
                        default=0.1)
    parser.add_argument('--num-samples', '-n', type=int,
                        help='nuber of sampling ponts after processing,'
                        + 'If -1 remain all points',
                        default=30)
    args = parser.parse_args()

    filter_contact_points_dir(args.input, args.rate_thresh, args.num_samples)


if __name__ == '__main__':
    main()
