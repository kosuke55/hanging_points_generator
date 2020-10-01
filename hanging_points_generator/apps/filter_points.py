import argparse

from hanging_points_generator.generator_utils \
    import filter_contact_points_dir


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--input', '-i', type=str,
                        help='input directory',
                        default='')
    args = parser.parse_args()

    filter_contact_points_dir(args.input)


if __name__ == '__main__':
    main()
