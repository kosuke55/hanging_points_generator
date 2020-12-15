import argparse

from hanging_points_generator.donwloader import download_sample_data

if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--output-dir', '-o', type=str,
                        help='output dir',
                        default=None)
    parser.add_argument('--rgbd', type=int,
                        help='Download sample rgbd and camera pose file.',
                        default=0)
    parser.add_argument('--urdf', type=int,
                        help='Download urdf file.',
                        default=1)
    parser.add_argument('--rosbag', type=int,
                        help='Download sample bag file.',
                        default=0)
    args = parser.parse_args()
    output_dir = args.output_dir
    rosbag = args.rosbag
    download_sample_data(output_dir=output_dir, rosbag=rosbag)
