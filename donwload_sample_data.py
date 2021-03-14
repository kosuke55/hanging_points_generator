import argparse

from hanging_points_generator.donwloader import download_sample_data

if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--output-dir', '-o', type=str,
                        help='output dir',
                        default=None)
    parser.add_argument('--rgbd', action='store_true',
                        help='Download sample rgbd and camera pose file.')
    parser.add_argument('--urdf', action='store_true',
                        help='Download urdf file.')
    parser.add_argument('--ycb-eval-data', action='store_true',
                        help='Download ycb evaluation data.')
    parser.add_argument('--rosbag', action='store_true',
                        help='Download sample bag file.')
    args = parser.parse_args()
    output_dir = args.output_dir
    rgbd = args.rgbd
    urdf = args.urdf
    ycb_eval_data = args.ycb_eval_data
    rosbag = args.rosbag

    download_sample_data(
        output_dir=output_dir,
        rgbd=rgbd,
        urdf=urdf,
        ycb_eval_data=ycb_eval_data,
        rosbag=rosbag)
