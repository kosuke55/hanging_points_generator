import argparse
import random


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--input', '-i', type=str,
                        help='input file',
                        default='filter_remain_list.txt')
    parser.add_argument('--output', '-o', type=str,
                        help='output file',
                        default='sampled_filter_remain_list.txt')
    parser.add_argument('--num-samples', '-n', type=int,
                        help='number of sampling data',
                        default=300)
    args = parser.parse_args()
    input_file = args.input
    output_file = args.output
    num_samples = args.num_samples

    files = []
    with open(input_file) as f:
        for file in f:
            files.append(file.strip())

    indices = random.sample(range(0, len(files)), num_samples)

    sampled_files = [files[i] for i in indices]

    print(sampled_files)
    print('number of sampled files: {}'.format(len(sampled_files)))

    with open(output_file, mode='w') as f:
        for file in sampled_files:
            f.writelines(file + '\n')


if __name__ == '__main__':
    main()
