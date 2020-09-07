import os.path as osp

import gdown


def download_sample_data(output_dir=None, rosbag=True):
    # color, depth, camera_pose
    if output_dir is None:
        output = 'sample_data.tgz'
    else:
        output = osp.join(output_dir, 'sample_data.tgz')
    gdown.cached_download(
        'https://drive.google.com/uc?export=download&id=1UGSOMWHXaQBmoJFXAd5NGefd_DdLjPna',
        output,
        md5='ab9f0c690c79e1959742ec0dbfc162d3',
        postprocess=gdown.extractall)

    if rosbag:
        if output_dir is None:
            output = 'create_mesh_sample_rosbag.tgz'
        else:
            output = osp.join(output_dir, 'create_mesh_sample_rosbag.tgz')
        gdown.cached_download(
            'https://drive.google.com/uc?export=download&id=1uacdgaBiNqkfbxe5NwvzpBO3hi7w-MVl',
            output,
            md5='ec2ff6bc03b8a4643d23528e1107e01d',
            postprocess=gdown.extractall)
