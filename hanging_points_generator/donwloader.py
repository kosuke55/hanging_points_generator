import os.path as osp

import gdown


def download_sample_data(output_dir=None):
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
