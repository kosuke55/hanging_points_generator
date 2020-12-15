import os.path as osp

import gdown


def download_sample_data(output_dir=None, rgbd=True, urdf=True, rosbag=True):
    # color, depth, camera_pose
    if rgbd:
        if output_dir is None:
            rgbd_path = 'sample_data.tgz'
        else:
            rgbd_path = osp.join(output_dir, 'sample_data.tgz')
        gdown.cached_download(
            'https://drive.google.com/uc?export=download&id=1UGSOMWHXaQBmoJFXAd5NGefd_DdLjPna',
            rgbd_path,
            md5='ab9f0c690c79e1959742ec0dbfc162d3',
            postprocess=gdown.extractall)

    # urdf
    if urdf:
        urdf_path = osp.joint(osp.dirname(osp.abspath(__file__)), 'urdf.tgz')
        gdown.cached_download(
            'https://drive.google.com/uc?export=download&id=1alkc-v-GQnAbpoIxxCBQ3fLOEpmRoN2Y',
            urdf_path,
            md5='b8d53e7315df1fad833676ef07b874d5',
            postprocess=gdown.extractall)

    # rosbag
    if rosbag:
        if output_dir is None:
            rosbag_inhand_path = 'create_mesh_sample_rosbag.tgz'
        else:
            rosbag_inhand_path = osp.join(
                output_dir, 'create_mesh_sample_rosbag.tgz')
        gdown.cached_download(
            'https://drive.google.com/uc?export=download&id=1yui__4qBy2mh2E3EBwJIKFtcYoZUXHkZ',
            rosbag_inhand_path,
            md5='2738459e0a58f4828715d23bfccaeb0f',
            postprocess=gdown.extractall)

        if output_dir is None:
            rosbag_handeye_path = 'create_mesh_handeye_sample_rosbag.tgz'
        else:
            rosbag_handeye_path = osp.join(
                output_dir, 'create_mesh_handeye_sample_rosbag.tgz')
        gdown.cached_download(
            'https://drive.google.com/uc?export=download&id=169jfyrdTAUmN35VEC61pUA0rJNOHVlnH',
            rosbag_handeye_path,
            md5='16dac750d95feb65365528dae2b217d5',
            postprocess=gdown.extractall)
