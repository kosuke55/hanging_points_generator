import subprocess

import gdown


def download_sample_data():
    url = 'https://drive.google.com/uc?export=download&id=1UGSOMWHXaQBmoJFXAd5NGefd_DdLjPna'  # noqa
    output = 'sample_data.tgz'
    gdown.download(url, output, quiet=False)
    md5 = 'ab9f0c690c79e1959742ec0dbfc162d3'
    gdown.cached_download(url, output, md5=md5, postprocess=gdown.extractall)
    subprocess.call('tar -xzvf sample_data.tgz', shell=True)
