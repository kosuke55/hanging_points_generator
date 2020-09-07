import shutil
import gdown


def download_sample_data(output_dir=None):
    url = 'https://drive.google.com/uc?export=download&id=1UGSOMWHXaQBmoJFXAd5NGefd_DdLjPna'  # noqa
    output = 'sample_data.tgz'
    md5 = 'ab9f0c690c79e1959742ec0dbfc162d3'
    gdown.cached_download(url, output, md5=md5, postprocess=gdown.extractall)

    if output_dir is not None:
        shutil.move('sample_data', output_dir)
