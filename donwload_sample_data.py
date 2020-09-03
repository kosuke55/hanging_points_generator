import subprocess

import gdown


url = 'https://drive.google.com/uc?export=download&id=1Ky6rrfpPxGyzzVK6ZwAnA1U40aJwXfrH'

output = 'sample_data.tgz'
gdown.download(url, output, quiet=False)

md5 = '1721569ed4e0d83ab1b65eb34dfcd546'

gdown.cached_download(url, output, md5=md5, postprocess=gdown.extractall)

subprocess.call('tar -xzvf sample_data.tgz sample_data', shell=True)
