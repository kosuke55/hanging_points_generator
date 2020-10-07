# Random Mesh Generator

### 3D-IWGAN
Generate a random shape using 3D-IWGAN https://arxiv.org/abs/1707.09557 method.  
This repository uses kaolin's code.  
https://github.com/NVIDIAGameWorks/kaolin/tree/master/examples/GANs/3D-IWGAN

### Training the network:

To train run
```
python train.py --modelnet-root path/to/modelnet --cache-dir cache/
```

### Install
- https://github.com/kosuke55/kaolin/tree/kosuke55-master
- this repository
- open3d

```
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
```