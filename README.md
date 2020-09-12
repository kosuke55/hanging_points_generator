# Mesh Reconstruction and Hanging Points Generator
[![Build Status](https://travis-ci.com/kosuke55/hanging_points_generator.svg?token=NmxadtM3pq1A9AssR1vm&branch=master)](https://travis-ci.com/kosuke55/hanging_points_generator)

## Setup
`pip install -e .`  
`pip3 install -r py3_requirement.txt`  

To use voxelization on ubuntu 16.04,  
`git clone -b hotfix/voxels https://github.com/kosuke55/Open3D.git`  
then refer to [this page](http://www.open3d.org/docs/release/compilation.html#ubuntu) and compile Open3d from source.([related PR](https://github.com/intel-isl/Open3D/pull/1688))  

If you use ubuntu 18.04 and python3, just
`pip install open3d`

## Mesh Reconstruction demo
```
python donwload_sample_data.py
cd examples/
python icp_camera_pose_estimation.py
python create_mesh_from_data.py
python create_mesh_voxelize_marching_cubes.py
```

## Mesh Reconstruction demo(ROS)
```
python donwload_sample_data.py
```

### inhand
```
roslaunch hanging_points_generator sample_create_mesh.launch
```
<img src="https://user-images.githubusercontent.com/39142679/92908721-775fe600-f461-11ea-907a-f56375f6be99.gif" height="200"> <img src="https://user-images.githubusercontent.com/39142679/92987591-dd4e7b00-f4fe-11ea-9730-efeac0674b7f.gif" height="200">  

### handeye
```
roslaunch hanging_points_generator sample_create_mesh_handeye.launch
```
<img src="https://user-images.githubusercontent.com/39142679/92906978-e0465e80-f45f-11ea-976d-a3dd0f3c1f3f.gif" height="200"> <img src="https://user-images.githubusercontent.com/39142679/92907012-edfbe400-f45f-11ea-8b3f-cc93dce589f9.gif" height="200">  

### texture mapping
Use [this](https://github.com/iory/texture-mapping) for texture mapping  
<img src="https://github.com/iory/texture-mapping/blob/master/docs/image/textured.gif?raw=true" width="200">  


## When using with ros (just an example in my environment)
`roslaunch hanging_points_generator hanging_points_generator.lanuch`  

#### create\_mesh_node.py services
- `/integrate_point_cloud`  : Add rgbd image and camerapose adjusted by icp registration to TSDF volume.  
- `/create_mesh` : Extract mesh.  
- `/meshfix` : Generate the completed mesh(urdf) from the missing mesh(ply).  
- `/generate_hanging_points` : Generate contact points using fixed mesh in pubullet and publish them as PoseArray.
- `/reset_volume` (just util): Reset voulume and integrate count.


Republish PointCloud and extract it around gripper. ex) [republish_kinectv2_hd_qhdhalf.launch](https://github.com/kosuke55/pr2demo/blob/master/launch/mesh_hooking/republish_kinectv2_hd_qhdhalf.launch), [attention_clipper_gripper.launch](https://github.com/kosuke55/pr2demo/blob/master/launch/mesh_hooking/attention_clipper_giripper.launch)  

In [skrobot_node.py](https://github.com/kosuke55/pr2demo/blob/master/scripts/skrobot_node.py)  
`create_mesh()`  
`generate_hanging_points()`  

And when hooking operation, lauch [mesh_hooking.launch](https://github.com/kosuke55/pr2demo/blob/master/launch/mesh_hooking/mesh_hooking.launch) to detect hook.


### Mesh reconstruction and Generating hanging points Result
**1.** Collect rgbd images.  
<img src="https://user-images.githubusercontent.com/39142679/80790397-ae77de00-8bc9-11ea-95cf-46130f707e6d.gif" width="200">  
**2.** Create mesh. (Left: ICP->TSDF Right: ICP->Voxelization-> Marching cubes)  
<img src="https://user-images.githubusercontent.com/39142679/80790404-b2a3fb80-8bc9-11ea-9b52-246e1c4273fe.gif"  alt="hoge" width="200" height="200" >  <img src="https://user-images.githubusercontent.com/39142679/80790323-7c667c00-8bc9-11ea-915c-bb51b1be854e.gif" width="200" height="200">  
**3.** Find hanging points in pybullet.  
<img src="https://user-images.githubusercontent.com/39142679/80790122-f8ac8f80-8bc8-11ea-8cdf-a20482292f1b.gif" width="200" height="200"> <img src="https://user-images.githubusercontent.com/39142679/80790221-3c06fe00-8bc9-11ea-9412-dd4971cc8866.gif" width="200" height="200">  

### How to check contact points
```
check_hanging_pose -i base.urdf -p contact_points -c 5 -f 1 -ipc 0
# If contact_points is dir, load multiple contact_points.

usage: check_hanging_pose [-h] --input INPUT --pose POSE
                          [--clustering CLUSTERING]
                          [--filter-penetration FILTER_PENETRATION]
                          [--inf-penetration-check INF_PENETRATION_CHECK]

optional arguments:
  -h, --help            show this help message and exit
  --input INPUT, -i INPUT
                        input urdf (default: None)
  --pose POSE, -p POSE  input pose (default: None)
  --clustering CLUSTERING, -c CLUSTERING
                        dbscan clustering min points (default: 2)
  --filter-penetration FILTER_PENETRATION, -f FILTER_PENETRATION
                        filter penetration (default: 0)
  --inf-penetration-check INF_PENETRATION_CHECK, -ipc INF_PENETRATION_CHECK
                        infinity penetration check (default: 1)
```

### Externals
[andyzeng/tsdf-fusion-python](https://github.com/andyzeng/tsdf-fusion-python)  
```
@inproceedings{zeng20163dmatch,
    title={3DMatch: Learning Local Geometric Descriptors from RGB-D Reconstructions},
    author={Zeng, Andy and Song, Shuran and Nie{\ss}ner, Matthias and Fisher, Matthew and Xiao, Jianxiong and Funkhouser, Thomas},
    booktitle={CVPR},
    year={2017}
}
```
