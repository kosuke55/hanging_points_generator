## hanging\_points\_generator

`rosrun hanging_points_generator create_mesh.py`  
`rosrun hanging_points_generator pub_contact_points.py`  

### create_mesh.py services  
- `/integrate_point_cloud`  : Add rgbd image and camerapose adjusted by icp registration to TSDF volume.  
- `/create_mesh` : Extract mesh.  
- `/meshfix` : Generate the completed mesh(urdf) from the missing mesh(ply).  
- `/generate_hanging_points` : Generate contact points using fixed mesh in pubullet and publish them as PoseArray.
- `/reset_volume` (just util): Reset voulume and integrate count.


For example, in [skrobot_node.py](https://github.com/kosuke55/pr2demo/blob/master/scripts/skrobot_node.py)  
`create_mesh_initial_pose()`  
`create_mesh()`  
`generate_hanging_points()`  
