## hanging\_points\_generator

`rosrun hanging_points_generator create_mesh.py`  
`python hanging_points_generator.py`  **TODO call by serice.**  
`rosrun hanging_points_generator pub_contact_points.py`  

###create_mesh.py services  
- `/integrate_point_cloud`  : Add rgbd image and camerapose adjusted by icp registration to TSDF volume.  
- `/create_mesh` : Extract mesh.  
- **TODO** `/meshfix` : Generate the completed mesh(urdf) from the missing mesh(ply).  

For example, in [skrobot_node.py](https://github.com/kosuke55/pr2demo/blob/master/scripts/skrobot_node.py)  
`create_mesh_initial_pose()`  
`create_mesh()`  
