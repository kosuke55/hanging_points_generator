## hanging\_points\_generator

`rosrun hanging_points_generator create_mesh.py`  
`rosrun hanging_points_generator pub_contact_points.py`  

`/integrate_point_cloud`  is the service for adding rgbd image to TSDF volume using camerapose adjusted by icp registration.  
`/create_mesh` is the service for extracting mesh.  

For example, in [skrobot_node.py](https://github.com/kosuke55/pr2demo/blob/master/scripts/skrobot_node.py)  
`create_mesh_initial_pose()`  
`create_mesh()`  
