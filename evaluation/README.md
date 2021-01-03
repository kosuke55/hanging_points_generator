## an example for pouring

- Annotate objects using [pose_annotation_tool](https://github.com/kosuke55/pose_annotation_tool) after making an annotation directory with [make_annotation_dir.py](https://github.com/kosuke55/pose_annotation_tool/blob/master/utils/make_annotation_dir.py)

```
python make_coords_json.py -i '/media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf'
rosrun annotation_tool annotation_tool
```
- Convert manual annotation format to coords json using [make_coords_json.py](https://github.com/kosuke55/pose_annotation_tool/blob/master/utils/make_coords_json.py).
```
python make_coords_json.py -i '/media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf/annotation_obj'
```

- Generate pouring points with pybullet
```
run-many 'python generate_pouring_points.py -i /media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf --skip' -n 4 -j 4
```

- Filter pouring points
```
filter-points -i /media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf  -ppn p -n -1  -hipc -c -1
```

- Calculate error with manual annotation
```
python eval_pose -i /media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf -gt /media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf/annotation_obj
```
