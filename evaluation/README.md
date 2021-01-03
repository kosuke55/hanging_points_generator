## an example for pouring

1. Annotate objects using [pose_annotation_tool](https://github.com/kosuke55/pose_annotation_tool) after making an annotation directory with [make_annotation_dir.py](https://github.com/kosuke55/pose_annotation_tool/blob/master/utils/make_annotation_dir.py)

```
python make_coords_json.py -i '/media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf'
rosrun annotation_tool annotation_tool
```
2. Convert manual annotation format to coords json using [make_coords_json.py](https://github.com/kosuke55/pose_annotation_tool/blob/master/utils/make_coords_json.py).
```
python make_coords_json.py -i '/media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf/annotation_obj'
```

3. Generate pouring points with pybullet
```
run-many 'python generate_pouring_points.py -i /media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf --skip' -n 4 -j 4
```

4. Filter pouring points
```
filter-points -i /media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf  -ppn p -n -1  -hipc -c -1
```

5. Calculate error with manual annotation
```
python eval_pose -i /media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf -gt /media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf/annotation_obj
```
