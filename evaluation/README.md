an example for pouring

In advance, convert manual annotation format to coords json using [make_coords_json.py](https://github.com/kosuke55/pose_annotation_tool/blob/master/utils/make_coords_json.py).
```
python make_coords_json.py -i '/media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf/annotation_obj'
```

```
run-many 'python generate_pouring_points.py -i /media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf --skip' -n 4 -j 4
filter-points -i /media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf  -ppn p -n -1  -hipc -c -1
python eval_pose -i /media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf -gt /media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf/annotation_obj
```