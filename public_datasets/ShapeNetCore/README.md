# ShapeNetCore

1. Download ShapeNetCore v2 from https://www.shapenet.org/download/shapenetcore.  
   However, for trainin GAN, use symbolic licks to make it the same as the v1 directory structure or use v1 instead of v2.  

2. Convert `model_normalized.onj` to urdf.  
   ```
   python obj2urdf_mini.py -i <ShapeNetCore dirrectory> -s <saved directory> -n <number of sampling> -t <task type. hanging or pouring>
   ```
   mini means to select a specific category from all objects and sample an arbitrary number for each.

3. After this process, genete function points using  
    [generate_hanging_points.py](../../hanging_points_generator/generate_hanging_points.py) or [generate_pouring_points.py](../../hanging_points_generator/generate_pouring_points.py)  

