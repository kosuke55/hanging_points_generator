# ShapeNetCore

1. Download ShapeNetCore v2 from https://www.shapenet.org/download/shapenetcore.  
   However, for trainin GAN, use symbolic link to make it the same as the v1 directory structure or use v1 instead of v2.  

2. Install https://github.com/kosuke55/shapenet_utils.

3. Convert `model_normalized.obj` to urdf.  
   ```
   python obj2urdf_mini.py -i <ShapeNetCore dirrectory> -s <saved directory> -n <number of sampling> -t <task type. hanging or pouring>
   ```
   mini means to select a specific category from all objects and sample an arbitrary number for each.

4. After this process, genete function points using  
    [generate_hanging_points.py](../../hanging_points_generator/generate_hanging_points.py) or [generate_pouring_points.py](../../hanging_points_generator/generate_pouring_points.py)  

5. filtering generated points.
    ```
    filter-points -i .  -ppn p -n -1  -hipc -c -1
    ```

6. Make a graph and a table of filtered generated points.
   ```
   python analyze_function_points.py -f <filtering_eresult.json> -o <output image path> -t <task type. hanging or pouring>
   ```
