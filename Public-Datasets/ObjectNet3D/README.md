These are for [ObjectNet3D](https://cvgl.stanford.edu/projects/objectnet3d/)  
Put a symlink for each code under ObjectNet3D/CAD or copy the off directory to this directory.  

hanging object list  

* cap  
* cup  
* headphone  
* helmet  
* key  
* scissors  
* slipper

```
urdf
├── cap
│   ├── 01
│   │   ├── base.stl
│   │   ├── base.urdf
│   │   ├── contact_points.json
│   │   └── rescale_base.urdf

dirname, filename = os.path.split(file)
filename_without_ext, ext = os.path.splitext(filename)
category_name = dirname.split("/")[-2]  # cap
idx = dirname.split("/")[-1]  # 01
```