These are for [The YCB Object and Model Set](http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/)  
Put a symlink for each code under ycb_directry or copy the off directory to this directory.  

```
hanging_object_list = [
    "019_pitcher_base",
    "022_windex_bottle",
    "025_mug",
    "033_spatula",
    "035_power_drill",
    "042_adjustable_wrench",
    "048_hammer",
    "050_medium_clamp",
    "051_large_clamp",
    "052_extra_large_clamp"
]

urdf
├── 019_pitcher_base
│   ├── base.stl
│   ├── base.urdf
│   ├── contact_points.json
│   ├── kinbody.xml
│   ├── nontextured.ply
│   ├── nontextured.stl
│   ├── rescale_base.urdf
│   ├── textured.dae
│   ├── textured.mtl
│   ├── textured.obj
│   ├── textured.urdf
│   └── texture_map.png

dirname, filename = os.path.split(file)
filename_without_ext, ext = os.path.splitext(filename)
category_name = dirname.split("/")[-1]  # 019_pitcher_base
```

