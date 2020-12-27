These are for [The YCB Object and Model Set](http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/)  
Use [ycb_downloader.py](http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/scripts_to_publish/ycb_downloader.py) to download ycb mesh object.  
Change this code like
```
output_directory = "./ycb_pouring_object_16"
files_to_download = ["google_16k"]

# hanging
objects_to_download = [
    "019_pitcher_base",
    "022_windex_bottle",
    "025_mug",
    "033_spatula",
    "035_power_drill",
    "037_scissors",
    "042_adjustable_wrench",
    "048_hammer",
    "050_medium_clamp",
    "051_large_clamp",
    "052_extra_large_clamp"
]

# pouring
objects_to_download = [
    "019_pitcher_base",
    "024_bowl",
    "025_mug",
    "027_skillet",
    "029_plate",
]
```

The skillet has a lid and is not a continuous mesh.  
Therefore, it is necessary to separate them using meshlab etc.  
[Here](https://drive.google.com/file/d/1kzWz5RrqptLo9tnOjkAi8z-faYVlpmdX/view?usp=sharing) is the mesh with the lid removed.

And run 'python [stl2urdf.py](./stl2urdf.py) -i <path_to_ycb_dir>' for creating urdf.
