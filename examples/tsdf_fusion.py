"""
Fuse RGB-D images in selected directory.
"""
from hanging_points_generator.lib import fusion

import argparse
import cv2
import numpy as np
import os
import time
import trimesh

if __name__ == "__main__":
    # (Optional) This is an example of how to compute the 3D bounds
    # in world coordinates of the convex hull of all camera view
    # frustums in the dataset
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--input_dir',
        "-i",
        help='input data directory in which formed data are contained')
    parser.add_argument(
        '--n_imgs',
        "-n",
        type=int,
        help='number of images in the selected directrory')
    args = parser.parse_args()
    print("Estimating voxel volume bounds...")
    n_imgs = args.n_imgs
    cam_intr = np.loadtxt(
        os.path.join(
            args.input_dir,
            "camera_pose/intrinsic.txt"))
    vol_bnds = np.zeros((3, 2))
    for i in range(n_imgs):
        depth_im = cv2.imread(
            os.path.join(args.input_dir, "depth%03d.png" %
                         (i)), -1).astype(float)
        print(depth_im.shape)
        depth_im /= 1000.  # depth is saved in 16-bit PNG in millimeters
        # set invalid depth to 0 (specific to 7-scenes dataset)
        # 4x4 rigid transformation matrix
        depth_im[depth_im == 65.535] = 0
        cam_pose = np.loadtxt(
            os.path.join(args.input_dir,
                         "camera_pose/camera_pose_icp%03d.txt" % (i)))

        # Compute camera view frustum and extend convex hull
        view_frust_pts = fusion.get_view_frustum(depth_im, cam_intr, cam_pose)
        vol_bnds[:, 0] = np.minimum(
            vol_bnds[:, 0], np.amin(view_frust_pts, axis=1))
        vol_bnds[:, 1] = np.maximum(
            vol_bnds[:, 1], np.amax(view_frust_pts, axis=1))

    # Integrate
    # Initialize voxel volume
    print("Initializing voxel volume...")
    tsdf_vol = fusion.TSDFVolume(vol_bnds, voxel_size=0.002)

    # Loop through RGB-D images and fuse them together
    t0_elapse = time.time()
    for i in range(n_imgs):
        print("Fusing frame %d/%d" % (i + 1, n_imgs))
        color_image = cv2.cvtColor(
            cv2.imread(
                os.path.join(args.input_dir, "color%03d.png" %
                             (i))), cv2.COLOR_BGR2RGB)
        depth_im = cv2.imread(
            os.path.join(args.input_dir, "depth%03d.png" %
                         (i)), -1).astype(float)
        depth_im /= 1000.
        depth_im[depth_im == 65.535] = 0
        # 4x4 rigid transformation matrix
        cam_pose = np.loadtxt(
            os.path.join(args.input_dir,
                         "camera_pose/camera_pose_icp%03d.txt" % (i)))

        # Integrate observation into voxel volume (assume color aligned with
        # depth)
        tsdf_vol.integrate(
            color_image,
            depth_im,
            cam_intr,
            cam_pose,
            obs_weight=1.)

    fps = n_imgs / (time.time() - t0_elapse)
    print("Average FPS: {:.2f}".format(fps))

    # Get mesh from voxel volume and save to disk (can be viewed with Meshlab)
    print("Saving mesh to mesh.ply...")
    verts, faces, norms, colors = tsdf_vol.get_mesh()
    fusion.meshwrite("/tmp/mesh.ply", verts, faces, norms, colors)
    mesh = trimesh.load("/tmp/mesh.ply")
    mesh.show()

    # Get point cloud from voxel volume and save to disk (can be viewed with
    # Meshlab)
    print("Saving point cloud to pc.ply...")
    point_cloud = tsdf_vol.get_point_cloud()
    fusion.pcwrite("/tmp/pc.ply", point_cloud)
    pc = trimesh.load("/tmp/pc.ply")
    pc.show()
