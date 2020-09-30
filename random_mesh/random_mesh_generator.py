import argparse
import os

import cc3d
import numpy as np
import torch
import trimesh
import kaolin as kal

from architectures import Generator
from hanging_points_generator.create_mesh import create_urdf

parser = argparse.ArgumentParser()
parser.add_argument(
    '--savedir', type=str,
    # default='/media/kosuke/SANDISK/meshdata/random_shape',
    default='/media/kosuke/SANDISK/meshdata/random_shape_ycb',
    help='Directory to save mesh to.')
parser.add_argument(
    '--pretrained_model', '-p',
    type=str, help='Pretrained models',
    default='/home/kosuke55/kaolin/examples/GANs/3D-IWGAN/log/3D_IWGAN/gen_5000.pth')
parser.add_argument(
    '--prefix', type=str,
    default='random',
    help='filename prefix')
parser.add_argument(
    '--expid', type=str, default='3D_IWGAN',
    help='Unique experiment identifier.')
parser.add_argument(
    '--device', type=str, default='cuda',
    help='Device to use.')
parser.add_argument(
    '--batchsize', type=int, default=1, help='Batch size.')
args = parser.parse_args()

gen = Generator().to(args.device)
gen.load_state_dict(torch.load(args.pretrained_model))
gen.eval()

obj_id = 0
min_length = 0.1
max_length = 0.15

while(obj_id < 100):
    z = torch.normal(
        torch.zeros(args.batchsize, 200),
        torch.ones(args.batchsize, 200)).to(args.device)
    fake_voxels = gen(z)

    for idx, model in enumerate(fake_voxels):
        model = model[:-2, :-2, :-2]

        model[torch.where(model >= 0.7)] = 1
        model[torch.where(model < 0.7)] = 0
        print(torch.sum(model))
        if torch.sum(model) < 100:
            continue
        model = model.cpu().detach().numpy().astype(np.int32)
        model = cc3d.connected_components(model, connectivity=6)

        max_label = 1
        num_max_label = 0
        for label in range(1, np.max(model) + 1):
            num_label = np.count_nonzero(model == label)
            if num_max_label < num_label:
                num_max_label = num_label
                max_label = label

        if num_max_label == 0:
            continue

        model[np.where(model != max_label)] = 0
        model[np.where(model == max_label)] = 1

        volume = 1
        for i in np.max(np.where(model == 1), axis=1) - \
                np.min(np.where(model == 1), axis=1) + 1:
            volume *= i

        density = np.count_nonzero(model) / volume
        # if density > 0.3:
        #     continue

        model = model.astype(np.float32)
        verts, faces = kal.conversions.voxelgrid_to_quadmesh(model)
        center = torch.mean(verts, dim=0)
        verts -= center
        max_xyz = torch.max(verts, axis=0).values
        min_xyz = torch.min(verts, axis=0).values
        length = torch.max(max_xyz - min_xyz, axis=0).values
        verts = verts * 0.15 / length  # -0.075~0.075

        mesh = kal.rep.QuadMesh.from_tensors(verts, faces)
        mesh.laplacian_smoothing(iterations=3)
        obj_dir = os.path.join(
            args.savedir, args.prefix + '_{:05}'.format(obj_id))
        os.makedirs(obj_dir, exist_ok=True)
        obj_file = os.path.join(obj_dir, 'tmp.obj')
        # tmp_file = '/tmp/tmp.obj'
        mesh.save_mesh(obj_file)

        mesh = trimesh.load(obj_file)
        mesh_invert = mesh.copy()
        mesh_invert.invert()
        mesh += mesh_invert
        mesh.merge_vertices()
        if mesh.vertices.shape[0] > 1 and mesh.faces.shape[0] > 1:
            create_urdf(mesh, obj_dir, init_texture=True)
        # os.makedirs(os.path.join(args.savedir, 'images'), exist_ok=True)
        # mesh.save_image(os.path.join(
        #     args.savedir, 'images', args.prefix + '_{:05}'.format(obj_id)),
        #     resolution=(640, 640))
        # 'images/tmp/mesh.png', resolution=(640, 640))

        print(obj_id)
        obj_id += 1
