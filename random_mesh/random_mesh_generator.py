import argparse
import os
import os.path as osp

import cc3d
import numpy as np
import torch
import trimesh
import kaolin as kal

from architectures import Generator
from hanging_points_generator.create_mesh import create_urdf
from hanging_points_generator.generator_utils import save_json

parser = argparse.ArgumentParser()
parser.add_argument(
    '--savedir', type=str,
    default='random_shape',
    # default='/media/kosuke/SANDISK/meshdata/random_shape_shapenet_hanging_1016',
    # default='/media/kosuke/SANDISK/meshdata/random_shape_shapenet_pouring',
    help='Directory to save mesh to.')
parser.add_argument(
    '--pretrained_model', '-p',
    type=str, help='Pretrained models',
    default='../gan_trained_model/hanging_gen_3000.pth')
    # default='/media/kosuke55/SANDISK/random_mesh_gan/log_hanging/3D_IWGAN/gen_3000.pth')
    # default='/media/kosuke55/SANDISK/random_mesh_gan/log_pouring/3D_IWGAN/gen_3000.pth')
parser.add_argument(
    '--prefix', type=str,
    default='random',
    help='filename prefix')
parser.add_argument(
    '--device', type=str, default='cuda',
    help='Device to use.')
parser.add_argument(
    '--batchsize', type=int, default=512, help='Batch size.')
parser.add_argument(
    '--gui', '-g',
    action='store_true', help='Show mesh')
parser.add_argument(
    '--min-length',
    type=float,
    help='max lenth of generated mesh.'
    'The unit is m',
    default=0.1)
parser.add_argument(
    '--max-length',
    type=float,
    help='max lenth of generated mesh.'
    'The unit is m',
    default=0.15)
parser.add_argument(
    '--num', '-n',
    type=int,
    help='The number of objects you want to generate.',
    default=10000)
parser.add_argument(
    '-filling-rate', '-fr',
    type=int,
    help='Remain only objects with a filling rate under this value. ',
    default=0)
parser.add_argument(
    '--save-image', '-si',
    action='store_true', help='Save images of generated objects')

args = parser.parse_args()

gen = Generator().to(args.device)
gen.load_state_dict(torch.load(args.pretrained_model))
gen.eval()


min_length = args.min_length
max_length = args.max_length
required_num = args.num
filling_rate_thresh = args.filling_rate

filling_rate_dict = {}
obj_id = 0
while obj_id < required_num:
    z = torch.normal(
        torch.zeros(args.batchsize, 200),
        torch.ones(args.batchsize, 200)).to(args.device)
    fake_voxels = gen(z)

    for idx, model in enumerate(fake_voxels):
        model = model[:-2, :-2, :-2]

        model[torch.where(model >= 0.7)] = 1
        model[torch.where(model < 0.7)] = 0
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

        model = model.astype(np.float32)
        verts, faces = kal.conversions.voxelgrid_to_quadmesh(model)
        center = torch.mean(verts, dim=0)
        verts -= center
        max_xyz = torch.max(verts, axis=0).values
        min_xyz = torch.min(verts, axis=0).values
        length = torch.max(max_xyz - min_xyz, axis=0).values
        target_length \
            = np.random.rand() * (max_length - min_length) + min_length
        verts = verts * target_length / length

        mesh = kal.rep.QuadMesh.from_tensors(verts, faces)
        mesh.laplacian_smoothing(iterations=3)
        obj_dir = osp.join(
            args.savedir, args.prefix + '_{:05}'.format(obj_id))
        os.makedirs(obj_dir, exist_ok=True)
        obj_file = osp.join(obj_dir, 'tmp.obj')
        mesh.save_mesh(obj_file)

        mesh = trimesh.load(obj_file)

        if filling_rate_thresh > 0:
            voxel = mesh.voxelized(0.001)
            voxel.fill()
            filling_rate = voxel.volume / mesh.convex_hull.volume
            print(obj_id, filling_rate)
            if filling_rate > filling_rate_thresh:
                continue
            filling_rate_dict[obj_dir] = filling_rate

        mesh_invert = mesh.copy()
        mesh_invert.invert()
        mesh += mesh_invert
        mesh.merge_vertices()
        if mesh.vertices.shape[0] > 1 and mesh.faces.shape[0] > 1:
            print('save: {}'.format(obj_dir))
            create_urdf(mesh, obj_dir, init_texture=True)

        if args.save_image:
            os.makedirs(osp.join(args.savedir, 'images'), exist_ok=True)
            mesh.save_image(osp.join(
                args.savedir, 'images', args.prefix + '_{:05}'.format(obj_id)),
                resolution=(640, 640))

        if args.gui:
            mesh.show()
        obj_id += 1
        if obj_id >= required_num:
            break

if filling_rate_thresh > 0:
    save_json(osp.join(args.savedir, 'filling_rate.json'), filling_rate_dict)
