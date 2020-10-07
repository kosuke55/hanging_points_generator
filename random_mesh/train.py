# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import json
import os

import numpy as np
import torch
import torch.optim as optim
from torch.utils.data import DataLoader
from tqdm import tqdm


from utils import calculate_gradient_penalty
from architectures import Generator, Discriminator

import kaolin as kal

import ipdb

all_category = ['table', 'monitor', 'phone',
                'watercraft', 'chair', 'lamp',
                'speaker', 'bench', 'plane',
                'bathtub', 'bookcase', 'bag',
                'basket', 'bowl', 'bus',
                'cabinet', 'camera', 'car',
                'dishwasher', 'file', 'knife',
                'laptop', 'mailbox', 'microwave',
                'piano', 'pillow', 'pistol',
                'printer', 'rocket', 'sofa',
                'washer', 'rifle', 'can', 'mug']


all_category = [
    '019_pitcher_base',
    '025_mug',
    '035_power_drill',
    '048_hammer',
    '051_large_clamp',
    '022_windex_bottle',
    '033_spatula',
    '042_adjustable_wrench',
    '050_medium_clamp',
    '052_extra_large_clamp'
]

parser = argparse.ArgumentParser()
parser.add_argument('--modelnet-root', type=str, help='Root directory of the ModelNet dataset.')
# parser.add_argument('--cache-dir', type=str, default='/media/kosuke/SANDISK/ShapeNetCore.v2/kaolin_cache',
#                     help='Path to write intermediate representation to.')
parser.add_argument('--cache-dir', type=str, default='/media/kosuke/SANDISK/meshdata/ycb_hanging_object/kaolin_cache',
                    help='Path to write intermediate representation to.')
parser.add_argument('--expid', type=str, default='3D_IWGAN', help='Unique experiment identifier.')
parser.add_argument('--device', type=str, default='cuda', help='Device to use')
parser.add_argument('--categories', type=str, nargs='+', default=all_category, help='list of object classes to use')
parser.add_argument('--epochs', type=int, default=50000, help='Number of train epochs.')
parser.add_argument('--batchsize', type=int, default=50, help='Batch size.')
parser.add_argument('--print-every', type=int, default=2, help='Print frequency (batches).')
parser.add_argument('--logdir', type=str, default='log', help='Directory to log data to.')
parser.add_argument('--resume', action='store_true', help='Resume training from last checkpoint.')

parser.add_argument('--shapenet-root', type=str,
                    default='/media/kosuke/SANDISK/meshdata/ShapeNetCore.v2',
                    help='Root directory of the shapenet dataset.')

parser.add_argument('--ycb-root', type=str,
                    default='/media/kosuke/SANDISK/meshdata/ycb_hanging_object/urdf',
                    help='Root directory of the ycb dataset.')

args = parser.parse_args()


# Setup Dataloader
# train_set = kal.datasets.modelnet.ModelNetVoxels(
#     basedir=args.modelnet_root, cache_dir=args.cache_dir,
#     categories=args.categories, resolutions=[30])
# train_set = kal.datasets.shapenet.ShapeNet_Voxels(
#     root=args.shapenet_root, cache_dir=args.cache_dir,
#     categories=args.categories, resolutions=[30],
#     voxel_range=1.)
train_set = kal.datasets.ycb_Voxels(
    root=args.ycb_root, cache_dir=args.cache_dir,
    categories=args.categories, resolutions=[30],
    voxel_range=1.)
dataloader_train = DataLoader(
    train_set, batch_size=args.batchsize, shuffle=True, num_workers=8)

# Setup Models
gen = Generator().to(args.device)
dis = Discriminator().to(args.device)


optim_g = optim.Adam(gen.parameters(), lr=.0001, betas=(0.5, 0.9))
optim_d = optim.Adam(dis.parameters(), lr=.0001, betas=(0.5, 0.9))

# Create log directory, if it doesn't already exist
logdir = os.path.join(args.logdir, args.expid)
if not os.path.isdir(logdir):
    os.makedirs(logdir)
    print('Created dir:', logdir)

# Log all commandline args
with open(os.path.join(logdir, 'args.txt'), 'w') as f:
    json.dump(args.__dict__, f, indent=2)


class Engine(object):
    """Engine that runs training and inference.
    Args
        - cur_epoch (int): Current epoch.
        - print_every (int): How frequently (# batches) to print loss.
        - validate_every (int): How frequently (# epochs) to run validation.
    """

    def __init__(self, print_every=1, resume=False):
        self.cur_epoch = 0
        self.train_loss = []
        self.val_loss = []
        self.bestval = 0
        self.print_every = print_every
        self.count = 0

        if resume:
            self.load()

    def train(self):
        loss_epoch = 0.
        num_batches = 0
        train_dis = True
        gen.train()
        dis.train()

        # Train loop
        for i, sample in tqdm(enumerate(dataloader_train), total=len(
                dataloader_train), desc='epoch={}'.format(self.cur_epoch)):

            voxels = sample['data']['30']
            optim_g.zero_grad(), gen.zero_grad()
            optim_d.zero_grad(), dis.zero_grad()

            # data creation
            real_voxels = torch.zeros(
                voxels.shape[0], 32, 32, 32).to(args.device)
            real_voxels[:, 1:-1, 1:-1, 1:-1] = voxels.to(args.device)

            z = torch.normal(
                torch.zeros(voxels.shape[0], 200),
                torch.ones(voxels.shape[0], 200)).to(args.device)

            fake_voxels = gen(z)
            d_on_fake = torch.mean(dis(fake_voxels))
            d_on_real = torch.mean(dis(real_voxels))
            gp_loss = 10 * calculate_gradient_penalty(
                dis, real_voxels.data, fake_voxels.data)
            d_loss = -d_on_real + d_on_fake + gp_loss

            # if i % 5 == 0:
            #     g_loss = -d_on_fake
            #     g_loss.backward()
            #     optim_g.step()
            # else:
            #     d_loss.backward()
            #     optim_d.step()

            if self.count == 0:
                g_loss = -d_on_fake
                g_loss.backward()
                optim_g.step()
                self.count += 1
            else:
                d_loss.backward()
                optim_d.step()
                self.count += 1

            # logging
            num_batches += 1
            # if i % args.print_every == 0:
            if self.count == 0:
                message = f'[TRAIN] Epoch {self.cur_epoch:03d}, Batch {i:03d}: gen: {float(g_loss.item()):2.3f}'
                message += f' dis = {float(d_loss.item()):2.3f}, gp = {float(gp_loss.item()):2.3f}'
                tqdm.write(message)
            if self.count == 5:
                self.count = 0

        self.train_loss.append(loss_epoch)
        self.cur_epoch += 1

    def load(self):
        gen.load_state_dict(torch.load(os.path.join(logdir, 'gen.pth')))
        dis.load_state_dict(torch.load(os.path.join(logdir, 'dis.pth')))
        optim_g.load_state_dict(torch.load(os.path.join(logdir, 'optim_g.pth')))
        optim_d.load_state_dict(torch.load(os.path.join(logdir, 'optim_d.pth')))
        # Read data corresponding to the loaded model
        with open(os.path.join(logdir, 'recent.log'), 'r') as f:
            run_data = json.load(f)
        self.cur_epoch = run_data['epoch']

    def save(self, epoch):
        # Create a dictionary of all data to save
        log_table = {
            'epoch': self.cur_epoch
        }

        # Save the recent model/optimizer states
        torch.save(
            gen.state_dict(), os.path.join(logdir, 'gen_{}.pth'.format(epoch)))
        torch.save(
            dis.state_dict(), os.path.join(logdir, 'dis_{}.pth'.format(epoch)))
        torch.save(
            optim_g.state_dict(), os.path.join(logdir, 'optim_g_{}.pth'.format(epoch)))
        torch.save(
            optim_d.state_dict(), os.path.join(logdir, 'optim_d_{}.pth'.format(epoch)))
        # Log other data corresponding to the recent model
        with open(os.path.join(logdir, 'recent.log'), 'w') as f:
            f.write(json.dumps(log_table))

        tqdm.write('====== Saved recent model ======>')


trainer = Engine(print_every=args.print_every, resume=args.resume)

for epoch in range(args.epochs):
    trainer.train()
    # if epoch % 5 == 4:
    if np.mod(epoch, 1000) == 0:
        trainer.save(epoch)
