import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from copy import copy
from torch.nn.parameter import Parameter
from torchvision import models

# visionnet_input = True
# logits_input = False
# knob_pos_smoothening = False
class BasicBlock(nn.Module):
    expansion = 1

    def __init__(self, in_planes, planes, stride=1):
        super(BasicBlock, self).__init__()
        self.conv1 = nn.Conv2d(in_planes, planes, kernel_size=3, stride=stride, padding=1, bias=False)
        self.bn1 = nn.BatchNorm2d(planes)
        self.conv2 = nn.Conv2d(planes, planes, kernel_size=3, stride=1, padding=1, bias=False)
        self.bn2 = nn.BatchNorm2d(planes)

        self.shortcut = nn.Sequential()
        if stride != 1 or in_planes != self.expansion*planes:
            self.shortcut = nn.Sequential(
                nn.Conv2d(in_planes, self.expansion*planes, kernel_size=1, stride=stride, bias=False),
                nn.BatchNorm2d(self.expansion*planes)
            )

    def forward(self, x):
        out = F.relu(self.bn1(self.conv1(x)))
        out = self.bn2(self.conv2(out))
        out += self.shortcut(x)
        out = F.relu(out)
        return out

class Flatten(nn.Module):
    def forward(self, x):
        return x.view(x.size(0), -1)

class ReduceMean(nn.Module):
    def forward(self, x):
        return torch.mean(x, dim=1, keepdim=True)

class SpatialSoftmax(nn.Module):
    def __init__(self, height, width, channel, temperature=None, data_format='NCHW'):
        super(SpatialSoftmax, self).__init__()
        self.data_format = data_format
        self.height = height
        self.width = width
        self.channel = channel

        if temperature:
            self.temperature = Parameter(torch.ones(1)*temperature)
        else:
            self.temperature = 1.

        pos_x, pos_y = torch.meshgrid(torch.linspace(-1., 1., self.height),torch.linspace(-1., 1., self.width))

        pos_x = pos_x.reshape(self.height*self.width).float()
        pos_y = pos_y.reshape(self.height*self.width).float()

        self.register_buffer('pos_x', pos_x)
        self.register_buffer('pos_y', pos_y)

    def forward(self, feature):
        # Output:
        #   (N, C*2) x_0 y_0 ...
        if self.data_format == 'NHWC':
            feature = feature.transpose(1, 3).tranpose(2, 3).view(-1, self.height*self.width)
        else:
            feature = feature.view(-1, self.height*self.width)
        softmax_attention = F.softmax(feature/self.temperature, dim=-1)
        expected_x = torch.sum(self.pos_x*softmax_attention, dim=1, keepdim=True)
        expected_y = torch.sum(self.pos_y*softmax_attention, dim=1, keepdim=True)
        expected_xy = torch.cat([expected_x, expected_y], 1)
        feature_keypoints = expected_xy.view(-1, self.channel*2)
        return feature_keypoints


class VisionModel(nn.Module):
    def __init__(self, cam="front"):
        super().__init__()

        self.front_origin = torch.tensor([[-0.5, 1.0]]).cuda() #y,z
        self.top_origin = torch.tensor([[0.5, 0.0]]).cuda() #x,y

        # input is nx3x256x256
        self.backbone = nn.Sequential(
            BasicBlock(3, 64, stride=2),
            BasicBlock(64, 64, stride=2),
            BasicBlock(64, 64, stride=1),
        )

        self.rm = nn.Sequential(
            ReduceMean()
        )

        self.position_regressor = nn.Sequential(
            ReduceMean(),
            Flatten(),
            nn.Linear(4096, 32),
            nn.Tanh(),
            nn.Linear(32, 2),
        )

    def forward(self, x):
        backbone = self.backbone(x)
        heatmap_features = self.rm(backbone)        
        pos_regression = self.position_regressor(backbone)
        return pos_regression, heatmap_features

class VisionModelXYZ(nn.Module):
    def __init__(self, cam="xyz"):
        super().__init__()
        self.top_net = VisionModel()
        self.front_net = VisionModel()

    def forward(self, x, y):
        pp1, hm1 = self.top_net.forward(x)
        pp2, hm2 = self.front_net.forward(y)
        pp = torch.stack((pp1[:,0], (pp1[:,1]+pp2[:,0])/2., pp2[:,1]), 1)
        return pp, hm1, hm2

if __name__ == "__main__":
    vision = VisionModel()
    input_data = torch.zeros(5, 3, 256, 256)
    input_data[0, 0, 32, 32] = 1
    input_data[1, 0, 32, 32] = 1
    input_data[2, 0, 32, 32] = 1
    input_data[3, 0, 32, 32] = 1
    input_data[4, 0, 32, 32] = 1
    # print(vision.pos_from_logits(logit, cam="front"))

    model_parameters = filter(lambda p: p.requires_grad, vision.parameters())
    params = sum([np.prod(p.size()) for p in model_parameters])
    print("Model has {} parameters...".format(params))

    logits, features = vision(input_data)

    # print(logits.size(), features.size())

        # def global2label_fr(self, x):
    #     x = x - self.front_origin.data.cpu().numpy()
    #     x[:,0], x[:,1] = -x[:,1], x[:,0].copy()
    #     x = (x + 0.5) * 64
    #     x = np.int8(np.round(x))
    #     label = self.gkern(64, 64, (x[0,1],x[0,0]),s=1)
    #     return label

    # def global2label_top(self, x):
    #     x = x - self.top_origin.data.cpu().numpy()
    #     x[:,0], x[:,1] = -x[:,1], x[:,0].copy()
    #     x = (x + 0.5) * 64
    #     x = np.int8(np.round(x))
    #     label = self.gkern(64, 64, (x[0,1],x[0,0]),s=1)
    #     # label = np.zeros((64,64))
    #     # label[x[0,0],x[0,1]] = 10
    #     return label
