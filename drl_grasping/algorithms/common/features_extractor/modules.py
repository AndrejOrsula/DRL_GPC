import ocnn
import torch


class OctreeConvRelu(torch.nn.Module):
    def __init__(self, depth, channel_in, channel_out, kernel_size=[3], stride=1):
        super(OctreeConvRelu, self).__init__()
        self.conv = ocnn.OctreeConv(depth,
                                    channel_in,
                                    channel_out,
                                    kernel_size,
                                    stride)
        self.relu = torch.nn.ReLU(inplace=True)

    def forward(self, data_in, octree):
        out = self.conv(data_in, octree)
        out = self.relu(out)
        return out


class OctreeConvBnRelu(torch.nn.Module):
    def __init__(self, depth, channel_in, channel_out, kernel_size=[3], stride=1, bn_eps=0.00001, bn_momentum=0.01):
        super(OctreeConvBnRelu, self).__init__()
        self.conv = ocnn.OctreeConv(depth,
                                    channel_in,
                                    channel_out,
                                    kernel_size,
                                    stride)
        self.bn = torch.nn.BatchNorm2d(channel_out,
                                       bn_eps,
                                       bn_momentum)
        self.relu = torch.nn.ReLU(inplace=True)

    def forward(self, data_in, octree):
        out = self.conv(data_in, octree)
        out = self.bn(out)
        out = self.relu(out)
        return out


class OctreeConvFastRelu(torch.nn.Module):
    def __init__(self, depth, channel_in, channel_out, kernel_size=[3], stride=1):
        super(OctreeConvFastRelu, self).__init__()
        self.conv = ocnn.OctreeConvFast(depth,
                                        channel_in,
                                        channel_out,
                                        kernel_size,
                                        stride)
        self.relu = torch.nn.ReLU(inplace=True)

    def forward(self, data_in, octree):
        out = self.conv(data_in, octree)
        out = self.relu(out)
        return out


class OctreeConvFastBnRelu(torch.nn.Module):
    def __init__(self, depth, channel_in, channel_out, kernel_size=[3], stride=1, bn_eps=0.00001, bn_momentum=0.01):
        super(OctreeConvFastBnRelu, self).__init__()
        self.conv = ocnn.OctreeConvFast(depth,
                                        channel_in,
                                        channel_out,
                                        kernel_size,
                                        stride)
        self.bn = torch.nn.BatchNorm2d(channel_out,
                                       bn_eps,
                                       bn_momentum)
        self.relu = torch.nn.ReLU(inplace=True)

    def forward(self, data_in, octree):
        out = self.conv(data_in, octree)
        out = self.bn(out)
        out = self.relu(out)
        return out


class OctreeConv1x1Relu(torch.nn.Module):
    def __init__(self, channel_in, channel_out, use_bias=True):
        super(OctreeConv1x1Relu, self).__init__()
        self.conv1x1 = ocnn.OctreeConv1x1(channel_in, channel_out, use_bias)
        self.relu = torch.nn.ReLU(inplace=True)

    def forward(self, data_in):
        out = self.conv1x1(data_in)
        out = self.relu(out)
        return out


class OctreeConv1x1BnRelu(torch.nn.Module):
    def __init__(self, channel_in, channel_out, use_bias=True, bn_eps=0.00001, bn_momentum=0.01):
        super(OctreeConv1x1BnRelu, self).__init__()
        self.conv1x1 = ocnn.OctreeConv1x1(channel_in, channel_out, use_bias)
        self.bn = torch.nn.BatchNorm2d(channel_out, bn_eps, bn_momentum)
        self.relu = torch.nn.ReLU(inplace=True)

    def forward(self, data_in):
        out = self.conv1x1(data_in)
        out = self.bn(out)
        out = self.relu(out)
        return out


class LinearRelu(torch.nn.Module):
    def __init__(self, channel_in, channel_out, use_bias=True):
        super(LinearRelu, self).__init__()
        self.fc = torch.nn.Linear(channel_in, channel_out, use_bias)
        self.relu = torch.nn.ReLU(inplace=True)

    def forward(self, data_in):
        out = self.fc(data_in)
        out = self.relu(out)
        return out


class LinearBnRelu(torch.nn.Module):
    def __init__(self, channel_in, channel_out, use_bias=True, bn_eps=0.00001, bn_momentum=0.01):
        super(LinearBnRelu, self).__init__()
        self.fc = torch.nn.Linear(channel_in, channel_out, use_bias)
        self.bn = torch.nn.BatchNorm1d(channel_out, bn_eps, bn_momentum)
        self.relu = torch.nn.ReLU(inplace=True)

    def forward(self, data_in):
        out = self.fc(data_in)
        out = self.bn(out)
        out = self.relu(out)
        return out

class ImageConvRelu(torch.nn.Module):
    def __init__(self, channel_in, channel_out, kernel_size=3, stride=1, padding=1):
        super(ImageConvRelu, self).__init__()
        self.conv = torch.nn.Conv2d(channel_in, channel_out, kernel_size, stride, padding)
        self.relu = torch.nn.ReLU(inplace=True)

    def forward(self, data_in):
        out = self.conv(data_in)
        out = self.relu(out)
        return out
