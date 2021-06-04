from drl_grasping.algorithms.common.features_extractor.modules import *
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
import gym
import torch
from torch import nn
import copy


class ImageCnnFeaturesExtractor(BaseFeaturesExtractor):
    """
    :param observation_space:
    :param channels_in: Number of input channels.
    :param channel_multiplier: Multiplier for the number of channels after each pooling.
                               With this parameter set to 1, the channels are [1, 2, 4, 8, ...] for [depth, depth-1, ..., full_depth].
    :param features_dim: Dimension of output feature vector. Note that this number is multiplied by the number of stacked inside one observation.
    """

    def __init__(self,
                 observation_space: gym.spaces.Box,
                 channels_in: int = 3,
                 width: int = 128,
                 height: int = 128,
                 channel_multiplier: int = 40,
                 full_depth_conv1d: bool = True,
                 full_depth_channels: int = 8,
                 features_dim: int = 96,
                 aux_obs_dim: int = 10,
                 max_pool_kernel: int = 4,
                 separate_networks_for_stacks: bool = True,
                 verbose: bool = False):

        self._channels_in = channels_in
        self._aux_obs_dim = aux_obs_dim
        self._separate_networks_for_stacks = separate_networks_for_stacks
        self._verbose = verbose

        # Determine number of stacked based on observation space shape
        self._n_stacks = observation_space.shape[0]
        # Chain up parent constructor
        super(ImageCnnFeaturesExtractor, self).__init__(observation_space,
                                                        self._n_stacks*(features_dim+aux_obs_dim))

        resolution = width*height
        flatten_dim = resolution//((max_pool_kernel**2)**2)*full_depth_channels

        if not self._separate_networks_for_stacks:
            raise("Shared network is not implemented for ImageCNN in this testing branch")

        self.convs1 = torch.nn.ModuleList(
            [ImageConvRelu(channels_in, channel_multiplier) for _ in range(self._n_stacks)])
        self.pools1 = torch.nn.ModuleList(
            [nn.MaxPool2d(max_pool_kernel) for _ in range(self._n_stacks)])
        self.convs2 = torch.nn.ModuleList([ImageConvRelu(
            channel_multiplier, 2*channel_multiplier) for _ in range(self._n_stacks)])
        self.pools2 = torch.nn.ModuleList(
            [nn.MaxPool2d(max_pool_kernel) for _ in range(self._n_stacks)])

        self.convs3 = torch.nn.ModuleList([ImageConvRelu(2*channel_multiplier, full_depth_channels,
                                                         kernel_size=1 if full_depth_conv1d else 3,
                                                         padding=0) for _ in range(self._n_stacks)])

        self.flatten = torch.nn.ModuleList(
            [torch.nn.Flatten() for _ in range(self._n_stacks)])

        # Last linear layer of the extractor, applied to all (flattened) voxels at full depth
        self.linear = torch.nn.ModuleList(
            [LinearRelu(flatten_dim, features_dim) for _ in range(self._n_stacks)])

        # One linear layer for auxiliary observations
        if self._aux_obs_dim != 0:
            self.aux_obs_linear = torch.nn.ModuleList([LinearRelu(self._aux_obs_dim,
                                                                  self._aux_obs_dim)
                                                       for _ in range(self._n_stacks)])

        number_of_learnable_parameters = sum(p.numel() for p in self.parameters()
                                             if p.requires_grad)
        print("Initialised ImageCnnFeaturesExtractor with "
              f"{number_of_learnable_parameters} parameters")
        if verbose:
            print(self)

    def forward(self, obs):

        aux_obs = obs['aux_obs']
        data = copy.deepcopy(obs['image'])

        for i in range(self._n_stacks):

            # Pass the data through all convolutional and polling layers
            data[i] = self.convs1[i](data[i])
            data[i] = self.pools1[i](data[i])
            data[i] = self.convs2[i](data[i])
            data[i] = self.pools2[i](data[i])
            data[i] = self.convs3[i](data[i])

            # Flatten into a feature vector
            data[i] = self.flatten[i](data[i])

            # Feed through the last linear layer
            data[i] = self.linear[i](data[i])

            if self._aux_obs_dim != 0:
                # Feed the data through linear layer
                aux_data = self.aux_obs_linear[i](aux_obs[:, i, :])
                # Concatenate auxiliary data
                data[i] = torch.cat((data[i], aux_data),
                                    dim=1)

        # Concatenate with other stacks
        data = torch.cat(data, dim=1)

        return data
