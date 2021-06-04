# This module adds a monkey patch to ReplayBuffer, such that octrees are
# directly supported and there is no extra RAM -> VRAM -> RAM overhead

# Note: needs to be included before `from stable_baselines3.common.buffers import ReplayBuffer` in the module that uses this

from stable_baselines3.common.buffers import ReplayBuffer

import numpy as np
import torch as th
from typing import Optional, Union, Dict
from stable_baselines3.common.type_aliases import ReplayBufferSamples
from stable_baselines3.common.vec_env import VecNormalize
from gym import spaces

import ocnn
from typing import Tuple


def preprocess_stacked_octree_batch(observation: th.Tensor, device, separate_batches: bool = True, is_image: bool = True, image_resolution: Tuple[float, float] = (128, 128)) -> Dict[str, th.Tensor]:
    # Note: Primordial magic is happening here,
    #       but there's no reason to tremble in fear.
    #       For your own good don't question it too much,
    #       it's just an optimised stacked octree batch...

    # print(np.ndarray(buffer=color_image.tobytes(), dtype='uint8', shape=(3*self._camera_height*self._camera_width,)).shape)
    # print(np.ndarray(buffer=depth_image.tobytes(), dtype='float32', shape=(self._camera_height*self._camera_width,)).shape)

    if is_image:
        if not separate_batches:
            raise(
                "ERROR: Only separate batches are currently supported for images on this temporary testing branch")

        image_batches = []

        number_of_pixels = image_resolution[0]*image_resolution[1]

        if observation.shape[2] >= 4*number_of_pixels:
            contains_depth = True
        else:
            contains_depth = False

        if contains_depth:
            for image_batch in np.split(observation, observation.shape[1], axis=1):

                images = []
                for image in image_batch:
                    # Convert to images, without aux obs
                    color_image, depth_image = np.split(
                        image[:, :-44], [3*number_of_pixels], axis=1)
                    color_image = np.ndarray(buffer=color_image.tobytes(
                    ), dtype=np.uint8, shape=(3*number_of_pixels,)).astype(np.float32)
                    depth_image = np.ndarray(buffer=depth_image.tobytes(
                    ), dtype=np.float32, shape=(number_of_pixels,))
                    # Normalize RGB image (depth is already normalized)
                    color_image /= 255

                    rgbd_image = np.insert(color_image,
                                           range(3, (3*number_of_pixels)+3, 3),
                                           depth_image).reshape(-1, image_resolution[0], image_resolution[1])

                    # Convert to tensor and append to image list
                    images.append(th.from_numpy(rgbd_image))

                # Make batch out of tensor (consisting of one stack)
                image_batches.append(th.stack(images).to(device))

            # Get number of auxiliary observations encoded as float32 and parse them
            n_aux_obs_f32 = int(np.frombuffer(buffer=observation[0, 0, -4:],
                                              dtype='uint32',
                                              count=1))
            aux_obs = th.from_numpy(
                np.frombuffer(buffer=observation[:, :, -(4*n_aux_obs_f32+8):-8].reshape(-1),
                              dtype='float32',
                              count=n_aux_obs_f32*observation.shape[0]*observation.shape[1]).reshape(observation.shape[:2] + (n_aux_obs_f32,)))

            return {'image': image_batches,
                    'aux_obs': aux_obs.to(device)}

        else:
            # RGB only

            for image_batch in np.split(observation, observation.shape[1], axis=1):

                images = []
                for image in image_batch:
                    # Convert to image, without aux obs
                    image_without_aux_obs = image[:, :-44].reshape(
                        -1, image_resolution[0], image_resolution[1]).astype(np.float32)
                    # Normalize
                    image_without_aux_obs /= 255
                    # Convert to tensor and append to image list
                    images.append(th.from_numpy(image_without_aux_obs))

                # Make batch out of tensor (consisting of one stack)
                image_batches.append(th.stack(images).to(device))

            # Get number of auxiliary observations encoded as float32 and parse them
            n_aux_obs_f32 = int(np.frombuffer(buffer=observation[0, 0, -4:],
                                              dtype='uint32',
                                              count=1))
            aux_obs = th.from_numpy(
                np.frombuffer(buffer=observation[:, :, -(4*n_aux_obs_f32+8):-8].reshape(-1),
                              dtype='float32',
                              count=n_aux_obs_f32*observation.shape[0]*observation.shape[1]).reshape(observation.shape[:2] + (n_aux_obs_f32,)))

            return {'image': image_batches,
                    'aux_obs': aux_obs.to(device)}

    if not separate_batches:
        octrees = []
        for octree in observation.reshape(-1, observation.shape[-1]):
            # Get original octree size
            octree_size = np.frombuffer(buffer=octree[-4:],
                                        dtype='uint32',
                                        count=1)
            # Convert to tensor and append to list
            octrees.append(th.from_numpy(octree[:octree_size[0]]))
        # Make batch out of tensor (consisting of n-stacked frames)
        octree_batch = ocnn.octree_batch(octrees)

        # Get number of auxiliary observations encoded as float32 and parse them
        n_aux_obs_f32 = int(np.frombuffer(buffer=observation[0, 0, -8:-4],
                                          dtype='uint32',
                                          count=1))
        aux_obs = th.from_numpy(
            np.frombuffer(buffer=observation[:, :, -(4*n_aux_obs_f32+8):-8].reshape(-1),
                          dtype='float32',
                          count=n_aux_obs_f32*observation.shape[0]*observation.shape[1]).reshape(observation.shape[:2] + (n_aux_obs_f32,)))

        return {'octree': octree_batch.to(device),
                'aux_obs': aux_obs.to(device)}

    else:
        octree_batches = []

        for octree_batch in np.split(observation, observation.shape[1], axis=1):
            octrees = []
            for octree in octree_batch:
                # Get original octree size
                octree_size = np.frombuffer(buffer=octree[-4:],
                                            dtype='uint32',
                                            count=1)
                # Convert to tensor and append to list
                octrees.append(th.from_numpy(octree[:octree_size[0]]))
            # Make batch out of tensor (consisting of one stack)
            octree_batches.append(ocnn.octree_batch(octrees).to(device))

        # Get number of auxiliary observations encoded as float32 and parse them
        n_aux_obs_f32 = int(np.frombuffer(buffer=observation[0, 0, -8:-4],
                                          dtype='uint32',
                                          count=1))
        aux_obs = th.from_numpy(
            np.frombuffer(buffer=observation[:, :, -(4*n_aux_obs_f32+8):-8].reshape(-1),
                          dtype='float32',
                          count=n_aux_obs_f32*observation.shape[0]*observation.shape[1]).reshape(observation.shape[:2] + (n_aux_obs_f32,)))

        return {'octree': octree_batches,
                'aux_obs': aux_obs.to(device)}


__old__init__ = ReplayBuffer.__init__
__old_get_samples__ = ReplayBuffer._get_samples


def __init___with_checking_for_octree(self,
                                      buffer_size: int,
                                      observation_space: spaces.Space,
                                      action_space: spaces.Space,
                                      device: Union[th.device, str] = "cpu",
                                      n_envs: int = 1,
                                      optimize_memory_usage: bool = False,
                                      separate_networks_for_stacks: bool = True,
                                      contains_image_obs: bool = True):
    __old__init__(self,
                  buffer_size=buffer_size,
                  observation_space=observation_space,
                  action_space=action_space,
                  device=device,
                  n_envs=n_envs,
                  optimize_memory_usage=optimize_memory_usage)

    # Determine if octrees are used
    # Note: This is not 100% reliable as there could be other observations that do the same (outside of this repo)
    self.contains_octree_obs = False
    if isinstance(observation_space, spaces.Box) and len(observation_space.shape) == 2:
        if np.uint8 == observation_space.dtype and \
            np.all(0 == observation_space.low) and \
                np.all(255 == observation_space.high):
            self.contains_octree_obs = True
            self._separate_networks_for_stacks = separate_networks_for_stacks
    self.contains_image_obs_instead_of_octree = contains_image_obs


def _get_samples_with_support_for_octree(self,
                                         batch_inds: np.ndarray,
                                         env: Optional[VecNormalize] = None) -> ReplayBufferSamples:

    if not self.contains_octree_obs:
        return __old_get_samples__(self, batch_inds=batch_inds, env=env)

    # Current observations
    obs = self.observations[batch_inds, 0, :]
    obs = preprocess_stacked_octree_batch(
        obs, self.device, separate_batches=self._separate_networks_for_stacks, is_image=self.contains_image_obs_instead_of_octree)

    # Next observations
    if self.optimize_memory_usage:
        next_obs = self.observations[(
            batch_inds + 1) % self.buffer_size, 0, :]
    else:
        next_obs = self.next_observations[batch_inds, 0, :]
    next_obs = preprocess_stacked_octree_batch(
        next_obs, self.device, separate_batches=self._separate_networks_for_stacks, is_image=self.contains_image_obs_instead_of_octree)

    return ReplayBufferSamples(
        observations=obs,
        actions=self.to_torch(self.actions[batch_inds, 0, :]),
        next_observations=next_obs,
        dones=self.to_torch(self.dones[batch_inds]),
        rewards=self.to_torch(self._normalize_reward(
            self.rewards[batch_inds], env)),
    )


ReplayBuffer.__init__ = __init___with_checking_for_octree
ReplayBuffer._get_samples = _get_samples_with_support_for_octree
