from drl_grasping.algorithms.common.features_extractor import OctreeCnnFeaturesExtractor, ImageCnnFeaturesExtractor
from drl_grasping.algorithms.common.octree_replay_buffer import preprocess_stacked_octree_batch
from stable_baselines3.common.policies import register_policy
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.utils import is_vectorized_observation
from stable_baselines3.common.vec_env.obs_dict_wrapper import ObsDictWrapper
from sb3_contrib.tqc.policies import Actor, Critic
from sb3_contrib.tqc.policies import TQCPolicy
from torch import nn
from typing import Any, Dict, List, Optional, Tuple, Type
import gym
import numpy as np
import torch as th

import ocnn


class ActorOctreeCnn(Actor):
    """
    Actor network (policy) for TQC.
    Overriden to not preprocess observations (unnecessary conversion into float)

    :param observation_space: Obervation space
    :param action_space: Action space
    :param net_arch: Network architecture
    :param features_extractor: Network to extract features
        (a CNN when using images, a nn.Flatten() layer otherwise)
    :param features_dim: Number of features
    :param activation_fn: Activation function
    :param use_sde: Whether to use State Dependent Exploration or not
    :param log_std_init: Initial value for the log standard deviation
    :param full_std: Whether to use (n_features x n_actions) parameters
        for the std instead of only (n_features,) when using gSDE.
    :param sde_net_arch: Network architecture for extracting features
        when using gSDE. If None, the latent features from the policy will be used.
        Pass an empty list to use the states as features.
    :param use_expln: Use ``expln()`` function instead of ``exp()`` when using gSDE to ensure
        a positive standard deviation (cf paper). It allows to keep variance
        above zero and prevent it from growing too fast. In practice, ``exp()`` is usually enough.
    :param clip_mean: Clip the mean output when using gSDE to avoid numerical instability.
    :param normalize_images: Whether to normalize images or not,
         dividing by 255.0 (True by default)
    """

    def __init__(
        self,
        observation_space: gym.spaces.Space,
        action_space: gym.spaces.Space,
        net_arch: List[int],
        features_extractor: nn.Module,
        features_dim: int,
        activation_fn: Type[nn.Module] = nn.ReLU,
        use_sde: bool = False,
        log_std_init: float = -3,
        full_std: bool = True,
        sde_net_arch: Optional[List[int]] = None,
        use_expln: bool = False,
        clip_mean: float = 2.0,
        normalize_images: bool = True,
    ):
        super(ActorOctreeCnn, self).__init__(
            observation_space=observation_space,
            action_space=action_space,
            net_arch=net_arch,
            features_extractor=features_extractor,
            features_dim=features_dim,
            activation_fn=activation_fn,
            use_sde=use_sde,
            log_std_init=log_std_init,
            full_std=full_std,
            sde_net_arch=sde_net_arch,
            use_expln=use_expln,
            clip_mean=clip_mean,
            normalize_images=normalize_images)

    def extract_features(self, obs: th.Tensor) -> th.Tensor:
        """
        Preprocess the observation if needed and extract features.
        Overriden to skip pre-processing (for some reason it converts tensor to Float)

        :param obs:
        :return:
        """
        assert self.features_extractor is not None, "No features extractor was set"
        return self.features_extractor(obs)
        # OVERRIDDEN
        # preprocessed_obs = preprocess_obs(obs, self.observation_space, normalize_images=self.normalize_images)
        # return self.features_extractor(preprocessed_obs)


class CriticOctreeCnn(Critic):
    """
    Critic network (q-value function) for TQC.

    :param observation_space: Obervation space
    :param action_space: Action space
    :param net_arch: Network architecture
    :param features_extractor: Network to extract features
        (a CNN when using images, a nn.Flatten() layer otherwise)
    :param features_dim: Number of features
    :param activation_fn: Activation function
    :param normalize_images: Whether to normalize images or not,
         dividing by 255.0 (True by default)
    :param share_features_extractor: Whether the features extractor is shared or not
        between the actor and the critic (this saves computation time)
    """

    def __init__(
        self,
        observation_space: gym.spaces.Space,
        action_space: gym.spaces.Space,
        net_arch: List[int],
        features_extractor: nn.Module,
        features_dim: int,
        activation_fn: Type[nn.Module] = nn.ReLU,
        normalize_images: bool = True,
        n_quantiles: int = 25,
        n_critics: int = 2,
        share_features_extractor: bool = True,
    ):
        super(CriticOctreeCnn, self).__init__(
            observation_space=observation_space,
            action_space=action_space,
            net_arch=net_arch,
            features_extractor=features_extractor,
            features_dim=features_dim,
            activation_fn=activation_fn,
            normalize_images=normalize_images,
            n_quantiles=n_quantiles,
            n_critics=n_critics,
            share_features_extractor=share_features_extractor)

    def extract_features(self, obs: th.Tensor) -> th.Tensor:
        """
        Preprocess the observation if needed and extract features.
        Overriden to skip pre-processing (for some reason it converts tensor to Float)

        :param obs:
        :return:
        """
        assert self.features_extractor is not None, "No features extractor was set"
        return self.features_extractor(obs)
        # OVERRIDDEN
        # preprocessed_obs = preprocess_obs(obs, self.observation_space, normalize_images=self.normalize_images)
        # return self.features_extractor(preprocessed_obs)


class OctreeCnnPolicy(TQCPolicy):
    """
    Policy class (with both actor and critic) for TQC.
    Overriden to not preprocess observations (unnecessary conversion into float)

    :param observation_space: Observation space
    :param action_space: Action space
    :param lr_schedule: Learning rate schedule (could be constant)
    :param net_arch: The specification of the policy and value networks.
    :param activation_fn: Activation function
    :param use_sde: Whether to use State Dependent Exploration or not
    :param log_std_init: Initial value for the log standard deviation
    :param sde_net_arch: Network architecture for extracting features
        when using gSDE. If None, the latent features from the policy will be used.
        Pass an empty list to use the states as features.
    :param use_expln: Use ``expln()`` function instead of ``exp()`` when using gSDE to ensure
        a positive standard deviation (cf paper). It allows to keep variance
        above zero and prevent it from growing too fast. In practice, ``exp()`` is usually enough.
    :param clip_mean: Clip the mean output when using gSDE to avoid numerical instability.
    :param features_extractor_class: Features extractor to use.
    :param features_extractor_kwargs: Keyword arguments
        to pass to the feature extractor.
    :param normalize_images: Whether to normalize images or not,
         dividing by 255.0 (True by default)
    :param optimizer_class: The optimizer to use,
        ``th.optim.Adam`` by default
    :param optimizer_kwargs: Additional keyword arguments,
        excluding the learning rate, to pass to the optimizer
    :param share_features_extractor: Whether to share or not the features extractor
        between the actor and the critic (this saves computation time)
    """

    def __init__(
        self,
        observation_space: gym.spaces.Space,
        action_space: gym.spaces.Space,
        lr_schedule,
        net_arch: Optional[List[int]] = None,
        activation_fn: Type[nn.Module] = nn.ReLU,
        use_sde: bool = False,
        log_std_init: float = -3,
        sde_net_arch: Optional[List[int]] = None,
        use_expln: bool = False,
        clip_mean: float = 2.0,
        features_extractor_class: Type[BaseFeaturesExtractor] = OctreeCnnFeaturesExtractor,
        features_extractor_kwargs: Optional[Dict[str, Any]] = None,
        normalize_images: bool = True,
        optimizer_class: Type[th.optim.Optimizer] = th.optim.Adam,
        optimizer_kwargs: Optional[Dict[str, Any]] = None,
        n_quantiles: int = 25,
        n_critics: int = 2,
        share_features_extractor: bool = True,
        separate_networks_for_stacks: bool = True,
        debug_write_octree: bool = False,
    ):
        features_extractor_kwargs.update(
            {'separate_networks_for_stacks': separate_networks_for_stacks})
        super(OctreeCnnPolicy, self).__init__(
            observation_space=observation_space,
            action_space=action_space,
            lr_schedule=lr_schedule,
            net_arch=net_arch,
            activation_fn=activation_fn,
            use_sde=use_sde,
            log_std_init=log_std_init,
            sde_net_arch=sde_net_arch,
            use_expln=use_expln,
            clip_mean=clip_mean,
            features_extractor_class=features_extractor_class,
            features_extractor_kwargs=features_extractor_kwargs,
            normalize_images=normalize_images,
            optimizer_class=optimizer_class,
            optimizer_kwargs=optimizer_kwargs,
            n_quantiles=n_quantiles,
            n_critics=n_critics,
            share_features_extractor=share_features_extractor)

        self._separate_networks_for_stacks = separate_networks_for_stacks
        self._debug_write_octree = debug_write_octree

    def make_actor(self, features_extractor: Optional[BaseFeaturesExtractor] = None) -> Actor:
        actor_kwargs = self._update_features_extractor(
            self.actor_kwargs, features_extractor)
        return ActorOctreeCnn(**actor_kwargs).to(self.device)

    def make_critic(self, features_extractor: Optional[BaseFeaturesExtractor] = None) -> Critic:
        critic_kwargs = self._update_features_extractor(
            self.critic_kwargs, features_extractor)
        return CriticOctreeCnn(**critic_kwargs).to(self.device)

    def predict(
        self,
        observation: np.ndarray,
        state: Optional[np.ndarray] = None,
        mask: Optional[np.ndarray] = None,
        deterministic: bool = False,
    ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        Overriden to create proper Octree batch.
        Get the policy action and state from an observation (and optional state).

        :param observation: the input observation
        :param state: The last states (can be None, used in recurrent policies)
        :param mask: The last masks (can be None, used in recurrent policies)
        :param deterministic: Whether or not to return deterministic actions.
        :return: the model's action and the next state
            (used in recurrent policies)
        """
        if isinstance(observation, dict):
            observation = ObsDictWrapper.convert_dict(observation)
        else:
            observation = np.array(observation)

        vectorized_env = is_vectorized_observation(
            observation, self.observation_space)

        if self._debug_write_octree:
            ocnn.write_octree(th.from_numpy(observation[-1]), 'octree.octree')

        # Make batch out of tensor (consisting of n-stacked octrees)
        octree_batch = preprocess_stacked_octree_batch(
            observation, self.device, separate_batches=self._separate_networks_for_stacks)

        with th.no_grad():
            actions = self._predict(octree_batch,
                                    deterministic=deterministic)
        # Convert to numpy
        actions = actions.cpu().numpy()

        if isinstance(self.action_space, gym.spaces.Box):
            if self.squash_output:
                # Rescale to proper domain when using squashing
                actions = self.unscale_action(actions)
            else:
                # Actions could be on arbitrary scale, so clip the actions to avoid
                # out of bound error (e.g. if sampling from a Gaussian distribution)
                actions = np.clip(
                    actions, self.action_space.low, self.action_space.high)

        if not vectorized_env:
            if state is not None:
                raise ValueError(
                    "Error: The environment must be vectorized when using recurrent policies.")
            actions = actions[0]

        return actions, state


register_policy("OctreeCnnPolicy", OctreeCnnPolicy)


class ActorImageCnn(Actor):
    """
    Actor network (policy) for TQC.
    Overriden to not preprocess observations (unnecessary conversion into float)

    :param observation_space: Obervation space
    :param action_space: Action space
    :param net_arch: Network architecture
    :param features_extractor: Network to extract features
        (a CNN when using images, a nn.Flatten() layer otherwise)
    :param features_dim: Number of features
    :param activation_fn: Activation function
    :param use_sde: Whether to use State Dependent Exploration or not
    :param log_std_init: Initial value for the log standard deviation
    :param full_std: Whether to use (n_features x n_actions) parameters
        for the std instead of only (n_features,) when using gSDE.
    :param sde_net_arch: Network architecture for extracting features
        when using gSDE. If None, the latent features from the policy will be used.
        Pass an empty list to use the states as features.
    :param use_expln: Use ``expln()`` function instead of ``exp()`` when using gSDE to ensure
        a positive standard deviation (cf paper). It allows to keep variance
        above zero and prevent it from growing too fast. In practice, ``exp()`` is usually enough.
    :param clip_mean: Clip the mean output when using gSDE to avoid numerical instability.
    :param normalize_images: Whether to normalize images or not,
         dividing by 255.0 (True by default)
    """

    def __init__(
        self,
        observation_space: gym.spaces.Space,
        action_space: gym.spaces.Space,
        net_arch: List[int],
        features_extractor: nn.Module,
        features_dim: int,
        activation_fn: Type[nn.Module] = nn.ReLU,
        use_sde: bool = False,
        log_std_init: float = -3,
        full_std: bool = True,
        sde_net_arch: Optional[List[int]] = None,
        use_expln: bool = False,
        clip_mean: float = 2.0,
        normalize_images: bool = False,
    ):
        super(ActorImageCnn, self).__init__(
            observation_space=observation_space,
            action_space=action_space,
            net_arch=net_arch,
            features_extractor=features_extractor,
            features_dim=features_dim,
            activation_fn=activation_fn,
            use_sde=use_sde,
            log_std_init=log_std_init,
            full_std=full_std,
            sde_net_arch=sde_net_arch,
            use_expln=use_expln,
            clip_mean=clip_mean,
            normalize_images=normalize_images)

    def extract_features(self, obs: th.Tensor) -> th.Tensor:
        """
        Preprocess the observation if needed and extract features.
        Overriden to skip pre-processing (for some reason it converts tensor to Float)

        :param obs:
        :return:
        """
        assert self.features_extractor is not None, "No features extractor was set"
        return self.features_extractor(obs)
        # OVERRIDDEN
        # preprocessed_obs = preprocess_obs(obs, self.observation_space, normalize_images=self.normalize_images)
        # return self.features_extractor(preprocessed_obs)


class CriticImageCnn(Critic):
    """
    Critic network (q-value function) for TQC.

    :param observation_space: Obervation space
    :param action_space: Action space
    :param net_arch: Network architecture
    :param features_extractor: Network to extract features
        (a CNN when using images, a nn.Flatten() layer otherwise)
    :param features_dim: Number of features
    :param activation_fn: Activation function
    :param normalize_images: Whether to normalize images or not,
         dividing by 255.0 (True by default)
    :param share_features_extractor: Whether the features extractor is shared or not
        between the actor and the critic (this saves computation time)
    """

    def __init__(
        self,
        observation_space: gym.spaces.Space,
        action_space: gym.spaces.Space,
        net_arch: List[int],
        features_extractor: nn.Module,
        features_dim: int,
        activation_fn: Type[nn.Module] = nn.ReLU,
        normalize_images: bool = False,
        n_quantiles: int = 25,
        n_critics: int = 2,
        share_features_extractor: bool = True,
    ):
        super(CriticImageCnn, self).__init__(
            observation_space=observation_space,
            action_space=action_space,
            net_arch=net_arch,
            features_extractor=features_extractor,
            features_dim=features_dim,
            activation_fn=activation_fn,
            normalize_images=normalize_images,
            n_quantiles=n_quantiles,
            n_critics=n_critics,
            share_features_extractor=share_features_extractor)

    def extract_features(self, obs: th.Tensor) -> th.Tensor:
        """
        Preprocess the observation if needed and extract features.
        Overriden to skip pre-processing (for some reason it converts tensor to Float)

        :param obs:
        :return:
        """

        assert self.features_extractor is not None, "No features extractor was set"
        return self.features_extractor(obs)
        # OVERRIDDEN
        # preprocessed_obs = preprocess_obs(obs, self.observation_space, normalize_images=self.normalize_images)
        # return self.features_extractor(preprocessed_obs)


class ImageCnnPolicy(TQCPolicy):
    """
    Policy class (with both actor and critic) for TQC.
    Overriden to not preprocess observations (unnecessary conversion into float)

    :param observation_space: Observation space
    :param action_space: Action space
    :param lr_schedule: Learning rate schedule (could be constant)
    :param net_arch: The specification of the policy and value networks.
    :param activation_fn: Activation function
    :param use_sde: Whether to use State Dependent Exploration or not
    :param log_std_init: Initial value for the log standard deviation
    :param sde_net_arch: Network architecture for extracting features
        when using gSDE. If None, the latent features from the policy will be used.
        Pass an empty list to use the states as features.
    :param use_expln: Use ``expln()`` function instead of ``exp()`` when using gSDE to ensure
        a positive standard deviation (cf paper). It allows to keep variance
        above zero and prevent it from growing too fast. In practice, ``exp()`` is usually enough.
    :param clip_mean: Clip the mean output when using gSDE to avoid numerical instability.
    :param features_extractor_class: Features extractor to use.
    :param features_extractor_kwargs: Keyword arguments
        to pass to the feature extractor.
    :param normalize_images: Whether to normalize images or not,
         dividing by 255.0 (True by default)
    :param optimizer_class: The optimizer to use,
        ``th.optim.Adam`` by default
    :param optimizer_kwargs: Additional keyword arguments,
        excluding the learning rate, to pass to the optimizer
    :param share_features_extractor: Whether to share or not the features extractor
        between the actor and the critic (this saves computation time)
    """

    def __init__(
        self,
        observation_space: gym.spaces.Space,
        action_space: gym.spaces.Space,
        lr_schedule,
        net_arch: Optional[List[int]] = None,
        activation_fn: Type[nn.Module] = nn.ReLU,
        use_sde: bool = False,
        log_std_init: float = -3,
        sde_net_arch: Optional[List[int]] = None,
        use_expln: bool = False,
        clip_mean: float = 2.0,
        features_extractor_class: Type[BaseFeaturesExtractor] = ImageCnnFeaturesExtractor,
        features_extractor_kwargs: Optional[Dict[str, Any]] = None,
        normalize_images: bool = False,
        optimizer_class: Type[th.optim.Optimizer] = th.optim.Adam,
        optimizer_kwargs: Optional[Dict[str, Any]] = None,
        n_quantiles: int = 25,
        n_critics: int = 2,
        share_features_extractor: bool = True,
        separate_networks_for_stacks: bool = True,
    ):
        features_extractor_kwargs.update(
            {'separate_networks_for_stacks': separate_networks_for_stacks})
        super(ImageCnnPolicy, self).__init__(
            observation_space=observation_space,
            action_space=action_space,
            lr_schedule=lr_schedule,
            net_arch=net_arch,
            activation_fn=activation_fn,
            use_sde=use_sde,
            log_std_init=log_std_init,
            sde_net_arch=sde_net_arch,
            use_expln=use_expln,
            clip_mean=clip_mean,
            features_extractor_class=features_extractor_class,
            features_extractor_kwargs=features_extractor_kwargs,
            normalize_images=normalize_images,
            optimizer_class=optimizer_class,
            optimizer_kwargs=optimizer_kwargs,
            n_quantiles=n_quantiles,
            n_critics=n_critics,
            share_features_extractor=share_features_extractor)

        self._separate_networks_for_stacks = separate_networks_for_stacks

    def make_actor(self, features_extractor: Optional[BaseFeaturesExtractor] = None) -> Actor:
        actor_kwargs = self._update_features_extractor(
            self.actor_kwargs, features_extractor)
        return ActorImageCnn(**actor_kwargs).to(self.device)

    def make_critic(self, features_extractor: Optional[BaseFeaturesExtractor] = None) -> Critic:
        critic_kwargs = self._update_features_extractor(
            self.critic_kwargs, features_extractor)
        return CriticImageCnn(**critic_kwargs).to(self.device)

    def predict(
        self,
        observation: np.ndarray,
        state: Optional[np.ndarray] = None,
        mask: Optional[np.ndarray] = None,
        deterministic: bool = False,
    ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        Overriden to create proper Octree batch.
        Get the policy action and state from an observation (and optional state).

        :param observation: the input observation
        :param state: The last states (can be None, used in recurrent policies)
        :param mask: The last masks (can be None, used in recurrent policies)
        :param deterministic: Whether or not to return deterministic actions.
        :return: the model's action and the next state
            (used in recurrent policies)
        """
        if isinstance(observation, dict):
            observation = ObsDictWrapper.convert_dict(observation)
        else:
            observation = np.array(observation)

        vectorized_env = is_vectorized_observation(
            observation, self.observation_space)

        # Make batch out of tensor (consisting of n-stacked octrees)
        image_batch = preprocess_stacked_octree_batch(
            observation, self.device, separate_batches=self._separate_networks_for_stacks, is_image=True)

        with th.no_grad():
            actions = self._predict(image_batch,
                                    deterministic=deterministic)
        # Convert to numpy
        actions = actions.cpu().numpy()

        if isinstance(self.action_space, gym.spaces.Box):
            if self.squash_output:
                # Rescale to proper domain when using squashing
                actions = self.unscale_action(actions)
            else:
                # Actions could be on arbitrary scale, so clip the actions to avoid
                # out of bound error (e.g. if sampling from a Gaussian distribution)
                actions = np.clip(
                    actions, self.action_space.low, self.action_space.high)

        if not vectorized_env:
            if state is not None:
                raise ValueError(
                    "Error: The environment must be vectorized when using recurrent policies.")
            actions = actions[0]

        return actions, state


register_policy("ImageCnnPolicy", ImageCnnPolicy)
