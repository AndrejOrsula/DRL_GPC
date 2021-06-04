from collections import deque
from drl_grasping.envs.tasks.grasp import Grasp
from drl_grasping.perception import CameraSubscriber
from drl_grasping.utils.conversions import orientation_quat_to_6d
from gym_ignition.utils.typing import Observation
from gym_ignition.utils.typing import ObservationSpace
from typing import Tuple
import abc
import gym
import numpy as np


class GraspColorImage(Grasp, abc.ABC):

    # Overwrite parameters for ManipulationGazeboEnvRandomizer
    _camera_enable: bool = True
    _camera_type: str = 'camera'
    _camera_width: int = 128
    _camera_height: int = 128
    _camera_update_rate: int = 10
    _camera_horizontal_fov: float = 0.9
    _camera_vertical_fov: float = 0.9
    _camera_position: Tuple[float, float, float] = (0.95, -0.55, 0.25)
    _camera_quat_xyzw: Tuple[float, float,
                             float, float] = (-0.0402991, -0.0166924, 0.9230002, 0.3823192)
    _camera_ros2_bridge_color: bool = True

    _workspace_volume: Tuple[float, float, float] = (0.24, 0.24, 0.2)
    _workspace_centre: Tuple[float, float, float] = (
        0.5, 0.0, _workspace_volume[2]/2)

    _object_spawn_centre: Tuple[float, float, float] = \
        (_workspace_centre[0],
         _workspace_centre[1],
         0.15)
    _object_spawn_volume_proportion: float = 0.75
    _object_spawn_volume: Tuple[float, float, float] = \
        (_object_spawn_volume_proportion*_workspace_volume[0],
         _object_spawn_volume_proportion*_workspace_volume[1],
         0.075)

    def __init__(self,
                 agent_rate: float,
                 robot_model: str,
                 restrict_position_goal_to_workspace: bool,
                 gripper_dead_zone: float,
                 full_3d_orientation: bool,
                 sparse_reward: bool,
                 normalize_reward: bool,
                 required_reach_distance: float,
                 required_lift_height: float,
                 reach_dense_reward_multiplier: float,
                 lift_dense_reward_multiplier: float,
                 act_quick_reward: float,
                 outside_workspace_reward: float,
                 ground_collision_reward: float,
                 n_ground_collisions_till_termination: int,
                 curriculum_enable_workspace_scale: bool,
                 curriculum_min_workspace_scale: float,
                 curriculum_enable_object_count_increase: bool,
                 curriculum_max_object_count: int,
                 curriculum_enable_stages: bool,
                 curriculum_stage_reward_multiplier: float,
                 curriculum_stage_increase_rewards: bool,
                 curriculum_success_rate_threshold: float,
                 curriculum_success_rate_rolling_average_n: int,
                 curriculum_restart_every_n_steps: int,
                 curriculum_skip_reach_stage: bool,
                 curriculum_skip_grasp_stage: bool,
                 curriculum_restart_exploration_at_start: bool,
                 max_episode_length: int,
                 image_n_stacked: int,
                 proprieceptive_observations: bool,
                 verbose: bool,
                 preload_replay_buffer: bool = False,
                 **kwargs):

        # Initialize the Task base class
        Grasp.__init__(self,
                       agent_rate=agent_rate,
                       robot_model=robot_model,
                       restrict_position_goal_to_workspace=restrict_position_goal_to_workspace,
                       gripper_dead_zone=gripper_dead_zone,
                       full_3d_orientation=full_3d_orientation,
                       sparse_reward=sparse_reward,
                       normalize_reward=normalize_reward,
                       required_reach_distance=required_reach_distance,
                       required_lift_height=required_lift_height,
                       reach_dense_reward_multiplier=reach_dense_reward_multiplier,
                       lift_dense_reward_multiplier=lift_dense_reward_multiplier,
                       act_quick_reward=act_quick_reward,
                       outside_workspace_reward=outside_workspace_reward,
                       ground_collision_reward=ground_collision_reward,
                       n_ground_collisions_till_termination=n_ground_collisions_till_termination,
                       curriculum_enable_workspace_scale=curriculum_enable_workspace_scale,
                       curriculum_min_workspace_scale=curriculum_min_workspace_scale,
                       curriculum_enable_object_count_increase=curriculum_enable_object_count_increase,
                       curriculum_max_object_count=curriculum_max_object_count,
                       curriculum_enable_stages=curriculum_enable_stages,
                       curriculum_stage_reward_multiplier=curriculum_stage_reward_multiplier,
                       curriculum_stage_increase_rewards=curriculum_stage_increase_rewards,
                       curriculum_success_rate_threshold=curriculum_success_rate_threshold,
                       curriculum_success_rate_rolling_average_n=curriculum_success_rate_rolling_average_n,
                       curriculum_restart_every_n_steps=curriculum_restart_every_n_steps,
                       curriculum_skip_reach_stage=curriculum_skip_reach_stage,
                       curriculum_skip_grasp_stage=curriculum_skip_grasp_stage,
                       curriculum_restart_exploration_at_start=curriculum_restart_exploration_at_start,
                       max_episode_length=max_episode_length,
                       verbose=verbose,
                       preload_replay_buffer=preload_replay_buffer,
                       **kwargs)

        # Perception (RGB-D camera - point cloud)
        self.camera_sub = CameraSubscriber(topic=f'/{self._camera_type}',
                                           is_point_cloud=False,
                                           node_name=f'drl_grasping_rgb_camera_sub_{self.id}')
        # Additional parameters
        self._image_n_stacked = image_n_stacked
        self._proprieceptive_observations = proprieceptive_observations

        # List of all octrees
        self.__stacked_images = deque([], maxlen=self._image_n_stacked)

    def create_observation_space(self) -> ObservationSpace:

        # float_size*channels*height*width - rgb image, stored as bytes
        size = 3*self._camera_height*self._camera_width
        if self._proprieceptive_observations:
            size += 4*11

        return gym.spaces.Box(low=0,
                              high=255,
                              shape=(self._image_n_stacked, size),
                              dtype=np.uint8)

    def create_proprioceptive_observation_space(self) -> ObservationSpace:

        # 0   - (gripper) Gripper state
        #       - 1.0: opened
        #       - -1.0: closed
        # 1:4 - (x, y, z) displacement
        #       - metric units, unbound
        # 4:10 - (v1_x, v1_y, v1_z, v2_x, v2_y, v2_z) 3D orientation in "6D representation"
        #       - normalised
        return gym.spaces.Box(low=np.array((-1.0,
                                            -np.inf, -np.inf, -np.inf,
                                            -1.0, -1.0, -1.0, -1.0, -1.0, -1.0)),
                              high=np.array((1.0,
                                             np.inf, np.inf, np.inf,
                                             1.0, 1.0, 1.0, 1.0, 1.0, 1.0)),
                              shape=(10,),
                              dtype=np.float32)

    def get_observation(self) -> Observation:

        # Get the latest image
        image = self.camera_sub.get_observation()
        # Convert to np array
        color_image = np.array(image.data,
                               dtype=np.uint8)

        if self._proprieceptive_observations:
            # Pad octree with zeros to have a place for proprioceptive observations
            color_image = np.pad(color_image,
                                 (0, 44),
                                 'constant',
                                 constant_values=0)

            # Add number of auxiliary observations to octree structure
            color_image[-4:] = np.ndarray(buffer=np.array([10],
                                                          dtype='uint32').tobytes(),
                                          shape=(4,),
                                          dtype='uint8')

            # Gather proprioceptive observations
            ee_position = self.get_ee_position()
            ee_orientation = orientation_quat_to_6d(
                quat_xyzw=self.get_ee_orientation())
            aux_obs = (self._gripper_state,) + ee_position + \
                ee_orientation[0] + ee_orientation[1]

            # Add auxiliary observations into the image structure
            color_image[-44:-4] = np.ndarray(buffer=np.array(aux_obs, dtype='float32').tobytes(),
                                             shape=(40,),
                                             dtype='uint8')

        self.__stacked_images.append(color_image)
        # For the first buffer after reset, fill with identical observations until deque is full
        while not self._image_n_stacked == len(self.__stacked_images):
            self.__stacked_images.append(color_image)

        # Create the observation
        observation = Observation(np.array(self.__stacked_images))

        if self._verbose:
            print(f"\nobservation: {observation}")

        # Return the observation
        return observation

    def reset_task(self):

        self.__stacked_images.clear()
        Grasp.reset_task(self)
