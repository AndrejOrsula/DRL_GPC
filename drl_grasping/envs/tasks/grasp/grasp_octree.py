from collections import deque
from drl_grasping.envs.tasks.grasp import Grasp
from drl_grasping.perception import CameraSubscriber, OctreeCreator
from drl_grasping.utils.conversions import orientation_quat_to_6d
from gym_ignition.utils.typing import Observation
from gym_ignition.utils.typing import ObservationSpace
from typing import Tuple
import abc
import gym
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class GraspOctree(Grasp, abc.ABC):

    # Overwrite parameters for ManipulationGazeboEnvRandomizer
    _camera_enable: bool = True
    _camera_type: str = 'auto'
    _camera_width: int = 256
    _camera_height: int = 256
    _camera_update_rate: int = 10
    _camera_horizontal_fov: float = 0.9
    _camera_vertical_fov: float = 0.9
    _camera_position: Tuple[float, float, float] = (0.95, -0.55, 0.25)
    _camera_quat_xyzw: Tuple[float, float,
                             float, float] = (-0.0402991, -0.0166924, 0.9230002, 0.3823192)
    _camera_ros2_bridge_points: bool = True

    _workspace_volume: Tuple[float, float, float] = (0.24, 0.24, 0.2)
    _workspace_centre: Tuple[float, float, float] = (
        0.5, 0.0, _workspace_volume[2]/2)

    # Size of octree (boundary size)
    _octree_size: float = 0.24
    # A small offset to include ground in the observations
    _octree_ground_offset: float = 0.015
    _octree_min_bound: Tuple[float, float, float] = (_workspace_centre[0]-_octree_size/2,
                                                     _workspace_centre[1] -
                                                     _octree_size/2,
                                                     0.0 - _octree_ground_offset)
    _octree_max_bound: Tuple[float, float, float] = (_workspace_centre[0]+_octree_size/2,
                                                     _workspace_centre[1] +
                                                     _octree_size/2,
                                                     (_octree_size) - _octree_ground_offset)

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
                 octree_depth: int,
                 octree_full_depth: int,
                 octree_include_color: bool,
                 octree_n_stacked: int,
                 octree_max_size: int,
                 proprieceptive_observations: bool,
                 preload_replay_buffer: bool = False,
                 robot_controller_backend: str = "moveit2",
                 use_sim_time: bool = True,
                 verbose: bool = False,
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
                       preload_replay_buffer=preload_replay_buffer,
                       robot_controller_backend=robot_controller_backend,
                       use_sim_time=use_sim_time,
                       verbose=verbose,
                       **kwargs)

        if octree_include_color:
            self._camera_type = 'rgbd_camera'
        else:
            self._camera_type = 'depth_camera'

        # Perception (RGB-D camera - point cloud)
        self.camera_sub = CameraSubscriber(topic=f'/{self._camera_type}/points',
                                           is_point_cloud=True,
                                           use_sim_time=use_sim_time,
                                           node_name=f'drl_grasping_point_cloud_sub_{self.id}')

        self.octree_creator = OctreeCreator(min_bound=self._octree_min_bound,
                                            max_bound=self._octree_max_bound,
                                            depth=octree_depth,
                                            full_depth=octree_full_depth,
                                            include_color=octree_include_color,
                                            use_sim_time=use_sim_time,
                                            debug_draw=False,
                                            debug_write_octree=True,
                                            robot_frame_id='world',
                                            node_name=f'drl_grasping_octree_creator_{self.id}')

        # Additional parameters
        self._octree_n_stacked = octree_n_stacked
        self._octree_max_size = octree_max_size
        self._proprieceptive_observations = proprieceptive_observations

        # List of all octrees
        self.__stacked_octrees = deque([], maxlen=self._octree_n_stacked)

        self.tf_broadcaster = TransformBroadcaster(self.octree_creator)

    def create_observation_space(self) -> ObservationSpace:

        # 0:n - octree
        # Note: octree is currently padded with zeros to have constant size
        # TODO: Customize replay buffer to support variable sized observations
        # If enabled, proprieceptive observations will be embedded inside octree in a hacky way
        # (replace with Dict once https://github.com/DLR-RM/stable-baselines3/pull/243 is merged)
        return gym.spaces.Box(low=0,
                              high=255,
                              shape=(self._octree_n_stacked,
                                     self._octree_max_size),
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

        # Get the latest point cloud
        point_cloud = self.camera_sub.get_observation()

        # Contrust octree from this point cloud
        octree = self.octree_creator(point_cloud).numpy()

        # Pad octree with zeros to have a consistent length
        # TODO: Customize replay buffer to support variable sized observations
        octree_size = octree.shape[0]
        if octree_size > self._octree_max_size:
            print(f"ERROR: Octree is larger than the maximum "
                  f"allowed size (exceeded with {octree_size})")
        octree = np.pad(octree,
                        (0, self._octree_max_size - octree_size),
                        'constant',
                        constant_values=0)

        # Write the original length into the padded octree for reference
        octree[-4:] = np.ndarray(buffer=np.array([octree_size],
                                                 dtype='uint32').tobytes(),
                                 shape=(4,),
                                 dtype='uint8')
        # To get it back:
        # octree_size = np.frombuffer(buffer=octree[-4:],
        #                             dtype='uint32',
        #                             count=1)

        if self._proprieceptive_observations:
            # Add number of auxiliary observations to octree structure
            octree[-8:-4] = np.ndarray(buffer=np.array([10],
                                                       dtype='uint32').tobytes(),
                                       shape=(4,),
                                       dtype='uint8')

            # Gather proprioceptive observations
            ee_quat = self.get_ee_orientation()
            ee_position = self.get_ee_position()
            ee_orientation = orientation_quat_to_6d(
                quat_xyzw=ee_quat)
            aux_obs = (self._gripper_state,) + ee_position + \
                ee_orientation[0] + ee_orientation[1]

            # Add auxiliary observations into the octree structure
            octree[-48:-8] = np.ndarray(buffer=np.array(aux_obs, dtype='float32').tobytes(),
                                        shape=(40,),
                                        dtype='uint8')

            tf = TransformStamped()
            tf.header.frame_id = 'world'
            tf.child_frame_id = 'robot_ee'
            tf.transform.translation.x = ee_position[0]
            tf.transform.translation.y = ee_position[1]
            tf.transform.translation.z = ee_position[2]
            tf.transform.rotation.x = ee_quat[0]
            tf.transform.rotation.y = ee_quat[1]
            tf.transform.rotation.z = ee_quat[2]
            tf.transform.rotation.w = ee_quat[3]
            self.tf_broadcaster.sendTransform(tf)

        self.__stacked_octrees.append(octree)
        # For the first buffer after reset, fill with identical observations until deque is full
        while not self._octree_n_stacked == len(self.__stacked_octrees):
            self.__stacked_octrees.append(octree)

        # Create the observation
        observation = Observation(np.array(self.__stacked_octrees))

        if self._verbose:
            print(f"\nobservation: {observation}")

        # Return the observation
        return observation

    def reset_task(self):

        self.__stacked_octrees.clear()
        Grasp.reset_task(self)
