from drl_grasping.envs.tasks.reach import Reach
from drl_grasping.perception import CameraSubscriber
from gym_ignition.utils.typing import Observation
from gym_ignition.utils.typing import ObservationSpace
from typing import Tuple
import abc
import gym
import numpy as np

# TODO: ReachDepthImage environment currently does not currently have a working CnnPolicy


class ReachDepthImage(Reach, abc.ABC):

    # Overwrite parameters for ManipulationGazeboEnvRandomizer
    _camera_enable: bool = True
    _camera_type: str = 'depth_camera'
    _camera_width: int = 128
    _camera_height: int = 128
    _camera_update_rate: int = 10
    _camera_horizontal_fov: float = 1.0
    _camera_vertical_fov: float = 1.0
    _camera_position: Tuple[float, float, float] = (1.1, -0.75, 0.45)
    _camera_quat_xyzw: Tuple[float, float,
                             float, float] = (-0.0402991, -0.0166924, 0.9230002, 0.3823192)
    _camera_ros2_bridge_depth: bool = True

    def __init__(self,
                 agent_rate: float,
                 robot_model: str,
                 restrict_position_goal_to_workspace: bool,
                 sparse_reward: bool,
                 act_quick_reward: float,
                 required_accuracy: float,
                 use_sim_time: bool = True,
                 verbose: bool = False,
                 **kwargs):

        # Initialize the Task base class
        Reach.__init__(self,
                       agent_rate=agent_rate,
                       robot_model=robot_model,
                       restrict_position_goal_to_workspace=restrict_position_goal_to_workspace,
                       sparse_reward=sparse_reward,
                       act_quick_reward=act_quick_reward,
                       required_accuracy=required_accuracy,
                       use_sim_time=use_sim_time,
                       verbose=verbose,
                       **kwargs)

        # Perception (RGB camera)
        self.camera_sub = CameraSubscriber(topic=f'/{self._camera_type}',
                                           is_point_cloud=False,
                                           use_sim_time=use_sim_time,
                                           node_name=f'drl_grasping_depth_camera_sub_{self.id}')

    def create_observation_space(self) -> ObservationSpace:

        # 0:height*width - depth image
        return gym.spaces.Box(low=0,
                              high=np.inf,
                              shape=(self._camera_height,
                                     self._camera_width, 1),
                              dtype=np.float32)

    def get_observation(self) -> Observation:

        # Get the latest image
        image = self.camera_sub.get_observation()

        # Construct from buffer and reshape
        depth_image = np.frombuffer(image.data, dtype=np.float32).reshape(self._camera_height,
                                                                          self._camera_width, 1)
        # Replace all instances of infinity with 0
        depth_image[depth_image == np.inf] = 0.0

        # Create the observation
        observation = Observation(depth_image)

        if self._verbose:
            print(f"\nobservation: {observation}")

        # Return the observation
        return observation
