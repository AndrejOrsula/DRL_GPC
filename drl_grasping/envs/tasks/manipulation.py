from drl_grasping.utils.math import quat_mul
from drl_grasping.utils.conversions import orientation_6d_to_quat
from gym_ignition.base import task
from gym_ignition.rbd import conversions
from gym_ignition.utils.typing import Action, Reward, Observation
from gym_ignition.utils.typing import ActionSpace, ObservationSpace
from itertools import count
from scipy.spatial.transform import Rotation
from typing import List, Tuple, Union
import abc
import math
import numpy as np


class Manipulation(task.Task, abc.ABC):
    _ids = count(0)

    # Parameters for ManipulationGazeboEnvRandomizer
    _robot_position: Tuple[float, float, float] = (0, 0, 0)
    _robot_quat_xyzw: Tuple[float, float, float, float] = (0, 0, 0, 1)
    _robot_arm_collision: bool = True
    _robot_hand_collision: bool = True
    _robot_initial_joint_positions_panda: Tuple[float, ...] = (0.0,
                                                               0.0,
                                                               0.0,
                                                               -1.57,
                                                               0.0,
                                                               1.57,
                                                               0.79,
                                                               0.0,
                                                               0.0)
    _robot_initial_joint_positions_ur5_rg2: Tuple[float, ...] = (0.0,
                                                                 0.0,
                                                                 1.57,
                                                                 0.0,
                                                                 -1.57,
                                                                 -1.57,
                                                                 0.0,
                                                                 0.0)

    _workspace_centre: Tuple[float, float, float] = (0.5, 0, 0.25)
    _workspace_volume: Tuple[float, float, float] = (1.0, 1.0, 1.0)

    _camera_enable: bool = False
    _camera_type: str = 'rgbd_camera'
    _camera_render_engine: str = 'ogre2'
    _camera_position: Tuple[float, float, float] = (0.5, 0, 1)
    _camera_quat_xyzw: Tuple[float, float,
                             float, float] = (-0.707, 0, 0.707, 0)
    _camera_width: int = 128
    _camera_height: int = 128
    _camera_update_rate: int = 10
    _camera_horizontal_fov: float = 1.0
    _camera_vertical_fov: float = 1.0
    _camera_clip_color: Tuple[float, float] = (0.01, 1000.0)
    _camera_clip_depth: Tuple[float, float] = (0.01, 10.0)
    _camera_ros2_bridge_color: bool = False
    _camera_ros2_bridge_depth: bool = False
    _camera_ros2_bridge_points: bool = False

    _ground_enable: bool = False
    _ground_position: Tuple[float, float, float] = (0, 0, 0)
    _ground_quat_xyzw: Tuple[float, float, float, float] = (0, 0, 0, 1)
    _ground_size: Tuple[float, float] = (2.0, 2.0)

    _object_enable: bool = False
    # 'box' [x, y, z], 'sphere' [radius], 'cylinder' [radius, height]
    _object_type: str = 'box'
    _object_dimensions: List[float] = [0.05, 0.05, 0.05]
    _object_mass: float = 0.1
    _object_collision: bool = True
    _object_visual: bool = True
    _object_static: bool = False
    _object_color: Tuple[float, float, float, float] = (0.8, 0.8, 0.8, 1.0)
    _object_spawn_centre: Tuple[float, float, float] = \
        (_workspace_centre[0],
         _workspace_centre[1],
         _workspace_centre[2])
    _object_spawn_volume_proportion: float = 0.75
    _object_spawn_volume: Tuple[float, float, float] = \
        (_object_spawn_volume_proportion*_workspace_volume[0],
         _object_spawn_volume_proportion*_workspace_volume[1],
         _object_spawn_volume_proportion*_workspace_volume[2])
    _object_quat_xyzw: Tuple[float, float, float, float] = (0, 0, 0, 1)

    _insert_scene_broadcaster_plugin: bool = True
    _insert_user_commands_plugin: bool = False

    _relative_position_scaling_factor: float = 0.1
    _z_relative_orientation_scaling_factor: float = np.pi/4.0

    def __init__(self,
                 agent_rate: float,
                 robot_model: str,
                 restrict_position_goal_to_workspace: bool,
                 robot_controller_backend: str = "moveit2",
                 verbose: bool = False,
                 **kwargs):
        # Add to ids
        self.id = next(self._ids)

        # Initialize the Task base class
        task.Task.__init__(self, agent_rate=agent_rate)

        self._robot_model = robot_model
        if 'panda' == robot_model:
            self._robot_initial_joint_positions = self._robot_initial_joint_positions_panda
        elif 'ur5_rg2' == robot_model:
            self._robot_initial_joint_positions = self._robot_initial_joint_positions_ur5_rg2

        # Control (MoveIt2)
        self.robot_controller_backend_id = robot_controller_backend
        if "moveit2" == robot_controller_backend:
            from drl_grasping.control import MoveIt2
            self.robot_controller = MoveIt2(robot_model=robot_model,
                                            use_sim_time=False,
                                            node_name=f'ign_moveit2_py_{self.id}')
        elif "frankx" == robot_controller_backend:
            from drl_grasping.control import PandaControl
            self.robot_controller = PandaControl()
        elif "ur5" == robot_controller_backend:
            from drl_grasping.control import UR5Control
            self.robot_controller = UR5Control()
        else:
            raise Exception(
                f"Unsupported robot controller backend '{robot_controller_backend}'. Please select 'moveit2' (for simulation) or 'frankx' (for real-life Franka Emika Panda).")

        # Names of important models
        self.robot_name = None
        self.robot_base_link_name = None
        self.robot_ee_link_name = None
        self.robot_gripper_link_names = []
        self.camera_name = None
        self.ground_name = None
        self.object_names = []

        # Additional parameters
        self._restrict_position_goal_to_workspace = restrict_position_goal_to_workspace
        self._verbose = verbose

    def create_spaces(self) -> Tuple[ActionSpace, ObservationSpace]:

        # Action space
        action_space = self.create_action_space()
        # Observation space
        observation_space = self.create_observation_space()

        return action_space, observation_space

    def create_action_space(self) -> ActionSpace:

        pass

    def create_observation_space(self) -> ObservationSpace:

        pass

    def set_action(self, action: Action) -> None:

        pass

    def get_observation(self) -> Observation:

        pass

    def get_reward(self) -> Reward:

        pass

    def is_done(self) -> bool:

        pass

    def reset_task(self) -> None:

        pass

    def set_position_goal(self,
                          absolute: Union[Tuple[float, float, float],
                                          None] = None,
                          relative: Union[Tuple[float, float, float],
                                          None] = None):

        target_pos = None

        if absolute is not None:
            # If absolute position is selected, directly use the action as target
            target_pos = absolute
        elif relative is not None:
            # Scale relative action to metric units
            relative_pos = self._relative_position_scaling_factor * relative
            # Get current position
            current_pos = self.get_ee_position()

            # Compute target position
            target_pos = [current_pos[0] + relative_pos[0],
                          current_pos[1] + relative_pos[1],
                          current_pos[2] + relative_pos[2]]

        if target_pos is not None:
            # Restrict target position to a limited workspace
            if self._restrict_position_goal_to_workspace:
                centre = self._workspace_centre
                volume = self._workspace_volume
                for i in range(3):
                    target_pos[i] = min(centre[i] + volume[i]/2,
                                        max(centre[i] - volume[i]/2,
                                            target_pos[i]))
            # Set position goal
            self.robot_controller.set_position_goal(target_pos)
        else:
            print('error: Neither absolute or relative position is set')

    def set_orientation_goal(self,
                             absolute: Union[Tuple[float, ...], None] = None,
                             relative: Union[Tuple[float, ...], None] = None,
                             representation: str = 'quat',
                             xyzw: bool = True):

        target_quat_xyzw = None

        if absolute is not None:
            # Convert absolute orientation representation to quaternion
            if 'quat' == representation:
                if xyzw:
                    target_quat_xyzw = absolute
                else:
                    target_quat_xyzw = conversions.Quaternion.to_xyzw(absolute)
            elif '6d' == representation:
                vectors = tuple(absolute[x:x + 3]
                                for x, _ in enumerate(absolute) if x % 3 == 0)
                target_quat_xyzw = orientation_6d_to_quat(
                    vectors[0], vectors[1])
            elif 'z' == representation:
                target_quat_xyzw = Rotation.from_euler(
                    'xyz', [np.pi, 0, absolute]).as_quat()

        elif relative is not None:
            # Get current orientation
            current_quat_xyzw = self.get_ee_orientation()

            # For 'z' representation, result should always point down
            # Therefore, create a new quatertnion that contains only yaw component
            if 'z' == representation:
                current_yaw = Rotation.from_quat(
                    current_quat_xyzw).as_euler('xyz')[2]
                current_quat_xyzw = Rotation.from_euler(
                    'xyz', [np.pi, 0, current_yaw]).as_quat()

            # Convert relative orientation representation to quaternion
            relative_quat_xyzw = None
            if 'quat' == representation:
                if xyzw:
                    relative_quat_xyzw = relative
                else:
                    relative_quat_xyzw = \
                        conversions.Quaternion.to_xyzw(relative)
            elif '6d' == representation:
                vectors = tuple(relative[x:x + 3]
                                for x, _ in enumerate(relative) if x % 3 == 0)
                relative_quat_xyzw = orientation_6d_to_quat(
                    vectors[0], vectors[1])
            elif 'z' == representation:
                relative *= self._z_relative_orientation_scaling_factor
                relative_quat_xyzw = Rotation.from_euler(
                    'xyz', [0, 0, relative]).as_quat()

            # Compute target position (combine quaternions)
            target_quat_xyzw = quat_mul(current_quat_xyzw, relative_quat_xyzw)

        if target_quat_xyzw is not None:
            # Normalise quaternion (should not be needed, but just to be safe)
            target_quat_xyzw /= np.linalg.norm(target_quat_xyzw)
            # Set orientation goal
            self.robot_controller.set_orientation_goal(target_quat_xyzw)
        else:
            print('error: Neither absolute or relative orientation is set')

    def get_ee_position(self) -> Tuple[float, float, float]:

        if "moveit2" != self.robot_controller_backend_id:
            # If using a real robot, use the corresponding backend
            return self.robot_controller.get_ee_position()

        # Else use simulation params (Gazebo lookup is much faster than MoveIt2 forward kinematics)
        robot = self.world.get_model(self.robot_name).to_gazebo()
        return robot.get_link(self.robot_ee_link_name).position()

    def get_ee_orientation(self) -> Tuple[float, float, float, float]:
        """
        Return the current xyzw quaternion of the end effector
        """

        if "moveit2" != self.robot_controller_backend_id:
            # If using a real robot, use the corresponding backend
            return self.robot_controller.get_ee_orientation()

        # Else use simulation params (Gazebo lookup is much faster than MoveIt2 forward kinematics)
        robot = self.world.get_model(self.robot_name).to_gazebo()
        return conversions.Quaternion.to_xyzw(robot.get_link(self.robot_ee_link_name).orientation())
