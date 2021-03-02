from drl_grasping.envs import tasks, models
from drl_grasping.utils import Tf2Broadcaster
from gym_ignition import randomizers
from gym_ignition.randomizers import gazebo_env_randomizer
from gym_ignition.randomizers.gazebo_env_randomizer import MakeEnvCallable
from gym_ignition.rbd import conversions
from scenario import gazebo as scenario
from scipy.spatial.transform import Rotation
from typing import Union, Tuple
import abc
import numpy as np
import os


# Tasks that are supported by this randomizer (used primarily for type hinting)
SupportedTasks = Union[tasks.Reach, tasks.ReachOctree,
                       tasks.Grasp, tasks.GraspOctree]


class ManipulationGazeboEnvRandomizer(gazebo_env_randomizer.GazeboEnvRandomizer,
                                      randomizers.abc.PhysicsRandomizer,
                                      randomizers.abc.TaskRandomizer,
                                      abc.ABC):
    """
    Basic randomizer for robotic manipulation environments that also populates the simulated world.
    """

    def __init__(self,
                 env: MakeEnvCallable,
                 physics_rollouts_num: int = 0,
                 robot_random_joint_positions: bool = False,
                 robot_random_joint_positions_std: float = 0.1,
                 camera_pose_rollouts_num: int = 0,
                 camera_random_pose_distance: float = 0.5,
                 camera_random_pose_height: Tuple[float, float] = (0.5, 1.0),
                 camera_noise_mean: float = None,
                 camera_noise_stddev: float = None,
                 ground_model_rollouts_num: int = 0,
                 object_random_pose: bool = False,
                 object_models_rollouts_num: int = 0,
                 object_random_model_count: int = 2,
                 visualise_workspace: bool = False,
                 visualise_spawn_volume: bool = False,
                 verbose: bool = False):

        # Initialize base classes
        randomizers.abc.TaskRandomizer.__init__(self)
        randomizers.abc.PhysicsRandomizer.__init__(self,
                                                   randomize_after_rollouts_num=physics_rollouts_num)
        gazebo_env_randomizer.GazeboEnvRandomizer.__init__(self,
                                                           env=env,
                                                           physics_randomizer=self)

        # Randomizers, their frequency and counters for different randomizers
        self._robot_random_joint_positions = robot_random_joint_positions
        self._camera_pose_rollouts_num = camera_pose_rollouts_num
        self._camera_pose_rollout_counter = camera_pose_rollouts_num
        self._ground_model_rollouts_num = ground_model_rollouts_num
        self._ground_model_rollout_counter = ground_model_rollouts_num
        self._object_random_pose = object_random_pose
        self._object_models_rollouts_num = object_models_rollouts_num
        self._object_models_rollout_counter = object_models_rollouts_num

        # Additional parameters
        self._robot_random_joint_positions_std = robot_random_joint_positions_std
        self._camera_random_pose_distance = camera_random_pose_distance
        self._camera_random_pose_height = camera_random_pose_height
        self._camera_noise_mean = camera_noise_mean
        self._camera_noise_stddev = camera_noise_stddev
        self._object_random_model_count = object_random_model_count
        self._visualise_workspace = visualise_workspace
        self._visualise_spawn_volume = visualise_spawn_volume
        self._verbose = verbose

        # Flag that determines whether environment has already been initialised
        self.__env_initialised = False

    # ===========================
    # PhysicsRandomizer interface
    # ===========================

    def get_engine(self):

        return scenario.PhysicsEngine_dart

    def randomize_physics(self, task: SupportedTasks, **kwargs) -> None:

        gravity_z = task.np_random.normal(loc=-9.80665, scale=0.02)
        if not task.world.to_gazebo().set_gravity((0, 0, gravity_z)):
            raise RuntimeError("Failed to set the gravity")

    # ========================
    # TaskRandomizer interface
    # ========================

    def randomize_task(self, task: SupportedTasks, **kwargs) -> None:

        # Get gazebo instance associated with the task
        if "gazebo" not in kwargs:
            raise ValueError("gazebo kwarg not passed to the task randomizer")
        gazebo = kwargs["gazebo"]

        if not self.__env_initialised:
            # TODO (low priority): TF2 - Move this to task
            # Broadcaster of tf (world -> robot, world -> camera)
            self._tf2_broadcaster = Tf2Broadcaster(
                node_name=f'drl_grasping_camera_tf_broadcaster_{task.id}')

            # Initialise all models and world plugins
            self.init_models(task=task,
                             gazebo=gazebo)

            # Visualise volumes in GUI if desired
            if self._visualise_workspace:
                self.visualise_workspace(task=task,
                                         gazebo=gazebo)
            if self._visualise_spawn_volume:
                self.visualise_spawn_volume(task=task,
                                            gazebo=gazebo)

            # Insert world plugins
            # TODO: fix (currently placed in default world)
            # if task._insert_scene_broadcaster_plugin:
            #     task.world.to_gazebo().insert_world_plugin("ignition-gazebo-scene-broadcaster-system",
            #                                                "ignition::gazebo::systems::SceneBroadcaster")
            # if task._insert_user_commands_plugin:
            #     task.world.to_gazebo().insert_world_plugin("ignition-gazebo-user-commands-system",
            #                                                "ignition::gazebo::systems::UserCommands")
            self.__env_initialised = True

        # Randomize models if needed
        self.randomize_models(task=task,
                              gazebo=gazebo)

        if hasattr(task, 'camera_sub'):
            # Execute steps until observation after reset is available
            task.camera_sub.reset_new_observation_checker()
            while not task.camera_sub.new_observation_available():
                if self._verbose:
                    print("Waiting for new observation after reset")
                if not gazebo.run(paused=False):
                    raise RuntimeError(
                        "Failed to execute a running Gazebo run")
        else:
            # Otherwise execute exactly one unpaused steps to update JointStatePublisher (and potentially others)
            if not gazebo.run(paused=False):
                raise RuntimeError("Failed to execute a running Gazebo run")

    def init_models(self,
                    task: SupportedTasks,
                    gazebo: scenario.GazeboSimulator):
        """
        Initialise all models at beginning.
        All models that are re-spawned with randomizers are ignored here.
        """

        model_names = task.world.to_gazebo().model_names()
        if len(model_names) > 0:
            print(f"World currently contains models: {model_names}")

        # Insert robot
        if task.robot_name is None:
            print("Inserting robot")
            self.add_robot(task=task,
                           gazebo=gazebo)

        # Insert camera
        if task._camera_enable and task.camera_name is None:
            print("Inserting camera")
            self.add_camera(task=task,
                            gazebo=gazebo)

        # Insert default ground plane if there is none and randomization is disabled
        if task._ground_enable and task.ground_name is None and not self.ground_model_randomizer_enabled():
            print("Inserting default ground plane")
            self.add_default_ground(task=task,
                                    gazebo=gazebo)

        # Insert default object if there is none and randomization is disabled
        if task._object_enable and len(task.object_names) == 0 and not self.object_models_randomizer_enabled():
            print("Inserting default object")
            self.add_default_object(task=task,
                                    gazebo=gazebo)

    def add_robot(self,
                  task: SupportedTasks,
                  gazebo: scenario.GazeboSimulator):

        robot = None
        if 'panda' == task._robot_model:
            robot = models.Panda(world=task.world,
                                 position=task._robot_position,
                                 orientation=conversions.Quaternion.to_wxyz(
                                     task._robot_quat_xyzw),
                                 initial_joint_positions=task._robot_initial_joint_positions)
        task.robot_name = robot.name()
        task.robot_ee_link_name = robot.get_ee_link_name()
        task.robot_gripper_link_names = robot.get_gripper_link_names()

        # TODO (low priority): TF2 - Move this to task
        robot_base_frame_id = robot.link_names()[0]
        self._tf2_broadcaster.broadcast_tf(translation=task._robot_position,
                                           rotation=task._robot_quat_xyzw,
                                           xyzw=True,
                                           child_frame_id=robot_base_frame_id)

        # Execute a paused run to process model insertion
        if not gazebo.run(paused=True):
            raise RuntimeError("Failed to execute a paused Gazebo run")

    def add_camera(self,
                   task: SupportedTasks,
                   gazebo: scenario.GazeboSimulator):

        # First add a sensor plugin to the world to enable
        task.world.to_gazebo().insert_world_plugin("libignition-gazebo-sensors-system.so",
                                                   "ignition::gazebo::systems::Sensors",
                                                   f"""
                                                    <sdf version="1.7">
                                                        <render_engine>{task._camera_render_engine}</render_engine>
                                                    </sdf>
                                                    """)

        # Create camera
        camera = models.Camera(world=task.world,
                               position=task._camera_position,
                               orientation=conversions.Quaternion.to_wxyz(
                                   task._camera_quat_xyzw),
                               camera_type=task._camera_type,
                               width=task._camera_width,
                               height=task._camera_height,
                               update_rate=task._camera_update_rate,
                               horizontal_fov=task._camera_horizontal_fov,
                               vertical_fov=task._camera_vertical_fov,
                               clip_color=task._camera_clip_color,
                               clip_depth=task._camera_clip_depth,
                               noise_mean=self._camera_noise_mean,
                               noise_stddev=self._camera_noise_stddev,
                               ros2_bridge_color=task._camera_ros2_bridge_color,
                               ros2_bridge_depth=task._camera_ros2_bridge_depth,
                               ros2_bridge_points=task._camera_ros2_bridge_points)
        task.camera_name = camera.name()

        # TODO (low priority): TF2 - Move this to task
        camera_base_frame_id = camera.frame_id()
        self._tf2_broadcaster.broadcast_tf(translation=task._camera_position,
                                           rotation=task._camera_quat_xyzw,
                                           xyzw=True,
                                           child_frame_id=camera_base_frame_id)

        # Execute a paused run to process model insertion
        if not gazebo.run(paused=True):
            raise RuntimeError("Failed to execute a paused Gazebo run")

    def add_default_ground(self,
                           task: SupportedTasks,
                           gazebo: scenario.GazeboSimulator):

        ground = models.Ground(world=task.world,
                               position=task._ground_position,
                               orientation=conversions.Quaternion.to_wxyz(
                                   task._ground_quat_xyzw),
                               size=task._ground_size)
        task.ground_name = ground.name()

        # Execute a paused run to process model insertion
        if not gazebo.run(paused=True):
            raise RuntimeError("Failed to execute a paused Gazebo run")

    def add_default_object(self,
                           task: SupportedTasks,
                           gazebo: scenario.GazeboSimulator):

        object_position = list(task._workspace_centre)
        object_position[2] += task._object_spawn_height

        object_model = None
        if 'box' == task._object_type:
            object_model = models.Box(world=task.world,
                                      position=object_position,
                                      orientation=conversions.Quaternion.to_wxyz(
                                          task._object_quat_xyzw),
                                      size=task._object_size,
                                      mass=task._object_mass,
                                      collision=task._object_collision,
                                      visual=task._object_visual,
                                      static=task._object_static,
                                      color=task._object_color)
        task.object_names.append(object_model.name())

        # Execute a paused run to process model insertion
        if not gazebo.run(paused=True):
            raise RuntimeError("Failed to execute a paused Gazebo run")

    def randomize_models(self,
                         task: SupportedTasks,
                         gazebo: scenario.GazeboSimulator):
        """
        Randomize models if needed.
        """

        # Randomize robot joint positions if needed, else reset
        if self.robot_joint_position_randomizer_enabled():
            self.robot_random_joint_positions(task=task)
        else:
            self.reset_robot_joint_positions(task=task)

        # Randomize ground plane if needed
        if task._ground_enable and self.ground_model_expired():
            self.randomize_ground(task=task)

        # Randomize camera if needed
        if task._camera_enable and self.camera_pose_expired():
            self.randomize_camera_pose(task=task)

        # Randomize objects if needed
        # Note: No need to randomize pose of new models because they are already spawned randomly
        if task._object_enable:
            if self.object_models_expired():
                self.randomize_object_models(task=task)
            elif self.object_poses_randomizer_enabled():
                self.object_random_pose(task=task)
            elif not self.object_models_randomizer_enabled():
                self.reset_default_object_pose(task=task)

        # Execute a paused run to process these randomization operations
        if not gazebo.run(paused=True):
            raise RuntimeError("Failed to execute a paused Gazebo run")

    def robot_random_joint_positions(self,
                                     task: SupportedTasks):

        joint_positions = []
        for joint_limits in models.Panda.get_joint_limits():
            mean = (joint_limits[0] + joint_limits[1])/2
            std = self._robot_random_joint_positions_std * \
                abs(joint_limits[1] - joint_limits[0])
            random_position = task.np_random.normal(loc=mean, scale=std)
            joint_positions.append(random_position)

        robot = task.world.to_gazebo().get_model(task.robot_name)
        if not robot.to_gazebo().reset_joint_positions(joint_positions):
            raise RuntimeError("Failed to reset the robot joint position")
        if not robot.to_gazebo().reset_joint_velocities([0.0] * len(joint_positions)):
            raise RuntimeError("Failed to reset the robot joint velocities")

        # Send new positions also to the controller
        finger_count = models.Panda.get_finger_count()
        task.moveit2.move_to_joint_positions(joint_positions[:-finger_count])

    def reset_robot_joint_positions(self,
                                    task: SupportedTasks):

        robot = task.world.to_gazebo().get_model(task.robot_name)
        if not robot.to_gazebo().reset_joint_positions(task._robot_initial_joint_positions):
            raise RuntimeError("Failed to reset the robot joint position")
        if not robot.to_gazebo().reset_joint_velocities([0.0] * len(task._robot_initial_joint_positions)):
            raise RuntimeError("Failed to reset the robot joint velocities")

        # Send new positions also to the controller
        finger_count = models.Panda.get_finger_count()
        task.moveit2.move_to_joint_positions(
            task._robot_initial_joint_positions[:-finger_count])

    def randomize_camera_pose(self,
                              task: SupportedTasks):

        # TODO: Take another look at this (random camera pose)
        # Get random camera pose, centred at object position (or centre of object spawn box)
        position, quat_xyzw = self.get_random_camera_pose(
            task,
            centre=task._workspace_centre,
            distance=self._camera_random_pose_distance,
            camera_height=self._camera_random_pose_height)

        # Move pose of the camera
        camera = task.world.to_gazebo().get_model(task.camera_name)
        camera.to_gazebo().reset_base_pose(position,
                                           conversions.Quaternion.to_wxyz(quat_xyzw))

        # TODO (low priority): TF2 - Move this to task
        camera_base_frame_id = models.Camera.frame_id_name(task.camera_name)
        self._tf2_broadcaster.broadcast_tf(translation=position,
                                           rotation=quat_xyzw,
                                           xyzw=True,
                                           child_frame_id=camera_base_frame_id)

    def get_random_camera_pose(self, task: SupportedTasks, centre, distance, camera_height):

        # Range [0;pi] [-pi;pi]
        theta = task.np_random.uniform(0.0, 1.0) * np.pi
        phi = task.np_random.uniform(-1.0, 1.0) * np.pi

        # Switch to cartesian coordinates
        x = np.sin(theta) * np.cos(phi)
        y = np.sin(theta) * np.sin(phi)
        z = task.np_random.uniform(camera_height[0], camera_height[1])

        pitch = np.arctan2(z, np.sqrt(x**2+y**2))
        yaw = np.arctan2(y, x) + np.pi

        position = [x*distance + centre[0],
                    y*distance + centre[1],
                    z*distance + centre[2]]
        quat_xyzw = Rotation.from_euler('xyz', [0.0, pitch, yaw],).as_quat()

        return position, quat_xyzw

    def randomize_ground(self,
                         task: SupportedTasks):

        # Remove existing ground
        if task.ground_name is not None:
            if not task.world.to_gazebo().remove_model(task.ground_name):
                raise RuntimeError(f"Failed to remove {task.ground_name}")

        # Add new random ground
        plane = models.RandomGround(world=task.world,
                                    position=task._ground_position,
                                    orientation=conversions.Quaternion.to_wxyz(
                                        task._ground_quat_xyzw),
                                    size=task._ground_size,
                                    np_random=task.np_random,
                                    texture_dir=os.environ.get('DRL_GRASPING_PBR_TEXTURES_DIR',
                                                               default=''))
        task.ground_name = plane.name()

    def reset_default_object_pose(self,
                                  task: SupportedTasks):

        assert(len(task.object_names) == 1)

        object_position = list(task._workspace_centre)
        object_position[2] += task._object_spawn_height

        obj = task.world.to_gazebo().get_model(task.object_names[0])
        obj.to_gazebo().reset_base_pose(object_position,
                                        conversions.Quaternion.to_wxyz(task._object_quat_xyzw))
        obj.to_gazebo().reset_base_world_velocity(
            [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

    def randomize_object_models(self,
                                task: SupportedTasks):

        # Remove all existing models
        if len(self.task.object_names) > 0:
            for object_name in self.task.object_names:
                if not task.world.to_gazebo().remove_model(object_name):
                    raise RuntimeError(f"Failed to remove {object_name}")
            self.task.object_names.clear()

        # Insert new models with random pose
        while len(self.task.object_names) < self._object_random_model_count:
            position, quat_random = self.get_random_object_pose(centre=task._workspace_centre,
                                                                volume=task._object_spawn_volume,
                                                                height=task._object_spawn_height,
                                                                np_random=task.np_random)
            try:
                model = models.RandomObject(world=task.world,
                                            position=position,
                                            orientation=quat_random,
                                            np_random=task.np_random)
                self.task.object_names.append(model.name())
            except:
                # TODO (low priority): Automatically blacklist a model if Gazebo does not accept it
                pass

    def object_random_pose(self,
                           task: SupportedTasks):

        for object_name in self.task.object_names:
            position, quat_random = self.get_random_object_pose(centre=task._workspace_centre,
                                                                volume=task._object_spawn_volume,
                                                                height=task._object_spawn_height,
                                                                np_random=task.np_random)
            obj = task.world.to_gazebo().get_model(object_name)
            obj.to_gazebo().reset_base_pose(position, quat_random)
            obj.to_gazebo().reset_base_world_velocity(
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

    def get_random_object_pose(self, centre, volume, height, np_random):

        position = [
            centre[0] + np_random.uniform(-volume[0]/2, volume[0]/2),
            centre[1] + np_random.uniform(-volume[1]/2, volume[1]/2),
            centre[2] + np_random.uniform(-volume[2]/2, volume[2]/2) + height,
        ]
        quat = np_random.uniform(-1, 1, 4)
        quat /= np.linalg.norm(quat)

        return position, quat

    # ============================
    # Randomizer rollouts checking
    # ============================

    def object_models_randomizer_enabled(self) -> bool:
        """
        Checks if object model randomizer is enabled.

        Return:
            True if enabled, false otherwise.
        """

        if self._object_models_rollouts_num == 0:
            return False
        else:
            return True

    def object_models_expired(self) -> bool:
        """
        Checks if object models need to be randomized.

        Return:
            True if expired, false otherwise.
        """

        if not self.object_models_randomizer_enabled():
            return False

        self._object_models_rollout_counter += 1

        if self._object_models_rollout_counter >= self._object_models_rollouts_num:
            self._object_models_rollout_counter = 0
            return True

        return False

    def object_poses_randomizer_enabled(self) -> bool:
        """
        Checks if object poses randomizer is enabled.

        Return:
            True if enabled, false otherwise.
        """

        return self._object_random_pose

    def robot_joint_position_randomizer_enabled(self) -> bool:
        """
        Checks if robot joint position randomizer is enabled.

        Return:
            True if enabled, false otherwise.
        """

        return self._robot_random_joint_positions

    def ground_model_randomizer_enabled(self) -> bool:
        """
        Checks if ground randomizer is enabled.

        Return:
            True if enabled, false otherwise.
        """

        if self._ground_model_rollouts_num == 0:
            return False
        else:
            return True

    def ground_model_expired(self) -> bool:
        """
        Checks if ground model needs to be randomized.

        Return:
            True if expired, false otherwise.
        """

        if not self.ground_model_randomizer_enabled():
            return False

        self._ground_model_rollout_counter += 1

        if self._ground_model_rollout_counter >= self._ground_model_rollouts_num:
            self._ground_model_rollout_counter = 0
            return True

        return False

    def camera_pose_randomizer_enabled(self) -> bool:
        """
        Checks if camera pose randomizer is enabled.

        Return:
            True if enabled, false otherwise.
        """

        if self._camera_pose_rollouts_num == 0:
            return False
        else:
            return True

    def camera_pose_expired(self) -> bool:
        """
        Checks if camera pose needs to be randomized.

        Return:
            True if expired, false otherwise.
        """

        if not self.camera_pose_randomizer_enabled():
            return False

        self._camera_pose_rollout_counter += 1

        if self._camera_pose_rollout_counter >= self._camera_pose_rollouts_num:
            self._camera_pose_rollout_counter = 0
            return True

        return False

    # =============================
    # Additional features and debug
    # =============================

    def visualise_workspace(self,
                            task: SupportedTasks,
                            gazebo: scenario.GazeboSimulator,
                            color: Tuple[float, float, float,
                                         float] = (0, 1, 0, 0.8)):

        # Insert a translucent box visible only in simulation with no physical interactions
        models.Box(world=task.world,
                   name="workspace_volume",
                   position=task._workspace_centre,
                   orientation=(0, 0, 0, 1),
                   size=task._workspace_volume,
                   collision=False,
                   visual=True,
                   gui_only=True,
                   static=True,
                   color=color)
        # Execute a paused run to process model insertion
        if not gazebo.run(paused=True):
            raise RuntimeError("Failed to execute a paused Gazebo run")

    def visualise_spawn_volume(self,
                               task: SupportedTasks,
                               gazebo: scenario.GazeboSimulator,
                               color: Tuple[float, float,
                                            float, float] = (0, 0, 1, 0.8),
                               color_with_height: Tuple[float, float,
                                                        float, float] = (1, 0, 1, 0.7)):

        # Insert translucent boxes visible only in simulation with no physical interactions
        models.Box(world=task.world,
                   name="object_spawn_volume",
                   position=task._workspace_centre,
                   orientation=(0, 0, 0, 1),
                   size=task._object_spawn_volume,
                   collision=False,
                   visual=True,
                   gui_only=True,
                   static=True,
                   color=color)
        models.Box(world=task.world,
                   name="object_spawn_volume_with_height",
                   position=(task._workspace_centre[0],
                             task._workspace_centre[1],
                             task._workspace_centre[2] + task._object_spawn_height),
                   orientation=(0, 0, 0, 1),
                   size=task._object_spawn_volume,
                   collision=False,
                   visual=True,
                   gui_only=True,
                   static=True,
                   color=color_with_height)
        # Execute a paused run to process model insertion
        if not gazebo.run(paused=True):
            raise RuntimeError("Failed to execute a paused Gazebo run")