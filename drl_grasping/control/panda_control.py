from frankx import Affine, Robot, Measure, Reaction, MotionData, LinearMotion, JointMotion, StopMotion
from gym_ignition.rbd import conversions
from scipy.spatial.transform import Rotation
from typing import Tuple
import time


# Note: Requires https://github.com/AndrejOrsula/frankx/tree/ignore_rt_kernel branch that enables libfranka without enforcing PREEMPT_RT kernel

class PandaControl():

    def __init__(self,
                 fci_ip: str = "172.27.23.65",
                 enforce_rt: bool = True,
                 dynamic_rel: float = 0.025,
                 gripper_speed: float = 0.02,
                 gripper_force: float = 20.0,
                 home_joint_positions: Tuple[float, ...] = (0.0,
                                                            0.0,
                                                            0.0,
                                                            -1.57,
                                                            0.0,
                                                            1.57,
                                                            0.79)):

        self.robot = Robot(fci_ip=fci_ip,
                           enforce_rt=enforce_rt)
        self.robot.set_default_behavior()
        self.robot.set_dynamic_rel(dynamic_rel)
        self.robot.recover_from_errors()

        self.gripper = self.robot.get_gripper()
        self.gripper.gripper_speed = gripper_speed
        self.gripper.gripper_force = gripper_force

        self._home_joint_positions = home_joint_positions
        self.set_reaction()
        
        # A small delay to get everything setup properly before continuing
        time.sleep(2)

    def set_position_goal(self, position_goal: Tuple[float, float, float]):
        self._position_goal = Affine(position_goal[0],
                                     position_goal[1],
                                     position_goal[2])

    def set_orientation_goal(self, orientation_goal: Tuple[float, float, float, float], is_xyzw: bool = True):
        if not is_xyzw:
            orientation_goal = conversions.Quaternion.to_xyzw(orientation_goal)

        rot_euler_xyz = Rotation.from_quat(
            orientation_goal).as_euler('xyz', degrees=False)
        self._orientation_goal = Affine(0.0, 0.0, 0.0,
                                        rot_euler_xyz[0], rot_euler_xyz[1], rot_euler_xyz[2])

    def plan_path(self, **kwargs):
        # Select the default planner
        self.plan_linear_path(**kwargs)

    def plan_linear_path(self, **kwargs):
        pose_goal = self._position_goal * self._orientation_goal
        self._motion = LinearMotion(pose_goal)

    def execute(self):
        if self._reaction is None:
            self.robot.move(self._motion)
        else:
            self.robot.move(self._motion, self._reaction)
            if self._reaction.has_fired:
                self.robot.recover_from_errors()
                print('Force exceeded 5N!')

        # # TODO: Try move async if needed (might not be good for non RT kernel)
        # thread = self.robot.move_async(self._motion)
        # thread.join()

    def move_home(self):
        self._motion = JointMotion(self._home_joint_positions)
        self.execute()

    def set_reaction(self, max_force_neg_z: float = 5.0, upwards_distance: float = 0.01):
        self._reaction = MotionData().with_reaction(Reaction(Measure.ForceZ < -max_force_neg_z,
                                                             StopMotion(Affine(0.0,
                                                                               0.0,
                                                                               upwards_distance),
                                                                        0.0)))

    def disable_reaction(self):
        self._reaction = None

    def gripper_close(self, **kwargs) -> bool:
        is_grasping = self.gripper.clamp()
        return is_grasping

    def gripper_open(self, width: float = 0.08, **kwargs):
        self.gripper.release(width)

        # # TODO: Try gripper async if needed (might not be good for non RT kernel)
        # gripper_thread = self.gripper.move_unsafe_async(0.05)
        # gripper_thread.join()

    def read(self):
        state = self.robot.read_once()
        print('\nPose: ', self.robot.current_pose())
        print('O_T_EE: ', state.O_T_EE)
        print('Joints: ', state.q)
        print('Elbow:  ', state.elbow)

    def get_ee_position(self) -> Tuple[float, float, float]:
        self.robot.read_once()
        return self.robot.current_pose().vector()[:3]

    def get_ee_orientation(self) -> Tuple[float, float, float, float]:
        """
        Return the current xyzw quaternion of the end effector
        """
        self.robot.read_once()
        rot_euler_xyz = self.robot.current_pose().angles()
        return Rotation.from_euler(angles=rot_euler_xyz, seq='xyz', degrees=False).as_quat()
