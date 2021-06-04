from drl_grasping import utils
from math3d.orientation import Orientation
from scipy.spatial.transform import Rotation
from typing import Tuple
import time
import urx


class UR5Control():

    def __init__(self,
                 host: str = "192.168.1.102",
                 use_rt: bool = False,
                 tcp_position: Tuple[float, float, float] = (0.0,
                                                             0.0,
                                                             0.24),
                 tcp_rpy: Tuple[float, float, float] = (0.0,
                                                        0.0,
                                                        0.0),
                 payload_weight: float = 1.0,
                 payload_centre_of_mass: Tuple[float, float, float] = (0.0,
                                                                       0.0,
                                                                       0.2),
                 tool_voltage: int = 24,
                 slow_gripper: bool = False,
                 gripper_close_delay: float = 1.0,
                 acceleration: float = 0.75,
                 velocity: float = 1.0,
                 move_home_acceleration: float = 0.78539816,
                 move_home_velocity: float = 1.0471976,
                 wait_for_execution: bool = True,
                 base_mounting_rotation: float = -2.0071286,
                 home_joint_positions: Tuple[float, ...] = (-2.0071286,
                                                            -1.5707963,
                                                            -1.5707963,
                                                            -1.5707963,
                                                            1.5707963,
                                                            1.5707963)):
        self.robot = urx.Robot(host=host, use_rt=use_rt)
        self.robot.set_tcp(tcp=tcp_position + tcp_rpy)
        self.robot.set_payload(weight=payload_weight,
                               cog=payload_centre_of_mass)
        self.robot.set_tool_voltage(val=tool_voltage)
        self.robot.set_digital_out(9, slow_gripper)

        self._gripper_close_delay = gripper_close_delay
        self._acceleration = acceleration
        self._velocity = velocity
        self._move_home_acceleration = move_home_acceleration
        self._move_home_velocity = move_home_velocity
        self._wait_for_execution = wait_for_execution
        self._base_mounting_rotation = base_mounting_rotation
        self._home_joint_positions = home_joint_positions

        self._is_gripper_closed = False

        # A small delay to get everything setup properly before continuing
        time.sleep(2)

    def set_position_goal(self,
                          position_goal: Tuple[float, float, float],
                          min_height: float = 0.015):

        position_goal[2] = max(min_height, position_goal[2])
        self._position_goal = tuple(utils.math.rot_z(position_goal,
                                                     self._base_mounting_rotation))

    def set_orientation_goal(self,
                             orientation_goal: Tuple[float, float, float, float],
                             is_xyzw: bool = True):

        rpy = Rotation.from_quat(
            orientation_goal).as_euler('xyz', degrees=False)

        rot = Orientation.new_euler(rpy, encoding='xyz')
        rot.rotate_zb(self._base_mounting_rotation)

        self._orientation_goal = tuple(rot.get_rotation_vector())

    def plan_path(self, **kwargs):
        pass

    def execute(self):
        print("Moving to ", self._position_goal + self._orientation_goal)
        self.robot.movel(tpose=self._position_goal + self._orientation_goal,
                         acc=self._acceleration,
                         vel=self._velocity,
                         wait=self._wait_for_execution,
                         relative=False)

    def move_home(self):
        print("Moving home")
        self.gripper_open()
        self.robot.movej(joints=self._home_joint_positions,
                         acc=self._move_home_acceleration,
                         vel=self._move_home_velocity,
                         wait=self._wait_for_execution,
                         relative=False)

    def stop(self):
        self.robot.stopj()

    def gripper_close(self, **kwargs):
        self.robot.set_digital_out(8, True)
        if not self._is_gripper_closed:
            time.sleep(self._gripper_close_delay)
            self._is_gripper_closed = True

    def gripper_open(self, **kwargs):
        self.robot.set_digital_out(8, False)
        self._is_gripper_closed = False

    def get_ee_position(self) -> Tuple[float, float, float]:
        return tuple(utils.math.rot_z(self.robot.get_pos(), -self._base_mounting_rotation))

    def get_ee_orientation(self) -> Tuple[float, float, float, float]:
        """
        Return the current xyzw quaternion of the end effector
        """

        rot = self.robot.get_orientation()
        rot.rotate_zb(-self._base_mounting_rotation)
        quat = rot.unit_quaternion
        quat_xyz = quat.vector_part
        quat_w = quat.scalar_part

        quat_xyzw = (quat_xyz[0], quat_xyz[1], quat_xyz[2], quat_w)

        return quat_xyzw

    def get_joint_positions(self) -> Tuple[float, float, float, float, float, float]:
        """
        Return the current xyzw quaternion of the end effector
        """

        return self.robot.getj()