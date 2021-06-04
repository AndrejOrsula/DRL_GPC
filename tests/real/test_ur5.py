#!/usr/bin/env python3

import time
from drl_grasping.control.ur5_control import UR5Control


def main(args=None):

    ur5_control = UR5Control()
    ur5_control.stop()

    print(ur5_control.get_ee_position())
    print(ur5_control.get_ee_orientation())

    ur5_control.gripper_close()
    time.sleep(1)
    ur5_control.gripper_open()
    time.sleep(1)

    ur5_control.move_home()
    print("Execution finished (move_home)")

    ur5_control.set_position_goal([0.45, 0.0, 0.1916])
    ur5_control.set_orientation_goal([1.0, 0.0, 0.0, 0.0],
                                     is_xyzw=True)
    ur5_control.execute()
    print("Execution finished (pose_goal)")

    ur5_control.stop()
    exit()


if __name__ == "__main__":
    main()
