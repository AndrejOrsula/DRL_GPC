#!/usr/bin/env python3

import time
from drl_grasping.control.panda_control import PandaControl


def main(args=None):

    panda_control = PandaControl(fci_ip="172.27.23.65",
                                 enforce_rt=True,
                                 dynamic_rel=0.005,
                                 gripper_speed=0.025,
                                 gripper_force=2,
                                 home_joint_positions=(-0.758936405539158,
                                                       -0.1634247208323649,
                                                       0.5819365766192949,
                                                       -2.039782614020731,
                                                       0.18485784827710336,
                                                       1.7258005222214592,
                                                       -2.633974810027414))

    panda_control.read()
    time.sleep(2)

    # panda_control.gripper_open(width=0.08)
    # time.sleep(2)

    # panda_control.gripper_close()
    # time.sleep(2)

    # panda_control.gripper_open(width=0.08)
    # time.sleep(2)

    # panda_control.move_home()
    # time.sleep(30)

    # panda_control.set_position_goal([0.5, 0.0, 0.25])
    # panda_control.set_orientation_goal([0.0, 0.0, 0.0, 1.0],
    #                                    is_xyzw=True)
    # panda_control.plan_path()
    # panda_control.execute()
    # time.sleep(10)


if __name__ == "__main__":
    main()
