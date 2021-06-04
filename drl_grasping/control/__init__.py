from .moveit2 import MoveIt2

try:
    from .panda_control import PandaControl
except Exception as e:
    print(f"PandaControl ('frankx' for real-life Franka Emika Panda) is disabled - {e}")

try:
    from .ur5_control import UR5Control
except Exception as e:
    print(f"UR5Control ('ur5' for real-life UR5) is disabled - {e}")
