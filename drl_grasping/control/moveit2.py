from moveit2 import MoveIt2Interface
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
import rclpy


class MoveIt2(MoveIt2Interface):
    def __init__(self, robot_model: str, separate_gripper_controller: bool = True, use_sim_time: bool = True, node_name: str = 'ign_moveit2_py'):
        try:
            rclpy.init()
        except:
            if not rclpy.ok():
                import sys
                sys.exit("ROS 2 could not be initialised")

        super().__init__(robot_model=robot_model,
                         separate_gripper_controller=separate_gripper_controller,
                         use_sim_time=use_sim_time,
                         node_name=node_name)

        self._moveit2_executor = MultiThreadedExecutor(1)
        self._moveit2_executor.add_node(self)
        thread = Thread(target=self._moveit2_executor.spin, args=())
        thread.daemon = True
        thread.start()

    def plan_path(self, **kwargs):
        # Select the default planner
        self.plan_kinematic_path(**kwargs)
