from gym_ignition import base
from gym_ignition.base import runtime
from gym_ignition.base import runtime
from gym_ignition.utils import logger
from gym_ignition.utils.typing import Action, Reward, Observation
from gym_ignition.utils.typing import SeedList
from gym_ignition.utils.typing import State, Action, Observation, Done, Info
from pynput import keyboard
import numpy as np
import sys
import time


class RealEvaluationRuntimeManual(runtime.Runtime):
    """
     Implementation of :py:class:`~gym_ignition.base.runtime.Runtime` for execution
     on trained agent on real-life Franka Emika Panda (evaluation only!!!).
     Implementation requires manual reset of the workspace as well as manual logging
     of success rate.
     Furthere, this runtime allows manual stepping of the execution (safe mode).

     Note: this class is currently implemented for GraspOctree task and its variants.
     Might work fine for others (e.g. Reach), but it is not tested.
    """

    def __init__(self,
                 task_cls: type,
                 agent_rate: float,
                 manual_steps: bool = False,
                 **kwargs):

        # Force use of frankx controller backend (real-life Franka Emika Panda)
        kwargs['robot_controller_backend'] = 'ur5'

        # Create the Task object
        task = task_cls(agent_rate=agent_rate, **kwargs)

        if not isinstance(task, base.task.Task):
            raise RuntimeError("The task is not compatible with the runtime")

        # Wrap the task with the runtime
        super().__init__(task=task, agent_rate=agent_rate)

        # Initialise spaces
        self.action_space, self.observation_space = self.task.create_spaces()
        # Store the spaces also in the task
        self.task.action_space = self.action_space
        self.task.observation_space = self.observation_space

        # Seed the environment
        self.seed()

        # Other parameters
        self._manual_steps = manual_steps
        if self._manual_steps:
            print("Safety feature for manual stepping is enabled. "
                  "Enter will need to be pressed for each step")
        print("Press 'ESC' to terminate.")
        print("Press 'd' once episode is done. Success must be logged manually.")

        # Initialise flags that are set manually
        self._manual_done = False
        self._manual_terminate = False

        # Register keyboard listener to manually trigger events (task is done, etc...)
        listener = keyboard.Listener(on_press=self.on_press)
        listener.start()

        # Initialise start time
        self.running_time = 0.0
        self.start_time = time.time()

    # =================
    # Runtime interface
    # =================

    def timestamp(self, running_time: bool = True) -> float:

        if running_time:
            return self.running_time
        else:
            return time.time() - self.start_time

    # =================
    # gym.Env interface
    # =================

    def step(self, action: Action) -> State:

        if self._manual_steps:
            input("Press Enter to continue...")

        if self._manual_terminate:
            print("Terminating")
            sys.exit()

        # Time the execution of the step to determine running time
        pre_step_time = time.time()

        # Set the action
        if not self._manual_done:
            if not self.action_space.contains(action):
                logger.warn("The action does not belong to the action space")
            self.task.set_action(action)

            print("Action finished")

        # Get the observation
        observation = self.task.get_observation()
        assert isinstance(observation, np.ndarray)
        if not self.observation_space.contains(observation):
            logger.warn("The observation does not belong "
                        "to the observation space")

        # Get the reward
        # Note: Automatic reward function in real world is not yet implemented
        # reward = self.task.get_reward()
        # assert isinstance(reward, float), "Failed to get the reward"
        reward = 0.0

        # Check termination
        # Note: Automatic done checking is not yet implemented for real world
        # done = self.task.is_done()
        done = self._manual_done

        # Get info
        # Note: There is currently no info relevant for real world use
        # info = self.task.get_info()
        info = {}

        # Update running time
        self.running_time += time.time() - pre_step_time

        # Return state
        return State((Observation(observation), Reward(reward), Done(done), Info(info)))

    def reset(self) -> Observation:
        input("Episode done, please reset the workspace")
        time.sleep(10.0)
        # Move home (specified for the used frankx)
        self.task.robot_controller.move_home()
        input("Press Enter once the physical workspace is reset...")

        self._manual_done = False

        # Reset the task
        self.task.reset_task()


        # Get the observation
        observation = self.task.get_observation()
        assert isinstance(observation, np.ndarray)
        if not self.observation_space.contains(observation):
            logger.warn("The observation does not belong "
                        "to the observation space")

        return Observation(observation)

    def seed(self, seed: int = None) -> SeedList:
        # Seed the task
        seed = self.task.seed_task(seed)
        return seed

    def render(self, mode='human'):
        pass

    def close(self):
        pass

    def on_press(self, key):
        print('')
        if keyboard.KeyCode.from_char('d') == key:
            print("Manual 'done' signal received")
            self._manual_done = True

        if keyboard.Key.esc == key:
            print("'terminate' signal received")
            self._manual_terminate = True
