#!/usr/bin/env bash

## Random seed to use for both the environment and agent (-1 for random)
SEED="42"

## ID of the environment
## Reach
# ENV_ID="Reach-Gazebo-v0"
# ENV_ID="Reach-ColorImage-Gazebo-v0"
# ENV_ID="Reach-Octree-Gazebo-v0"
# ENV_ID="Reach-OctreeWithColor-Gazebo-v0"
## Grasp
# ENV_ID="Grasp-Gazebo-v0"
# ENV_ID="Grasp-Octree-Gazebo-v0"
# ENV_ID="Grasp-OctreeWithColor-Gazebo-v0"
# ENV_ID="Grasp-ColorImage-Gazebo-v0"
ENV_ID="Grasp-RgbdImage-Gazebo-v0"

## Robot model
ROBOT_MODEL="panda"
# ROBOT_MODEL="ur5_rg2"

## Algorithm to use
# ALGO="sac"
# ALGO="td3"
ALGO="tqc"

## Path to trained agent (to continue training)
# TRAINED_AGENT=""${ENV_ID}"_1/rl_model_0000_steps.zip"

## Path to a replay buffer that should be preloaded before training begins
# PRELOAD_REPLAY_BUFFER="training/preloaded_buffers/"${ENV_ID}"_1/replay_buffer.pkl"

## Continuous evaluation (-1 to disable)
EVAL_FREQUENCY=-1
EVAL_EPISODES=10

## Path the parent training directory
TRAINING_DIR="training"
## Path to logs
LOG_DIR=""${TRAINING_DIR}"/"${ENV_ID}"/logs"
## Path to tensorboard logs
TENSORBOARD_LOG_DIR=""${TRAINING_DIR}"/"${ENV_ID}"/tensorboard_logs"

## Extra arguments to be passed into the script
EXTRA_ARGS=""
# EXTRA_ARGS="--save-replay-buffer"

########################################################################################################################
########################################################################################################################

## Spawn ign_moveit2 subprocess in background, while making sure to forward termination signals
IGN_MOVEIT2_CMD="ros2 launch drl_grasping ign_moveit2_headless.launch.py"
if [ "$ROBOT_MODEL" = "ur5_rg2" ]; then
    IGN_MOVEIT2_CMD="ros2 launch drl_grasping ign_moveit2_headless_ur5_rg2.launch.py"
fi
echo "Launching ign_moveit2 in background:"
echo "${IGN_MOVEIT2_CMD}"
echo ""
${IGN_MOVEIT2_CMD} &
## Kill all subprocesses when SIGINT SIGTERM EXIT are received
subprocess_pid_ign_moveit2="${!}"
terminate_subprocesses() {
    echo "INFO: Caught signal, killing all subprocesses..."
    pkill -P "${subprocess_pid_ign_moveit2}"
}
trap 'terminate_subprocesses' SIGINT SIGTERM EXIT ERR

## Arguments
TRAIN_ARGS="--env "${ENV_ID}" --algo "${ALGO}" --seed "${SEED}" --log-folder "${LOG_DIR}" --tensorboard-log "${TENSORBOARD_LOG_DIR}" --eval-freq "${EVAL_FREQUENCY}" --eval-episodes "${EVAL_EPISODES}" "${EXTRA_ARGS}""
## Add trained agent to args in order to continue training
if [ ! -z "${TRAINED_AGENT}" ]; then
    TRAIN_ARGS=""${TRAIN_ARGS}" --trained-agent "${LOG_DIR}"/"${ALGO}"/"${TRAINED_AGENT}""
fi
## Add preload replay buffer to args in order to preload buffer with transitions that use custom heuristic (demonstration)
if [ ! -z "${PRELOAD_REPLAY_BUFFER}" ]; then
    TRAIN_ARGS=""${TRAIN_ARGS}" --preload-replay-buffer "${PRELOAD_REPLAY_BUFFER}""
fi

## Execute train script
TRAIN_CMD="ros2 run drl_grasping train.py "${TRAIN_ARGS}""
echo "Executing train command:"
echo "${TRAIN_CMD}"
echo ""
${TRAIN_CMD}
