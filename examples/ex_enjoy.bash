#!/usr/bin/env bash

## Random seed to use for both the environment and agent (-1 for random)
SEED="77"

## ID of the environment
## Reach
# ENV_ID="Reach-Gazebo-v0"
# ENV_ID="Reach-ColorImage-Gazebo-v0"
# ENV_ID="Reach-Octree-Gazebo-v0"
# ENV_ID="Reach-OctreeWithColor-Gazebo-v0"
## Grasp
# ENV_ID="Grasp-Gazebo-v0"
# ENV_ID="Grasp-Octree-Gazebo-v0"
ENV_ID="Grasp-OctreeWithColor-Gazebo-v0"

## Robot model
ROBOT_MODEL="panda"
# ROBOT_MODEL="ur5_rg2"

## Algorithm to use
# ALGO="sac"
# ALGO="td3"
ALGO="tqc"

## Checkpoint to load
CHECKPOINT=0

## Path the parent training directory
TRAINING_DIR="training"
## Path to logs
LOG_DIR=""${TRAINING_DIR}"/"${ENV_ID}"_real_eval/logs"
## Path to tensorboard logs
TENSORBOARD_LOG_DIR=""${TRAINING_DIR}"/"${ENV_ID}"/tensorboard_logs"
## Path to reward logs (enjoy)
REWARD_LOG_DIR=""${TRAINING_DIR}"/"${ENV_ID}"/reward_logs"

## Extra arguments to be passed into the script
EXTRA_ARGS=""

########################################################################################################################
########################################################################################################################

# ## Spawn ign_moveit2 subprocess in background, while making sure to forward termination signals
# REAL_CMD="ros2 launch drl_grasping real.launch.py"
# echo "Launching real runtime (camera and stuff) in background:"
# echo "${REAL_CMD}"
# echo ""
# ${REAL_CMD} &
# ## Kill all subprocesses when SIGINT SIGTERM EXIT are received
# subprocess_pid_ign_moveit2="${!}"
# terminate_subprocesses() {
#     echo "INFO: Caught signal, killing all subprocesses..."
#     pkill -P "${subprocess_pid_ign_moveit2}"
# }
# trap 'terminate_subprocesses' SIGINT SIGTERM EXIT ERR

## Arguments
ENJOY_ARGS="--env "${ENV_ID}" --algo "${ALGO}" --seed "${SEED}" --folder "${LOG_DIR}" --reward-log "${REWARD_LOG_DIR}""
## Add trained agent to args in order to continue training
if [ ! -z "${CHECKPOINT}" ]; then
    ENJOY_ARGS=""${ENJOY_ARGS}" --load-checkpoint "${CHECKPOINT}""
fi

## Execute enjoy script
ENJOY_CMD="ros2 run drl_grasping enjoy.py "${ENJOY_ARGS}""
echo "Executing enjoy command:"
echo "${ENJOY_CMD}"
echo ""
${ENJOY_CMD}
