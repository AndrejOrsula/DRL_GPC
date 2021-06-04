### CUDA 11.2 does not seem to work with Pytorch on some machines (I suspect it is caused by differences in driver version, even if they are included in support matrix)
# ARG PARENT_IMAGE=nvidia/cuda:11.2.1-cudnn8-devel-ubuntu20.04
ARG PARENT_IMAGE=nvidia/cuda:11.0.3-cudnn8-devel-ubuntu20.04
FROM ${PARENT_IMAGE}

# Enable OpenGl for ignition camera sensors (required to create context)
COPY --from=nvidia/opengl:1.2-glvnd-runtime-ubuntu20.04 /usr/local /usr/local
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

### Use bash by default
SHELL ["/bin/bash", "-c"]

### Setup timezone (in order to prevent interactive install)
ARG TIMEZONE=Europe/Copenhagen
ARG DEBIAN_FRONTEND=noninteractive
RUN rm -f /etc/localtime && \
    ln -s /usr/share/zoneinfo/"${TIMEZONE}" /etc/localtime && \
    apt-get update && \
    apt-get install -yq --no-install-recommends \
        locales \
        locales-all \
        tzdata && \
    rm -rf /var/lib/apt/lists/*

### Install toolchain
ARG PYTHON3_VERSION=3.8
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
        apt-utils \
        software-properties-common \
        git \
        wget \
        curl \
        build-essential \
        cmake \
        ninja-build \
        gnupg2 \
        lsb-release \
        python3-pip \
        python3-wheel \
        python${PYTHON3_VERSION} \
        python${PYTHON3_VERSION}-dev \
        libpython${PYTHON3_VERSION}-dev \
        nano && \
    rm -rf /var/lib/apt/lists/*

### Install ros2
ARG ROS2_DISTRO=foxy
ENV ROS2_DISTRO=${ROS2_DISTRO}
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt-get update && \
    apt-get install -yq --no-install-recommends \
        ros-${ROS2_DISTRO}-ros-base && \
    pip3 install \
        argcomplete && \
    rm -rf /var/lib/apt/lists/*

### Install ignition
ARG IGNITION_VERSION=dome
ENV IGNITION_VERSION=${IGNITION_VERSION}
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt-get update && \
    apt-get install -yq --no-install-recommends \
        ignition-${IGNITION_VERSION} && \
    rm -rf /var/lib/apt/lists/*

### Install torch
### Note: torch 1.8.0 does not always play nice with CUDA, restrict to 1.7.1 for now 
RUN pip3 install \
        torch==1.7.1 \
        torchvision==0.8.2 \
        tqdm \
        yacs
# RUN pip3 install \
#         torch \
#         torchvision \
#         tqdm \
#         yacs

### Install stable_baselines3
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
    libopenmpi-dev \
    zlib1g-dev && \
    rm -rf /var/lib/apt/lists/* && \
    pip3 install \
        seaborn \
        optuna \
        stable-baselines3[extra]

### Set working directory for building dependencies
ARG DRL_GRASPING_DIR=/root/drl_grasping
ENV DRL_GRASPING_DIR=${DRL_GRASPING_DIR}
ENV DRL_GRASPING_DEPS_DIR=${DRL_GRASPING_DIR}/deps
WORKDIR ${DRL_GRASPING_DEPS_DIR}

### Build O-CNN
RUN cd ${DRL_GRASPING_DEPS_DIR} && \
    git clone https://github.com/AndrejOrsula/O-CNN.git && \
    cd ${DRL_GRASPING_DEPS_DIR}/O-CNN/pytorch && \
    python3 setup.py install --build_octree && \
    cd ../octree/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DUSE_OPENMP=ON -DUSE_PYTHON=ON -DABI=ON -DKEY64=ON && \
    cmake --build . --config Release

### Build gym_ignition (with its iDynTree dependency)
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
        apt-transport-https \
        clang-10 \
        cmake-curses-gui \
        valgrind \
        libgflags-dev \
        swig \
        python3-numpy \
        libxml2-dev \
        coinor-libipopt-dev \
        libeigen3-dev && \
    cd ${DRL_GRASPING_DEPS_DIR} && \
    git clone https://github.com/robotology/idyntree && \
    mkdir -p ${DRL_GRASPING_DEPS_DIR}/idyntree/build && \
    cd ${DRL_GRASPING_DEPS_DIR}/idyntree/build && \
    cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS:BOOL=OFF -DIDYNTREE_USES_PYTHON=True -DIDYNTREE_USES_IPOPT:BOOL=ON && \
    cmake --build . --target install && \
    cd ${DRL_GRASPING_DEPS_DIR} && \
    git clone https://github.com/AndrejOrsula/gym-ignition.git && \
    mkdir -p ${DRL_GRASPING_DEPS_DIR}/gym-ignition/build && \
    cd ${DRL_GRASPING_DEPS_DIR}/gym-ignition/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && cmake --build . && \
    cmake --build . --target install && \
    cd ${DRL_GRASPING_DEPS_DIR}/gym-ignition && \
    pip3 install . && \
    rm -rf /var/lib/apt/lists/*

### Upgrade numpy (else iDynTree fails)
RUN pip3 install -U \
        numpy

### Install colcon, vcs toolkit and rosdep prior to building ROS-based projects
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://repo.ros2.org/ubuntu/main $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    source /opt/ros/${ROS2_DISTRO}/setup.bash && \
    apt-get update && \
    apt-get install -yq --no-install-recommends \
        python3-colcon-common-extensions \
        python3-vcstool \
        python3-rosdep && \
    rosdep init && \
    rosdep update
# Note: Do not remove apt lists until last ROS-based project is built

### Build moveit2
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
        libompl-dev && \
    mkdir -p ${DRL_GRASPING_DEPS_DIR}/moveit2/src && \
    cd ${DRL_GRASPING_DEPS_DIR}/moveit2/src && \
    git clone https://github.com/ros-planning/moveit2.git && \
    vcs import < moveit2/moveit2.repos ; \
    cd .. && \
    source /opt/ros/${ROS2_DISTRO}/setup.bash && \
    rosdep install -r --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} -y && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

### Set working directory for source (repos) dir
ENV DRL_GRASPING_REPOS_DIR=${DRL_GRASPING_DIR}/repos
WORKDIR ${DRL_GRASPING_REPOS_DIR}

### Install python dependencies for the main project
RUN pip3 install \
        open3d \
        trimesh \
        pcg_gazebo

### Install stable-baselines3 contrib for TQC algo
RUN pip3 install \
        sb3-contrib

ARG DRL_GRASPING_GIT_BRANCH=test_compare_image_cnn
ARG DRL_GRASPING_DOCKER_VERSION=0.12.1
RUN apt-get update && mkdir -p ${DRL_GRASPING_REPOS_DIR}/drl_grasping/src && \
    cd ${DRL_GRASPING_REPOS_DIR}/drl_grasping/src && \
    git clone https://github.com/AndrejOrsula/drl_grasping.git -b ${DRL_GRASPING_GIT_BRANCH} && \
    vcs import < drl_grasping/drl_grasping.repos ; \
    cd .. && \
    source /opt/ros/${ROS2_DISTRO}/setup.bash && \
    source ${DRL_GRASPING_DEPS_DIR}/moveit2/install/local_setup.bash && \
    rosdep install -r --from-paths src -i -y --rosdistro ${ROS_DISTRO} && \
    colcon build --merge-install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*

### Set working project directory
WORKDIR ${DRL_GRASPING_DIR}

### Add entrypoint sourcing the environment
COPY ./entrypoint.bash ./entrypoint.bash
RUN chmod +x ./entrypoint.bash

### Set domain ID for ROS2 in order to not interfere with host
### Note: ROS_LOCALHOST_ONLY prevents MoveIt2 from working, so please do not use it
ENV ROS_DOMAIN_ID 69

ENTRYPOINT ["/bin/bash", "entrypoint.bash"]
CMD ["/bin/bash"]
