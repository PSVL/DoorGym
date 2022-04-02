FROM nvidia/cuda:11.2.0-cudnn8-runtime-ubuntu20.04

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic

###################################### user #####################################

ENV SHELL=/bin/bash \
    USER=kl \
    UID=1000 \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8

ENV HOME=/home/${USER}

RUN adduser --disabled-password \
    --gecos "Default user" \
    --uid ${UID} \
    ${USER} 

RUN echo "root:root" | chpasswd
RUN echo "${USER}:111111" | chpasswd

###################################### install python3.6 #####################################

RUN apt-get update && apt-get install -y software-properties-common gcc \
    && add-apt-repository -y ppa:deadsnakes/ppa

RUN apt-get update && apt-get install -y python3.6 python3-distutils python3-pip python3-apt
RUN rm -rf /usr/bin/python3
RUN ln -s /usr/bin/python3.6 /usr/bin/python3

###################################### basic tools #####################################

RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -yq dist-upgrade \
    && apt-get -o Acquire::ForceIPv4=true install -yq --no-install-recommends \
    locales \
    cmake \
    make \
    git \
    vim \
    gedit \
    wget \
    sudo \
    lsb-release \
    build-essential \
    net-tools \
    dirmngr \
    gnupg2 \
    mercurial \
    libopencv-dev \
    libv4l-dev \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    libpcap-dev \
    python3-pip \
    python3-setuptools \
    python3-opencv \
    python3-numpy \
    python3-empy  \
    python3-lxml \
    python3-pymodbus \
    terminator \
    screen \
    openssh-server \
    libglfw3-dev \
    iputils-ping \
    libosmesa6-dev \
    libgl1-mesa-glx \
    libglfw3 \
    libglew-dev \
    libopenmpi-dev patchelf \
    python3.6-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

###################################### ROS #####################################

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# install bootstrap tools
RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -o Acquire::ForceIPv4=true install --no-install-recommends -y \
    ros-$ROS_DISTRO-desktop-full \
    ros-$ROS_DISTRO-soem \
    ros-$ROS_DISTRO-openslam-gmapping \
    ros-$ROS_DISTRO-geodesy \
    ros-$ROS_DISTRO-ddynamic-reconfigure \
    ros-$ROS_DISTRO-perception \
    ros-$ROS_DISTRO-rgbd-launch \
    ros-$ROS_DISTRO-pointcloud-to-laserscan \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-spacenav-node \
    ros-$ROS_DISTRO-tf2-sensor-msgs \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-twist-mux \
    ros-$ROS_DISTRO-velodyne-simulator \
    ros-$ROS_DISTRO-gazebo-ros \
    ros-$ROS_DISTRO-move-base-msgs \
    ros-$ROS_DISTRO-lms1xx \
    ros-$ROS_DISTRO-rviz-imu-plugin \
    ros-$ROS_DISTRO-interactive-marker-twist-server \
    ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-rosserial-arduino \
    ros-$ROS_DISTRO-rosserial \
    ros-$ROS_DISTRO-socketcan-interface \
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-moveit-planners-ompl \
    ros-$ROS_DISTRO-moveit-* \
    ros-$ROS_DISTRO-ur-* \
    ros-$ROS_DISTRO-velocity-controllers \
    ros-$ROS_DISTRO-joint-state-controller \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-gazebo-ros-control \
    ros-$ROS_DISTRO-geographic-msgs \
    ros-$ROS_DISTRO-force-torque-sensor-controller \
    ros-$ROS_DISTRO-industrial-msgs \
    ros-$ROS_DISTRO-effort-controllers \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init \
    && rosdep update

##################################### PIP3 ######################################

RUN pip3 install --upgrade pip setuptools

RUN pip3 install \
    rospkg \
    catkin-tools \
    matplotlib \
    pandas \
    pypozyx \
    requests \
    scikit-image \
    scikit-learn \
    gdown \
    seaborn \
    easydict \
    lmdb \  
    shapely \
    wandb \
    virtualenv \
    pytest \
    -U "mujoco-py<2.2,>=2.1"

####################################### procman ###########################################

RUN cd ${HOME} && git clone https://github.com/lcm-proj/lcm \
    && cd lcm \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make install

RUN cd ${HOME} && git clone https://github.com/ARG-NCTU/procman \
    && cd procman \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make install

####################################### mujoco ###########################################

RUN cd ${HOME} \
    && mkdir .mujoco \
    && cd .mujoco \
    && wget "https://github.com/deepmind/mujoco/releases/download/2.1.0/mujoco210-linux-x86_64.tar.gz"\
    && tar zxvf mujoco210-linux-x86_64.tar.gz\
    && cd mujoco210 \ 
    && cd bin/ && wget "https://www.roboti.us/file/mjkey.txt" \
    && cp mjkey.txt ~/.mujoco

RUN pip3 install \
    absl-py==0.8.0 \
    alabaster==0.7.12 \
    astor==0.8.0 \
    atari-py==0.2.6 \
    atomicwrites==1.3.0 \
    attrs==19.1.0 \
    babel==2.7.0 \
    backcall==0.1.0 \
    chardet==3.0.4 \
    click==7.0 \
    cloudpickle==1.2.1 \
    cycler==0.10.0 \
    decorator==4.4.0 \
    dill==0.3.0 \
    docutils==0.15.2 \
    future==0.17.1 \
    gast==0.2.2 \
    glfw==1.8.3 \
    google-pasta==0.1.7 \
    gtimer==1.0.0b5 \
    gym==0.14.0 \
    h5py==2.9.0 \
    idna==2.8 \
    imagehash==4.0 \
    imageio==2.5.0 \
    imagesize==1.1.0 \
    importlib-metadata==0.20 \
    ipython==7.8.0 \
    ipython-genutils==0.2.0 \
    jedi==0.15.1 \
    jinja2==2.10.1 \
    joblib==0.13.2 \
    keras-applications==1.0.8 \
    keras-preprocessing==1.1.0 \
    kiwisolver==1.1.0 \
    lockfile==0.12.2 \
    markdown==3.1.1 \
    markupsafe==1.1.1 \
    matplotlib==3.1.1 \
    more-itertools==7.2.0 \
    numpydoc==0.9.1 \
    packaging==19.1 \
    parso==0.5.1 \
    pexpect==4.7.0 \
    pickleshare==0.7.5 \
    pluggy==0.12.0 \
    progressbar2==3.43.1 \
    prompt-toolkit==2.0.9 \
    protobuf==3.9.1 \
    ptyprocess==0.6.0 \
    py==1.8.0 \
    pyglet==1.3.2 \
    pygments==2.4.2 \
    pyparsing==2.4.2 \
    pytest==5.1.2 \
    pytest-instafail==0.3.0 \
    python-dateutil==2.8.0 \
    python-utils==2.3.0 \
    pytz==2019.2 \
    pywavelets==1.0.3 \
    pyzmq==18.1.0 \
    requests==2.22.0 \
    snowballstemmer==1.9.0 \
    sphinx==2.2.0 \
    sphinx-rtd-theme==0.4.3 \
    sphinxcontrib-applehelp==1.0.1 \
    sphinxcontrib-devhelp==1.0.1 \
    sphinxcontrib-htmlhelp==1.0.2 \
    sphinxcontrib-jsmath==1.0.1 \
    sphinxcontrib-qthelp==1.0.2 \
    sphinxcontrib-serializinghtml==1.1.3 \
    tensorboard==1.14.0 \
    tensorboardx==1.8 \
    tensorflow-estimator==1.14.0 \
    termcolor==1.1.0 \
    tqdm==4.35.0 \
    traitlets==4.3.2 \
    urllib3==1.25.3 \
    wcwidth==0.1.7 \
    werkzeug==0.15.6 \
    wrapt==1.11.2 \
    zipp==0.6.0 \
    zmq==0.0.0 \
    numpy==1.12.0 

RUN pip3 install tensorflow==2.4.0

####################################### openai gym ###########################################

RUN cd ${HOME} && git clone https://github.com/openai/baselines \
    && cd baselines \
    && pip3 install -e .

##################################### Pytorch ######################################

RUN python3 -m pip install -U pip 

RUN python3 -m pip install \
    torch==1.8.2+cu111 torchvision==0.9.2+cu111 torchaudio==0.8.2 -f https://download.pytorch.org/whl/lts/1.8/torch_lts.html

##################################### setting #####################################

RUN pip3 install netifaces==0.11.0

RUN chown -R ${USER}:${USER} ${HOME}/
RUN echo "kl ALL=(ALL)  ALL" > /etc/sudoers

RUN echo "source /opt/ros/noetic/setup.bash" >> ${HOME}/.bashrc

# Support of nvidia-docker 2.0
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

USER ${USER}

WORKDIR ${HOME}