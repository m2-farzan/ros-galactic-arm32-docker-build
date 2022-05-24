FROM ubuntu:20.04 AS common

RUN apt update && \
    apt upgrade -y && \
    apt install -y wget g++ make cmake libacl1-dev libssl-dev libncurses-dev

RUN c_rehash /etc/ssl/certs

WORKDIR /root

RUN wget https://mfarzan.ir/cmake-bin.tar.gz && \
    tar -xzvf cmake-bin.tar.gz && \
    mv /root/cmake-bin /root/cmake-3.23.1 && \
    cd cmake-3.23.1 && \
    make install && \
    rm /root/cmake-*

RUN apt install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

RUN apt install -y curl gnupg lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update

RUN apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget

RUN python3 -m pip install -U \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  setuptools \
  rosinstall-generator

WORKDIR /root/ros2

RUN mkdir src && \
    rosinstall_generator ros_base --deps --rosdistro galactic > base.repos && \
    vcs import src < base.repos

RUN rosdep init && \
    rosdep update

FROM common AS build

RUN rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers"

RUN mkdir -p /opt/ros/galactic && \
    colcon build --executor sequential --merge-install --install-base /opt/ros/galactic

FROM common

RUN rosdep install --dependency-types exec --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers"

COPY --from=build /opt/ros/galactic /opt/ros/galactic

