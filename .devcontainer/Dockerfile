FROM osrf/ros:noetic-desktop-full

ARG USERNAME=cpu
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# User
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/zsh $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

#  Proxy
RUN sed -i 's|http://archive.ubuntu.com|http://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list \
 && sed -i 's|http://security.ubuntu.com|http://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list

RUN sed -i 's@http://packages.ros.org@https://mirrors.tuna.tsinghua.edu.cn/ros@g' \
 /etc/apt/sources.list.d/ros1-latest.list || true


RUN apt-get update && apt-get install -y \
        zsh \
        curl \
        git \
        sudo \
        build-essential \
        python3-catkin-tools \
        ros-${ROS_DISTRO}-foxglove-bridge \
        ros-${ROS_DISTRO}-web-video-server \
    && rm -rf /var/lib/apt/lists/*

# Install Oh My Zsh and configure plugins
RUN su - cpu -c 'sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended' && \
  su - cpu -c 'git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions' && \
  su - cpu -c 'git clone https://github.com/zsh-users/zsh-syntax-highlighting ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting' && \
  su - cpu -c 'git clone https://github.com/zsh-users/zsh-completions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-completions' && \
  su - cpu -c 'sed -i "s/plugins=(git)/plugins=(git sudo zsh-autosuggestions zsh-syntax-highlighting zsh-completions)/" ~/.zshrc'

# Install CMake 3.18.6
RUN apt-get update && apt-get install -y build-essential libssl-dev wget && \
    wget https://github.com/Kitware/CMake/releases/download/v3.18.6/cmake-3.18.6.tar.gz -O /tmp/cmake.tar.gz && \
    cd /tmp && tar -xzf cmake.tar.gz && cd cmake-3.18.6 && \
    ./bootstrap --prefix=/usr/local && make -j$(nproc) && make install && \
    cd /tmp && rm -rf cmake-3.18.6 cmake.tar.gz

# Install OSQP and OSQP-Eigen
RUN cd /tmp && \
    # Clone OSQP
    git clone https://github.com/osqp/osqp.git && \
    cd osqp && \
    git checkout v0.6.2 && \
    git submodule update --init --recursive && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd /tmp && rm -rf osqp && \
    \
    # Clone OSQP-Eigen
    git clone https://github.com/robotology/osqp-eigen.git && \
    cd osqp-eigen && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd /tmp && rm -rf osqp-eigen


# Configure the .zshrc file for cpu
RUN echo "\n# auto source of ros1\nsource /opt/ros/${ROS_DISTRO}/setup.zsh\n" >> /home/cpu/.zshrc && \
    echo "# auto source uav_motion_planning workspace\nif [ -f /home/cpu/uav_motion_planning/devel/setup.zsh ]; then source /home/cpu/uav_motion_planning/devel/setup.zsh; fi" >> /home/cpu/.zshrc && \
    echo "# Disable Zsh nomatch behavior\nsetopt nonomatch" >> /home/cpu/.zshrc


USER $USERNAME

WORKDIR /home/$USERNAME/uav_motion_planning