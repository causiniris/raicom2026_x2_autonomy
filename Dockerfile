# base image
FROM ubuntu:22.04

SHELL ["/bin/bash", "-c"]

# --------------------
# configurable parameters
# --------------------
ARG DEBIAN_FRONTEND=noninteractive
ARG USE_MIRROR=1
ARG MIRROR_URL="mirrors.aliyun.com"
ARG INSTALL_GPP13=1
ARG USERNAME=agi
ARG UID=1001
ARG GID=1001
ARG ROS_MIRROR="tuna"

ENV DEBIAN_FRONTEND=${DEBIAN_FRONTEND}
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV TZ=Etc/UTC

# --------------------
# 1. 自动换源 (阿里云)
# --------------------
RUN if [ "${USE_MIRROR}" = "1" ]; then \
      cp /etc/apt/sources.list /etc/apt/sources.list.bak && \
      sed -i "s|archive.ubuntu.com|${MIRROR_URL}|g" /etc/apt/sources.list && \
      sed -i "s|security.ubuntu.com|${MIRROR_URL}|g" /etc/apt/sources.list ; \
    fi

# --------------------
# 2. 基础环境 + 小工具
# --------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg lsb-release locales wget sudo ca-certificates && \
    locale-gen en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

# --------------------
# 3. 安装 ROS2 Humble + 你的项目特定依赖
# --------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-humble-desktop python3-argcomplete python3-colcon-common-extensions \
      ros-humble-grid-map-msgs ros-humble-grid-map-ros ros-humble-rmw-cyclonedds-cpp \
      libgflags-dev libscrypt-dev liboctomap-dev liborocos-kdl-dev \
      libacl1-dev python3-pip cmake && \
    rm -rf /var/lib/apt/lists/*


# --------------------
# 4. 配置非 root 用户
# --------------------
RUN groupadd -g ${GID} ${USERNAME} || true && \
    useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} && \
    usermod -aG sudo ${USERNAME} && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN mkdir -p /agibot/data/var/ && \
    chown -R ${USERNAME}:${USERNAME} /agibot

USER ${USERNAME}
WORKDIR /home/${USERNAME}


# 环境变量
# add /home/agi/.local/bin to PATH for cmake installation
RUN echo "export PATH=/home/${USERNAME}/.local/bin:$PATH" >> /home/${USERNAME}/.bashrc

# automatically source ROS environment in non-root user's bashrc for interactive use
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc

# add ROS_DOMAIN_ID=20 to avoid DDS conflicts in multi-robot scenarios
RUN echo "export ROS_DOMAIN_ID=20" >> /home/${USERNAME}/.bashrc

RUN echo "source /home/${USERNAME}/x2_deploy_workspace/mc/build/install/share/aimdk_msgs/local_setup.bash" >> /home//${USERNAME}/.bashrc

CMD ["/bin/bash"]