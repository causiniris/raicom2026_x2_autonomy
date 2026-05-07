# base image
FROM ubuntu:22.04

SHELL ["/bin/bash", "-c"]


ARG DEBIAN_FRONTEND=noninteractive
ARG USE_MIRROR=1
ARG MIRROR_URL="mirrors.aliyun.com"
ARG INSTALL_GPP13=1
ARG USERNAME=agi
ARG USER_UID=1001
ARG USER_GID=1001
ARG ROS_MIRROR="tuna"

ENV DEBIAN_FRONTEND=${DEBIAN_FRONTEND}
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV TZ=Etc/UTC


RUN if [ "${USE_MIRROR}" = "1" ]; then \
      cp /etc/apt/sources.list /etc/apt/sources.list.bak && \
      sed -i "s|archive.ubuntu.com|${MIRROR_URL}|g" /etc/apt/sources.list && \
      sed -i "s|security.ubuntu.com|${MIRROR_URL}|g" /etc/apt/sources.list ; \
    fi


RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg lsb-release locales wget sudo ca-certificates && \
    locale-gen en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*


RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-humble-desktop python3-argcomplete python3-colcon-common-extensions \
      ros-humble-grid-map-msgs ros-humble-grid-map-ros ros-humble-rmw-cyclonedds-cpp \
      libgflags-dev libscrypt-dev liboctomap-dev liborocos-kdl-dev \
      libacl1-dev python3-pip cmake unzip  && \
    rm -rf /var/lib/apt/lists/*

RUN pip install anthropic

RUN groupadd -g ${USER_GID} ${USERNAME} || true && \
    useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash ${USERNAME} && \
    usermod -aG sudo ${USERNAME} && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN mkdir -p /agibot/data/var/ && \
    chown -R ${USERNAME}:${USERNAME} /agibot

USER ${USERNAME}
WORKDIR /home/${USERNAME}


RUN echo "export PATH=/home/${USERNAME}/.local/bin:$PATH" >> /home/${USERNAME}/.bashrc

RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc

RUN echo "export ROS_DOMAIN_ID=20" >> /home/${USERNAME}/.bashrc

RUN echo "source /home/${USERNAME}/x2_deploy_workspace/aimdk_msgs/share/aimdk_msgs/local_setup.bash" >> /home//${USERNAME}/.bashrc

CMD ["/bin/bash"]