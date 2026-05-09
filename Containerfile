FROM docker.io/yahboomtechnology/ros-humble:0.1.0

# 2. 새 사용자 생성 (호스트 유저와 UID/GID 맞춤)
ARG USER_NAME

ARG GCC_VERSION="15"
ARG BINUTILS_VERSION="2.46.0"
ARG GDB_VERSION="17.1"
ARG LLVM_VERSION="22"

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    rm -f /etc/apt/sources.list.d/ros2-latest.list /etc/apt/sources.list.d/ros2.list && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 2. 필수 패키지 및 Nav2 의존성 설치
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
    \
    python3-pip \
    jq \
    ros-humble-xacro \
    # ros-humble-gazebo-ros-pkgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-vision-opencv \
    ros-humble-joint-state-publisher-gui \
    ros-humble-joint-state-publisher \
    # --- [Nav2 관련 패키지 추가] ---
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    # ros-humble-turtlebot3-msgs \
    # ros-humble-turtlebot3-simulations \
    # -----------------------------
    libopencv-dev \
    python3-opencv \
    # 패키지 목록
    automake \
    build-essential \
    ca-certificates \
    curl \
    gettext \
    git \
    gnupg \
    libtool \
    locales \
    lsb-release \
    nano \
    pkg-config \
    perl \
    software-properties-common \
    sudo \
    tar \
    unzip \
    wget \
    xdg-utils \
    zip \
    # -----------------------------
    && \
    rm -rf /var/lib/apt/lists/*0

RUN locale-gen en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN useradd -m -s /bin/bash -G sudo $USER_NAME \
    && echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USER_NAME \
    && chmod 0440 /etc/sudoers.d/$USER_NAME

RUN usermod --shell /bin/bash $USER_NAME

RUN mkdir -p /run/user/1000 && chown -R $USER_NAME:$USER_NAME /run/user/1000

# 1. Gazebo 공식 모델 및 TurtleBot3 모델 미리 다운로드 (바닥 추락 방지)
# RUN mkdir -p /home/$USER_NAME/.gazebo/models && \
#     git clone https://github.com/osrf/gazebo_models.git /home/$USER_NAME/.gazebo/models/gazebo_models_repo && \
#     mv /home/$USER_NAME/.gazebo/models/gazebo_models_repo/* /home/$USER_NAME/.gazebo/models/ && \
#     rm -rf /home/$USER_NAME/.gazebo/models/gazebo_models_repo && \
#     chown -R $USER_NAME:$USER_NAME /home/$USER_NAME/.gazebo

# 최신 GCC 툴체인 설치를 위한 PPA 추가 및 설치
RUN add-apt-repository ppa:ubuntu-toolchain-r/test -y && \
    apt-get update && \
    apt-get install -y --no-install-recommends gcc-$GCC_VERSION g++-$GCC_VERSION && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-$GCC_VERSION 100 --slave /usr/bin/g++ g++ /usr/bin/g++-$GCC_VERSION && \
    apt-get update && \
    apt-get upgrade -y && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 최신 Binutils, GDB 빌드에 필요한 패키지 설치
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    bison \
    flex \
    texinfo \
    zlib1g-dev \
    libzstd-dev \
    libncurses-dev \
    libexpat1-dev \
    libgmp-dev \
    libsource-highlight-dev \
    python3-dev \
    libmpfr-dev \
    liblzma-dev \
    && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 최신 Binutils 빌드 및 설치
RUN cd /tmp && \
    curl -fsSLO https://sourceware.org/pub/binutils/releases/binutils-${BINUTILS_VERSION}.tar.xz && \
    tar -xf binutils-${BINUTILS_VERSION}.tar.xz && \
    mkdir -p binutils-build && \
    cd binutils-build && \
    ../binutils-${BINUTILS_VERSION}/configure --prefix=/usr/local --disable-multilib --disable-werror && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/binutils*

# 최신 GDB 빌드 및 설치
RUN cd /tmp && \
    curl -fsSLO https://ftp.gnu.org/gnu/gdb/gdb-${GDB_VERSION}.tar.xz && \
    tar -xf gdb-${GDB_VERSION}.tar.xz && \
    mkdir -p gdb-build && \
    cd gdb-build && \
    ../gdb-${GDB_VERSION}/configure --prefix=/usr/local --with-python && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/gdb*

# 최신 LLVM 툴체인 설치 (현재 우분투 26.04 저장소를 제공하지 않아서 직접 다운로드하여 설치)
RUN curl -fsSLO https://apt.llvm.org/llvm.sh && \
    chmod +x llvm.sh && \
    ./llvm.sh $LLVM_VERSION && \
    rm llvm.sh

ENV PATH="/usr/lib/llvm-${LLVM_VERSION}/bin:$PATH"

# Ninja 설치
RUN curl -fsSLO https://github.com/ninja-build/ninja/releases/latest/download/ninja-linux-aarch64.zip && \
    unzip -o ninja-linux-aarch64.zip -d /usr/local/bin/ && \
    chmod +x /usr/local/bin/ninja && \
    rm -f ninja-linux-aarch64.zip

# CMake 설치
RUN cd /tmp && \
    curl -fsSLO https://apt.kitware.com/kitware-archive.sh && \
    chmod +x kitware-archive.sh && \
    ./kitware-archive.sh && \
    apt-get update && \
    apt-get install -y --no-install-recommends cmake && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    rm -f kitware-archive.sh

# 환경 변수 자동 로드
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/profile.d/ros.sh \
    && echo "if [ -f /home/$USER_NAME/workspace/install/setup.bash ]; then source /home/$USER_NAME/workspace/install/setup.bash; fi" >> /home/$USER_NAME/.bashrc

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    libcamera-dev \
    libcamera-tools \
    v4l-utils \
    && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libegl-dev libgles-dev libevent-dev \
    libyaml-dev python3-yaml python3-ply \
    libgnutls28-dev openssl \
    python3-pip python3-setuptools python3-wheel \
    && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    pip3 install --upgrade meson jinja2

RUN cd /opt && \
    git clone https://github.com/raspberrypi/libcamera.git && \
    cd libcamera && \
    meson setup build \
        -Dpipelines=rpi/pisp \
        -Dgstreamer=enabled \
        -Dtest=false \
        -Dlc-compliance=disabled \
        -Dcam=disabled \
        -Dqcam=disabled && \
    ninja -C build install && \
    ldconfig

RUN echo "export GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0" >> /etc/profile.d/gst.sh && \
    echo "export GST_PLUGIN_PATH=/usr/local/lib/gstreamer-1.0" >> /etc/profile.d/gst.sh && \
    echo "export LIBCAMERA_IPA_MODULE_PATH=/usr/local/lib/aarch64-linux-gnu/libcamera/ipa" >> /etc/profile.d/libcamera.sh && \
    echo "export GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0" >> /etc/profile.d/gst.sh

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    gstreamer1.0-tools \
    && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]
