ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base AS dependencies
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build pkg-config \
    libeigen3-dev libceres-dev libsuitesparse-dev libtbb-dev \
    libopencv-dev libapriltag-dev libyaml-cpp-dev \
    ros-${ROS_DISTRO}-rosbag2 ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    ros-${ROS_DISTRO}-sensor-msgs python3-venv \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv /opt/rosbags && /opt/rosbags/bin/pip install --no-cache-dir rosbags==0.10.11
ENV PATH="/opt/rosbags/bin:${PATH}"
WORKDIR /workspace

FROM dependencies AS build
COPY . /workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cmake -S . -B /tmp/kalibr2-build -G Ninja -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 -DKALIBR2_WITH_ROS2=ON && \
    cmake --build /tmp/kalibr2-build -j2 && \
    ctest --test-dir /tmp/kalibr2-build --output-on-failure && \
    cmake --install /tmp/kalibr2-build --prefix /opt/kalibr2
RUN printf '#!/bin/sh\n. /opt/ros/%s/setup.sh\nexec "$@"\n' "$ROS_DISTRO" > /entrypoint-kalibr2.sh && chmod +x /entrypoint-kalibr2.sh
ENTRYPOINT ["/entrypoint-kalibr2.sh"]
CMD ["/opt/kalibr2/bin/kalibr2", "--help"]
