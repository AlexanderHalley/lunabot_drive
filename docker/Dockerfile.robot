# The deployable rover: the whole workspace built into one image.
#
#   docker build -f docker/Dockerfile.robot -t lunabot/robot .      # from the repo root
#   docker compose -f docker/compose.robot.yml up -d                # on the rover
#
# Built for linux/arm64 (the Jetson Orin) and linux/amd64 (the desktop, where
# the same image is run against Isaac before it goes on the rover) -- CI builds
# both natively and pushes them to ghcr.io. See docs/DEPLOYMENT.md.
#
# Why Docker on the Orin at all: JetPack 6 is Ubuntu 22.04, whose ROS is Humble,
# and this workspace is Jazzy (24.04). The container brings its own 24.04
# userspace. Nothing in the stack needs the GPU today (rtabmap is CPU, the OAK-D
# computes depth on the camera, cuVSLAM is not in Jazzy), so no NVIDIA runtime
# is involved; when something does, see docs/DEPLOYMENT.md "GPU".
#
# Unlike Dockerfile.jazzy (the dev/CI image, source bind-mounted), this one
# COPIES the source and builds it in: the image is the deployment unit, and a
# tag names exactly what runs.

ARG ROS_DISTRO=jazzy

# ---------------------------------------------------------------------------
# deps: ROS + every rosdep dependency + the vcs sources. Only the manifests and
# lunabot.repos are copied, so a code change does not re-run apt.
# ---------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base AS deps

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-vcstool \
      python3-rosdep \
      # SocketCAN debugging on the rover: candump, ip link.
      can-utils \
      iproute2 \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/lunabot_ws
COPY lunabot.repos /opt/lunabot_ws/lunabot.repos
# The manifests, at their package paths: rosdep reads them, and keeping the
# layout means the later COPY of the source lands on top unchanged.
COPY src/lunabot_msgs/package.xml src/lunabotics/src/lunabot_msgs/package.xml
COPY src/lunabot_description/package.xml src/lunabotics/src/lunabot_description/package.xml
COPY src/lunabot_hardware/package.xml src/lunabotics/src/lunabot_hardware/package.xml
COPY src/lunabot_bringup/package.xml src/lunabotics/src/lunabot_bringup/package.xml
COPY src/lunabot_perception/package.xml src/lunabotics/src/lunabot_perception/package.xml
COPY src/lunabot_slam/package.xml src/lunabotics/src/lunabot_slam/package.xml
COPY src/lunabot_navigation/package.xml src/lunabotics/src/lunabot_navigation/package.xml
COPY src/lunabot_excavation/package.xml src/lunabotics/src/lunabot_excavation/package.xml
COPY src/lunabot_sim/package.xml src/lunabotics/src/lunabot_sim/package.xml

# No apt "Recommends" for anything rosdep installs: they were most of the
# image -- Java, four LLVMs, Vulkan drivers, a speech recogniser -- and the
# rover runs none of them. The mock smoke test and the Isaac run prove what
# is left is enough (docs/DEPLOYMENT.md).
RUN echo 'APT::Install-Recommends "false";' > /etc/apt/apt.conf.d/99-no-recommends

# Same skip-keys as CI and the README (lunabot.repos explains both).
RUN mkdir -p src/third_party \
    && vcs import src/third_party < lunabot.repos \
    && apt-get update \
    && rosdep update --rosdistro=${ROS_DISTRO} \
    && rosdep install --from-paths src --ignore-src -y \
         --skip-keys "sparkcan isaac_ros_visual_slam" \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# build: compile into /opt/lunabot. Merged and NOT symlinked: the install tree
# must stand alone once the source is gone (colcon_defaults.yaml's
# symlink-install is for development and is not read here).
# ---------------------------------------------------------------------------
FROM deps AS build

COPY src src/lunabotics/src
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build \
         --merge-install \
         --install-base /opt/lunabot \
         --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
         --event-handlers console_cohesion+

# ---------------------------------------------------------------------------
# runtime: deps (so every library rosdep installed is present) + the install
# tree. Not slimmed to exec-only dependencies yet: a missing runtime library is
# a rover that does not start, and the size saved is not worth that risk until
# CI can prove the slim image runs. The source and build tree stay behind.
# ---------------------------------------------------------------------------
FROM deps AS runtime

COPY --from=build /opt/lunabot /opt/lunabot
COPY docker/cyclonedds.xml /etc/lunabot/cyclonedds.xml
COPY --chmod=755 docker/robot_entrypoint.sh /usr/local/bin/robot_entrypoint.sh
# `docker compose exec stack bash` skips the entrypoint; source ROS there too.
RUN printf 'source /opt/ros/%s/setup.bash\nsource /opt/lunabot/setup.bash\n' "${ROS_DISTRO}" >> /root/.bashrc

# Root inside the container: SocketCAN, the OAK-D's USB device and the joystick
# all need device access, and the rover runs nothing else. Bags written to the
# mounted data directory are therefore root-owned on the host.
ENV ROS_DOMAIN_ID=42 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///etc/lunabot/cyclonedds.xml \
    LUNABOT_LAUNCH="robot.launch.py hw:=mock"

# Which commit this image is, for `docker inspect` and the logs.
ARG GIT_SHA=unknown
ENV LUNABOT_GIT_SHA=${GIT_SHA}
LABEL org.opencontainers.image.source="https://github.com/AlexanderHalley/lunabot_drive" \
      org.opencontainers.image.revision="${GIT_SHA}" \
      org.opencontainers.image.description="Lunabot 2027 ROS 2 Jazzy stack (rover runtime)"

WORKDIR /root
ENTRYPOINT ["/usr/local/bin/robot_entrypoint.sh"]
# Empty CMD: the entrypoint launches $LUNABOT_LAUNCH. Pass a command to run
# that instead, e.g. `docker run ... lunabot/robot bash`.
CMD []
