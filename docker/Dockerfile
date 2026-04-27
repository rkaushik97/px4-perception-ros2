# Dockerfile for px4-perception-ros2
#
# Extends the prepared px4-sim base image (ROS2 Jazzy + Gazebo Harmonic +
# PX4 SITL on Ubuntu 24.04, see https://github.com/erdemuysalx/px4-sim) with
# the Python machine-learning stack required by the YOLO detector node, so
# a fresh container has everything ready out of the box — no host-side venv
# or pip install dance required.
#
# Build (from this directory or via docker-compose):
#   docker build -t px4-perception:latest -f docker/Dockerfile .
#
# Run (with GPU and project bind-mount, see docker-compose.override.yml):
#   docker compose up -d

FROM erdemuysalx/px4-sitl:latest

# Avoid tzdata / debconf prompts during apt operations.
ENV DEBIAN_FRONTEND=noninteractive

# 1. System packages.
#    python3-pip: needed because the ML stack below is installed via pip.
#    rosdep / colcon: present in the base image, reinstalling is a no-op
#    but guards against thinner upstream images.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-rosdep \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 2. Python ML stack, installed system-wide.
#    --break-system-packages bypasses Ubuntu 24.04's PEP 668 lock; safe in a
#    container.
#    Torch comes from the CUDA 12.4 wheel index for actual GPU support.
RUN pip3 install --no-cache-dir --break-system-packages \
        --index-url https://download.pytorch.org/whl/cu124 \
        torch==2.5.1 \
        torchvision==0.20.1

# 3. Ultralytics, with numpy pinned to <2 in the SAME resolve.
#    cv_bridge in ROS2 Jazzy is built against the numpy 1.x ABI and segfaults
#    with numpy 2.x. Constraining numpy<2 alongside ultralytics ensures pip
#    never tries to upgrade past 1.x — which would also fail to uninstall the
#    debian-managed numpy (no RECORD file, see PEP 668 / dpkg interaction).
#    --ignore-installed numpy makes pip install a fresh numpy under its own
#    control instead of trying to mutate the debian-owned one.
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed numpy \
        "numpy<2" ultralytics

# 4. Project workdir.
#    The host repo is bind-mounted here by docker-compose.override.yml,
#    so colcon builds and edits land directly on the host filesystem.
WORKDIR /root/px4-perception-ros2

# Drop back to the base image's default entrypoint / cmd.
