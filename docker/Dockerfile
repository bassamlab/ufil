FROM ros:jazzy

LABEL maintainer="schaefer@embedded.rwth-aachen.de"
LABEL version="0.1"
LABEL description="Docker image for development of Ufil with ROS Jazzy ."

# Install ros dependencies
RUN apt-get update && apt-get install -y\
    ros-jazzy-desktop-full \
    ros-jazzy-udp-msgs \
    ros-jazzy-etsi-its-messages \
    ros-jazzy-lanelet2 \
    ros-jazzy-derived-object-msgs \
    ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-rmw-cyclonedds-cpp \
    && apt-get clean && rm -rf /var/lib/apt/lists/\* /tmp/\* /var/tmp/*

# Install apt dependencies
RUN apt-get update && apt-get install -y \
    libtbb-dev \
    libboost-dev \
    libpaho-mqttpp-dev \
    libpaho-mqtt-dev \
    liboscpack-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/\* /tmp/\* /var/tmp/*

# Install python dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    && apt-get clean && rm -rf /var/lib/apt/lists/\* /tmp/\* /var/tmp/*

RUN pip3 install --no-cache-dir --break-system-packages \
    casadi \
    h5py \
    progressbar \
    ultralytics \
    ultralytics-thop \
    pandas \
    numpy \
    matplotlib \
    pyyaml \
    pillow \
    psutil \
    requests \
    tqdm \
    scipy \
    seaborn \
    stonesoup \
    scikit-image \
    scikit-learn \
    python-dateutil \
    pytz \
    six \
    threadpoolctl \
    tzdata

# Setup workspace
WORKDIR /workspace

# Setup terminal sessions
COPY .bashrc /root/.bashrc
