# syntax=docker/dockerfile:1
#
# Stages:
#   system       ROS + system packages (shared base)
#   python-lock  only used by update-python-lock.sh to regenerate constraints.txt
#   runtime      image used by docker-compose.yaml
#   dev          devcontainer image (runtime + non-root user and dev tooling)

# Pinned by digest (multi-arch index) for reproducible builds. Update deliberately:
#   docker buildx imagetools inspect ros:jazzy
ARG ROS_IMAGE=ros:jazzy@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617

FROM ${ROS_IMAGE} AS system

# ROS, system and Python tooling packages; shared with CI (.gitlab-ci.yml)
COPY install-system.sh /tmp/install/
RUN /tmp/install/install-system.sh

# ---------------------------------------------------------------------------
FROM system AS python-lock

COPY requirements.txt /tmp/pip/
# torch is resolved together with the setuptools pin (newer torch requires setuptools>=77)
RUN pip3 install --no-cache-dir --break-system-packages \
        --index-url https://download.pytorch.org/whl/cpu \
        "setuptools<72" torch torchvision \
    && pip3 install --no-cache-dir --break-system-packages -r /tmp/pip/requirements.txt \
    && pip3 freeze --path /usr/local/lib/python3.12/dist-packages > /constraints.txt

FROM scratch AS python-lock-export
COPY --from=python-lock /constraints.txt /

# ---------------------------------------------------------------------------
FROM system AS runtime

LABEL org.opencontainers.image.title="ufil-runtime" \
      org.opencontainers.image.description="Runtime image for Ufil with ROS 2 Jazzy" \
      org.opencontainers.image.authors="schaefer@embedded.rwth-aachen.de" \
      org.opencontainers.image.source="https://git.unibw.de/cas/project/simon/software/ufil/ufil-workspaces/ufil-workspace"

# Pinned Python packages; shared with CI (.gitlab-ci.yml)
COPY install-python.sh requirements.txt constraints.txt /tmp/install/
RUN /tmp/install/install-python.sh /tmp/install

WORKDIR /workspace

COPY .bashrc /root/.bashrc

# ---------------------------------------------------------------------------
FROM runtime AS dev

ARG USERNAME=ufil
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive

# Ubuntu Noble ships a default "ubuntu" user with UID 1000; replace it with ours
RUN if id -u "$USER_UID" >/dev/null 2>&1; then userdel "$(id -un "$USER_UID")"; fi \
    && groupadd --gid "$USER_GID" "$USERNAME" \
    && useradd --uid "$USER_UID" --gid "$USER_GID" --shell /bin/bash -m "$USERNAME" \
    && apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    build-essential \
    gdb \
    git \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/"$USERNAME" \
    && chmod 0440 /etc/sudoers.d/"$USERNAME"

COPY --chown=$USER_UID:$USER_GID .bashrc /home/$USERNAME/.bashrc

ENV SHELL=/bin/bash
USER $USERNAME
WORKDIR /home/$USERNAME/ws

CMD ["/bin/bash"]
