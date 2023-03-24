FROM ubuntu:20.04

RUN DEBIAN_FRONTEND="noninteractive" apt-get update && apt-get -y install tzdata && apt-get clean

RUN apt-get update \
  && apt-get install -y \
    build-essential \
    gcc \
    g++ \
    gdb \
    clang \
    make \
    ninja-build \
    cmake \
    autoconf \
    automake \
    libtool \
    valgrind \
    locales-all \
    dos2unix \
    rsync \
    tar \
    python3 \
    python3-dev \
    python-is-python3 \
    curl \
    git \
  && apt-get clean
SHELL ["/bin/bash", "-c"]

ARG UID=1000
ARG GID=1000
ARG USERNAME="builder"
RUN apt-get update -qqy && apt-get install -qqy --no-install-recommends sudo && apt-get clean && rm -rf /var/lib/apt/*
RUN \
  set -euxo pipefail \
  && useradd --uid ${UID} -m -s /bin/bash ${USERNAME} \
  && groupmod --gid ${GID} ${USERNAME} \
  && usermod -aG dialout,sudo,video ${USERNAME} \
  && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
  && echo $'\
export PS1="\e[0;32m\u \e[0;34m\W \e[m$ "\n\
' >> /home/${USERNAME}/.bashrc

WORKDIR /home/${USERNAME}
USER ${USERNAME}

RUN git clone --depth 1 --branch 2023 https://github.com/tek5030/setup_scripts.git \
    && cd setup_scripts && ./00-run-complete-setup && cd .. && rm -rf setup_scripts
