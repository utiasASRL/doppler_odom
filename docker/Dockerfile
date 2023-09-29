FROM ubuntu:20.04

CMD ["/bin/bash"]

# Args for setting up non-root users, example command to use your own user:
# docker build -t ct_icp \
#   --build-arg USERID=$(id -u) \
#   --build-arg GROUPID=$(id -g) \
#   --build-arg USERNAME=$(whoami) \
#   --build-arg HOMEDIR=${HOME} .
ARG GROUPID=0
ARG USERID=0
ARG USERNAME=root
ARG HOMEDIR=/root

RUN if [ ${GROUPID} -ne 0 ]; then addgroup --gid ${GROUPID} ${USERNAME}; fi \
  && if [ ${USERID} -ne 0 ]; then adduser --disabled-password --gecos '' --uid ${USERID} --gid ${GROUPID} ${USERNAME}; fi

ENV DEBIAN_FRONTEND=noninteractive

## timezone
ENV TZ=America/Toronto
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

## Switch to specified user to create directories
USER ${USERID}:${GROUPID}

## Switch to root to install dependencies
USER 0:0

## Dependencies
RUN apt update && apt upgrade -q -y
RUN apt update && apt install -q -y cmake git build-essential lsb-release curl gnupg2
RUN apt update && apt install -q -y libboost-all-dev libomp-dev
RUN apt update && apt install -q -y libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
RUN apt update && apt install -q -y freeglut3-dev
RUN apt update && apt install -q -y python3 python3-distutils python3-pip
RUN pip3 install pybind11

# UTF-8
RUN apt install -q -y locales \
  && locale-gen en_US en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

## Install misc dependencies
RUN apt update && apt install -q -y \
  tmux \
  libboost-all-dev libomp-dev \
  libpcl-dev \
  virtualenv \
  texlive-latex-extra \
  texlive-fonts-recommended \
  cm-super \
  dvipng \
  clang-format

## Install python dependencies
# jupyter
RUN pip3 install pexpect ipympl
# evaluation
RUN pip3 install asrl-pylgmath asrl-pysteam

## Switch to specified user
USER ${USERID}:${GROUPID}
