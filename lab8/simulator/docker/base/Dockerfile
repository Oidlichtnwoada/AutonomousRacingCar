# -------------------------------------------------------
# Author: Thomas Pintaric (thomas.pintaric@epfl.ch)
# SPDX-License-Identifier: GPL-3.0-or-later
# -------------------------------------------------------

FROM ubuntu:20.04 as intermediate
SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG SSH_PRIVATE_KEY
ENV STAGING_DIR=/tmp/f1tenth_simulator

RUN apt-get update && apt-get install --no-install-recommends -y \
    apt-utils build-essential cmake git ssh ca-certificates \
    libboost1.71-dev  libboost-iostreams1.71-dev libboost-system1.71-dev \
    libeigen3-dev libopencv-dev python3-pip
RUN mkdir /root/.ssh/
RUN echo "${SSH_PRIVATE_KEY}" > /root/.ssh/id_rsa
RUN chmod 700 /root/.ssh/id_rsa
RUN touch /root/.ssh/known_hosts
RUN ssh-keyscan github.com >> /root/.ssh/known_hosts
RUN git clone git@github.com:pintaric/f1tenth_simulator.git ${STAGING_DIR}
RUN pip3 install -r ${STAGING_DIR}/requirements.txt
RUN ${STAGING_DIR}/build_release.bash

FROM ubuntu:20.04
SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
COPY --from=intermediate /tmp/f1tenth_simulator/cmake-install-release /opt/f1tenth_simulator
LABEL Description="F1/Tenth Racecar Simulator" Version="0.1.0"
RUN apt-get update && apt-get install --no-install-recommends -y \
    libopencv-core4.2  libboost-iostreams1.71.0 libboost-system1.71.0 \
    libopencv-imgproc4.2 libopencv-imgcodecs4.2 libgomp1 && \
    apt-get update && rm -rf /var/lib/apt/lists/*
EXPOSE 50051
ENTRYPOINT ["/opt/f1tenth_simulator/racecar_simulator", "-c /opt/f1tenth_simulator/configuration/racecar_simulator.json", "-d"]