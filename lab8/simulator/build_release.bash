#!/bin/bash 
# -------------------------------------------------------
# Author: Thomas Pintaric (thomas.pintaric@gmail.com)
# SPDX-License-Identifier: GPL-3.0-or-later
# -------------------------------------------------------

DIR="${BASH_SOURCE[0]}"
NAME=$(basename -- "$0")
SCRIPT_DIR=$( builtin cd "$( dirname "${DIR}" )" > /dev/null && pwd ${PWD_OPT})
SOURCE_DIR=${SCRIPT_DIR}
BUILD_DIR=${SCRIPT_DIR}/cmake-build-release
INSTALL_DIR=${SCRIPT_DIR}/cmake-install-release

NUM_THREADS=$(grep ^cpu\\scores /proc/cpuinfo | uniq |  awk '{print $4}')
cmake --config Release -S ${SOURCE_DIR} -B ${BUILD_DIR}
cmake --build ${BUILD_DIR} --parallel ${NUM_THREADS}

if [ ! -d ${INSTALL_DIR} ]; then
    mkdir -p ${INSTALL_DIR}
fi

cp -v ${BUILD_DIR}/simulator/racecar_simulator ${INSTALL_DIR}
cp -v ${BUILD_DIR}/map_converter/map_converter ${INSTALL_DIR}
cp -v -r ${SOURCE_DIR}/simulator/configuration ${INSTALL_DIR}
