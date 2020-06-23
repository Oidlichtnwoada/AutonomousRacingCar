#!/bin/bash 
# -------------------------------------------------------
# Author: Thomas Pintaric (thomas.pintaric@gmail.com)
# SPDX-License-Identifier: GPL-3.0-or-later
# -------------------------------------------------------

DIR="${BASH_SOURCE[0]}"
NAME=$(basename -- "$0")
SCRIPT_DIR=$( builtin cd "$( dirname "${DIR}" )" > /dev/null && pwd ${PWD_OPT})
INSTALL_DIR=${SCRIPT_DIR}/cmake-install-release
SIMULATOR_EXECUTABLE=${INSTALL_DIR}/racecar_simulator
JSON_CONFIG_FILE=${INSTALL_DIR}/configuration/racecar_simulator.json

if [ ! -f ${SIMULATOR_EXECUTABLE} ]; then
    echo "Simulator executable not found. Expected location: ${SIMULATOR_EXECUTABLE}"
    exit 1
fi

if [ ! -f ${JSON_CONFIG_FILE} ]; then
    echo "Configuration file not found. Expected location: ${JSON_CONFIG_FILE}"
    exit 1
fi

${SIMULATOR_EXECUTABLE} -c ${JSON_CONFIG_FILE} $1 $2 $3 $4 $5 $6 $7 $8 $9
