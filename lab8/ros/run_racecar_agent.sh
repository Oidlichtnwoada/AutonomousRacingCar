#!/bin/bash 

# -------------------------------------------------------
# Author: Thomas Pintaric (thomas.pintaric@gmail.com)
# SPDX-License-Identifier: 0BSD
# -------------------------------------------------------

if [ -n "${ZSH_VERSION:-}" ]; then
	DIR="${(%):-%N}"
	if [ $options[posixargzero] != "on" ]; then
		setopt posixargzero
		NAME=$(basename -- "$0")
		unsetopt posixargzero
	else
		NAME=$(basename -- "$0")
	fi
else
	DIR="${BASH_SOURCE[0]}"
	NAME=$(basename -- "$0")
fi

SCRIPT_DIR=$( builtin cd "$( dirname "${DIR}" )" > /dev/null && pwd ${PWD_OPT})
WORKSPACE_DIR=${SCRIPT_DIR}/workspace

if [ -z ${ROS_ROOT} ]; then
    source ${SCRIPT_DIR}/set_ros_env.sh
fi

if [ ! -f ${ROS_ROOT}/package.xml ]; then
    echo "ROS not found. Aborting."
    exit 1
fi

if [ ! -d ${WORKSPACE_DIR} ]; then
    echo "Workspace not found. Aborting."
    exit 1
fi

source ${WORKSPACE_DIR}/devel/setup.bash
cd ${WORKSPACE_DIR}
roslaunch racecar_agent racecar_agent.launch
