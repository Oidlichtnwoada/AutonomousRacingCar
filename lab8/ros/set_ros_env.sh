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

export PYTHON_VERSION_MAJOR=3
export PYTHON_VERSION_MINOR=7
export PYTHON_VERSION_MICRO=6
export PYTHON_VERSION=${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}

SCRIPT_DIR=$( builtin cd "$( dirname "${DIR}" )" > /dev/null && pwd ${PWD_OPT})
ROS_SDK_ROOT=${SCRIPT_DIR}/melodic-python${PYTHON_VERSION_MAJOR}

export PYTHONPATH="${SCRIPT_DIR}/catkin_tools.install/lib/python${PYTHON_VERSION}/site-packages:${PYTHONPATH}"
export PATH="${SCRIPT_DIR}/catkin_tools.install/bin:${PATH}"
export CMAKE_PREFIX_PATH="$(pyenv virtualenv-prefix):${CMAKE_PREFIX_PATH}"

source ${ROS_SDK_ROOT}/devel/setup.bash
source ${ROS_SDK_ROOT}/install/setup.bash
