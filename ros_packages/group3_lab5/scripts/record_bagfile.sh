#!/bin/bash

# -------------------------------------------------------
# VU Autonomous Racing Cars (2020S) - TU Wien
# -------------------------------------------------------
# Author: Thomas Pintaric (thomas@pintaric.org)
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

export BASE_DIR=$( builtin cd "$( dirname "${DIR}" )" > /dev/null && pwd ${PWD_OPT})
export BAG_DIR=${BASE_DIR}/../bags
source /opt/ros/melodic/setup.bash
rosbag record gt_pose tf -O ${BAG_DIR}/$1.bag
