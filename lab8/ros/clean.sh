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
ALL_DIRS="catkin_tools catkin_tools.install melodic-python3"

for DELETE_DIR in ${ALL_DIRS}; do
    if [ -d ${SCRIPT_DIR}/${DELETE_DIR} ]; then
        rm -rf ${SCRIPT_DIR}/${DELETE_DIR}
    fi
done
