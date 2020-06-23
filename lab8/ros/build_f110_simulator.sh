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
WORKSPACE_DIR=${SCRIPT_DIR}/f110_simulator_workspace

if [ -z ${ROS_ROOT} ]; then
    source /opt/ros/melodic/setup.bash
fi

if [ ! -f ${ROS_ROOT}/package.xml ]; then
    echo "ROS not found. Aborting."
    exit 1
fi

if [ -d ${WORKSPACE_DIR} ]; then
    rm -rf ${WORKSPACE_DIR}
fi

mkdir -p ${WORKSPACE_DIR}/src
git clone https://github.com/f1tenth/f1tenth_simulator.git ${WORKSPACE_DIR}/src/f1tenth_simulator
cp ${SCRIPT_DIR}/../map_preprocessor/maps/* ${WORKSPACE_DIR}/src/f1tenth_simulator/maps
sed -i 's/levine/f1\_aut/g' ${WORKSPACE_DIR}/src/f1tenth_simulator/launch/simulator.launch

cd ${WORKSPACE_DIR}
catkin config --init -DCMAKE_BUILD_TYPE=Release
catkin build
