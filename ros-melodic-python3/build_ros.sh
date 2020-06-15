#!/bin/bash 

# -------------------------------------------------------
# Author: Thomas Pintaric (thomas.pintaric@epfl.ch)
# SPDX-License-Identifier: 0BSD
# -------------------------------------------------------
#
# This shell script will build ROS Melodic (rospy, ros_core,
# ros_comm, actionlib, dynamic_reconfigure) with support for 
# Python 3.7 (virtualenv).
# It is intended to be run on a system with an existing
# ROS Melodic installation (e.g. Ubuntu 18.04 with the official
# "ros-melodic-desktop-full" package installed). Otherwise,
# some dependencies might be missing (such as boost*-dev).

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

# NOTE: By default, pyenv doesn't build CPython with --enable-shared.
# In the case of linker errors (e.g. "relocation R_X86_64_PC32 against
# symbol `_PyRuntime' can not be used"), the solution is to do re-run
# "pyenv install" as follows:
#
# env PYTHON_CONFIGURE_OPTS="--enable-shared" pyenv install 3.7.6

SCRIPT_DIR=$( builtin cd "$( dirname "${DIR}" )" > /dev/null && pwd ${PWD_OPT})
ROS_SDK_ROOT=${SCRIPT_DIR}/melodic-python${PYTHON_VERSION_MAJOR}

cd ${SCRIPT_DIR}

pyenv virtualenv-delete ros-melodic-python${PYTHON_VERSION_MAJOR}
pyenv virtualenv ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}.${PYTHON_VERSION_MICRO} ros-melodic-python${PYTHON_VERSION_MAJOR}
pyenv local ros-melodic-python${PYTHON_VERSION_MAJOR}

pip install pip -U

mkdir -p ${SCRIPT_DIR}/catkin_tools.install
mkdir -p ${SCRIPT_DIR}/catkin_tools
cd ${SCRIPT_DIR}/catkin_tools
git clone --depth=1 https://github.com/catkin/catkin_tools.git .
pip install -r requirements.txt --upgrade
python setup.py install --prefix ${SCRIPT_DIR}/catkin_tools.install --record install_manifest.txt

export PYTHONPATH="${SCRIPT_DIR}/catkin_tools.install/lib/python${PYTHON_VERSION}/site-packages:${PYTHONPATH}"
export PATH="${SCRIPT_DIR}/catkin_tools.install/bin:${PATH}"
export CMAKE_PREFIX_PATH="$(pyenv virtualenv-prefix):${CMAKE_PREFIX_PATH}"

pip install rosdep rospkg rosinstall_generator rosinstall wstool vcstools
rosdep init
rosdep update

mkdir -p ${ROS_SDK_ROOT}/src
cd ${ROS_SDK_ROOT}

# TODO: Patch dynamic_reconfigure for Python 3, then use "wstool merge custom.rosinstall"
# https://answers.ros.org/question/252882/how-to-clone-with-proper-branch-options-using-wstool/

export INSTALL_ROS_PACKAGES="rospy ros_core ros_comm actionlib joy"
export DO_NOT_INSTALL_ROS_PACKAGES="rqt_rviz rviz_plugin_tutorials librviz_tutorial"

catkin config --init -DCMAKE_BUILD_TYPE=Release --blacklist ${DO_NOT_INSTALL_ROS_PACKAGES} --install
rosinstall_generator ${INSTALL_ROS_PACKAGES} --rosdistro melodic --deps --tar > melodic-custom.rosinstall
wstool init --shallow -j8 src melodic-custom.rosinstall
export ROS_PYTHON_VERSION=3

pip install empy
sed -i '/set(SETUPTOOLS_ARG_EXTRA "--install-layout=deb")/d' src/catkin/cmake/python.cmake

catkin build catkin
source ${ROS_SDK_ROOT}/devel/setup.bash
source ${ROS_SDK_ROOT}/install/setup.bash

catkin build
source ${ROS_SDK_ROOT}/devel/setup.bash
source ${ROS_SDK_ROOT}/install/setup.bash

catkin clean -y --build
pip install defusedxml netifaces pycryptodomex gnupg
