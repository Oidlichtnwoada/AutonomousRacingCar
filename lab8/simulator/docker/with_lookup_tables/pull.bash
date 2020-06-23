#!/bin/bash

# -------------------------------------------------------
# Author: Thomas Pintaric (thomas.pintaric@epfl.ch)
# SPDX-License-Identifier: GPL-3.0-or-later
# -------------------------------------------------------

ACCOUNT=pintaric
REPOSITORY=f1tenth_simulator
TAG_WITH_LUT=0.1.0-with-lut

docker pull ${ACCOUNT}/${REPOSITORY}:${TAG_WITH_LUT} "$@"