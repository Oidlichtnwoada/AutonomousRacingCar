#!/bin/bash

# -------------------------------------------------------
# Author: Thomas Pintaric (thomas.pintaric@epfl.ch)
# SPDX-License-Identifier: GPL-3.0-or-later
# -------------------------------------------------------

ACCOUNT=pintaric
REPOSITORY=f1tenth_simulator
TAG=0.1.0

docker run -it --rm -p 50051:50051 ${ACCOUNT}/${REPOSITORY}:${TAG} "$@"