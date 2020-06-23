#!/bin/bash

# -------------------------------------------------------
# Author: Thomas Pintaric (thomas.pintaric@epfl.ch)
# SPDX-License-Identifier: GPL-3.0-or-later
# -------------------------------------------------------

ACCOUNT=pintaric
REPOSITORY=f1tenth_simulator
TAG=0.1.0
TAG_WITH_LUT=0.1.0-with-lut

docker build . --tag ${ACCOUNT}/${REPOSITORY}:${TAG_WITH_LUT} \
    --build-arg BASE=${ACCOUNT}/${REPOSITORY}:${TAG} \
    --rm -f ./Dockerfile "$@"