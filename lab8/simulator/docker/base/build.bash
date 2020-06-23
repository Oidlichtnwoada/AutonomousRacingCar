#!/bin/bash

# -------------------------------------------------------
# Author: Thomas Pintaric (thomas.pintaric@epfl.ch)
# SPDX-License-Identifier: GPL-3.0-or-later
# -------------------------------------------------------

ACCOUNT=pintaric
REPOSITORY=f1tenth_simulator
TAG=0.1.0

SSH_PRIVATE_KEY_FILE=github_rsa

if [ ! -f ${SSH_PRIVATE_KEY_FILE} ]; then
    echo 'Private SSH key "'${SSH_PRIVATE_KEY_FILE}'" not found.'
    exit 1
fi

SSH_PRIVATE_KEY=$(<${SSH_PRIVATE_KEY_FILE})
docker build . --tag ${ACCOUNT}/${REPOSITORY}:${TAG} \
    --build-arg SSH_PRIVATE_KEY="${SSH_PRIVATE_KEY}" \
    --rm -f ./Dockerfile "$@"