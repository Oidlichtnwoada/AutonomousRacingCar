# -------------------------------------------------------
# Author: Thomas Pintaric (thomas.pintaric@epfl.ch)
# SPDX-License-Identifier: GPL-3.0-or-later
# -------------------------------------------------------

ARG BASE
FROM ${BASE}
SHELL ["/bin/bash", "-c"]
RUN /opt/f1tenth_simulator/racecar_simulator -c /opt/f1tenth_simulator/configuration/racecar_simulator.json -o
EXPOSE 50051
ENTRYPOINT ["/opt/f1tenth_simulator/racecar_simulator", "-c /opt/f1tenth_simulator/configuration/racecar_simulator.json"]