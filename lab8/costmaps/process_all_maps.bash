#!/bin/bash 
# -------------------------------------------------------
# Author: Thomas Pintaric (thomas.pintaric@gmail.com)
# SPDX-License-Identifier: GPL-3.0-or-later
# -------------------------------------------------------

python3 ./map_preprocessor.py --input=maps/f1_aut.yaml --output=maps/f1_aut.npz
python3 ./map_preprocessor.py --input=maps/f1_esp.yaml --output=maps/f1_esp.npz
python3 ./map_preprocessor.py --input=maps/f1_gbr.yaml --output=maps/f1_gbr.npz
python3 ./map_preprocessor.py --input=maps/f1_mco.yaml --output=maps/f1_mco.npz
