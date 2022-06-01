#!/usr/bin/env bash

cd envs/
pip install -e .
pip install --upgrade scipy
cd ../ikfastpy
python3 setup.py build_ext --inplace
cd ..