#!/bin/sh
cd testcode/build
cmake ../
cmake --build .
cd ../..
python cpp_fitting_testing.py