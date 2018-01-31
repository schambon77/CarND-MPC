#! /bin/bash
cd build
git pull origin master
cmake .. && make
./mpc
cd ..