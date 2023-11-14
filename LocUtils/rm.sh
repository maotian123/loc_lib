#!/bin/bash
if test -d "build"; then
  echo "Directory exists"
else
  echo "Directory does not exist"
  mkdir build
fi

rm -rf libs
mkdir libs
cd build
cmake ..
make -j4
sudo make install
