#!/bin/bash
###
 # @Descripttion: 
 # @version: 
 # @Author: lxy
 # @Date: 2023-10-05 12:41:05
 # @LastEditors: lxy
 # @LastEditTime: 2023-10-05 12:41:07
### 
if test -d "build"; then
  echo "Directory exists"
else
  echo "Directory does not exist"
  mkdir build
fi

cd build
cmake ..
make -j4

