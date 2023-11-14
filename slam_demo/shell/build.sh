#!/bin/bash
if test -d "../build"; then
  echo "Directory exists"
else
  echo "Directory does not exist"
  mkdir ../build
fi

cd ../build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 ..
make
