#!/bin/sh -xe

mkdir build
cd build
cmake \
  -DCMAKE_BUILD_TYPE="Debug" \
  -DENABLE_OPENMP="ON" \
  -DENABLE_SANITIZERS="ON" \
  -DUSE_OPENCASCADE="OFF" \
  ..
make -j$(nproc) VERBOSE=1
# Skip visual tests on CI - rendering differs between environments
