#!/bin/sh -xe

mkdir build
cd build

# Use official pre-built OpenCASCADE (x64 only)
OCC_DIR="${OpenCASCADE_DIR:-/c/occ/opencascade-7.9.3}"

if [ "$1" = "release" ]; then
    if [ "$2" = "openmp" ]; then
        ENABLE_OPENMP="ON"
    else
        ENABLE_OPENMP="OFF"
    fi

    BUILD_TYPE=RelWithDebInfo
    cmake \
        -G "Visual Studio 17 2022" \
        -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
        -DCMAKE_PREFIX_PATH="${OCC_DIR}" \
        -DENABLE_OPENMP="${ENABLE_OPENMP}" \
        -DUSE_OPENCASCADE="ON" \
        -DENABLE_LTO=ON \
        -DCMAKE_GENERATOR_PLATFORM="x64" \
        ..
else
    BUILD_TYPE=Debug
    cmake \
        -G "Visual Studio 17 2022" \
        -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
        -DCMAKE_PREFIX_PATH="${OCC_DIR}" \
        -DENABLE_OPENMP="ON" \
        -DUSE_OPENCASCADE="ON" \
        -DCMAKE_GENERATOR_PLATFORM="x64" \
        ..
fi

cmake --build . --config "${BUILD_TYPE}" -- -maxcpucount

# Skip visual tests on CI - rendering differs between environments

if [ "$2" = "openmp" ]; then
    mv bin/$BUILD_TYPE/solvespace.exe bin/$BUILD_TYPE/solvespace.exe
else
    mv bin/$BUILD_TYPE/solvespace.exe bin/$BUILD_TYPE/solvespace_single_core.exe
fi
