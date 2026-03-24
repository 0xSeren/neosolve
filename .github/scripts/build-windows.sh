#!/bin/sh -xe

mkdir build
cd build

# Use vcpkg toolchain for dependencies
VCPKG_ROOT="${VCPKG_INSTALLATION_ROOT:-/c/vcpkg}"

if [ "$1" = "release" ]; then
    if [ "$2" = "openmp" ]; then
        ENABLE_OPENMP="ON"
    else
        ENABLE_OPENMP="OFF"
    fi

    if [ "$3" = "x64" ]; then
        PLATFORM="x64"
        VCPKG_TRIPLET="x64-windows"
    else
        PLATFORM="Win32"
        VCPKG_TRIPLET="x86-windows"
    fi

    BUILD_TYPE=RelWithDebInfo
    cmake \
        -G "Visual Studio 17 2022" \
        -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
        -DCMAKE_TOOLCHAIN_FILE="${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" \
        -DVCPKG_TARGET_TRIPLET="${VCPKG_TRIPLET}" \
        -DENABLE_OPENMP="${ENABLE_OPENMP}" \
        -DENABLE_LTO=ON \
        -DCMAKE_GENERATOR_PLATFORM="${PLATFORM}" \
        ..
else
    BUILD_TYPE=Debug
    cmake \
        -G "Visual Studio 17 2022" \
        -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
        -DCMAKE_TOOLCHAIN_FILE="${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" \
        -DVCPKG_TARGET_TRIPLET="x86-windows" \
        -DENABLE_OPENMP="ON" \
        -DENABLE_SANITIZERS="ON" \
        -DCMAKE_GENERATOR_PLATFORM="Win32" \
        ..
fi

cmake --build . --config "${BUILD_TYPE}" -- -maxcpucount

# Run the tests in the proper environment (required for having the ASan libraries available)
cmake --build . --config "${BUILD_TYPE}" -t test_solvespace -- -maxcpucount

if [ "$3" = "x64" ]; then
	if [ "$2" != "openmp" ]; then
		mv bin/$BUILD_TYPE/solvespace.exe bin/$BUILD_TYPE/solvespace_single_core_x64.exe
	else
		mv bin/$BUILD_TYPE/solvespace.exe bin/$BUILD_TYPE/solvespace_x64.exe
	fi
else
	if [ "$2" != "openmp" ]; then
		mv bin/$BUILD_TYPE/solvespace.exe bin/$BUILD_TYPE/solvespace_single_core.exe
	fi
fi
