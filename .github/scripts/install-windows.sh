#!/bin/sh -xe

# Install OpenCASCADE via vcpkg (vcpkg is pre-installed on GitHub Actions)
# Use VCPKG_ROOT if set, otherwise default to C:/vcpkg
VCPKG_DIR="${VCPKG_INSTALLATION_ROOT:-/c/vcpkg}"

# Update vcpkg to latest
cd "$VCPKG_DIR"
git pull || true
./bootstrap-vcpkg.bat

# Install OpenCASCADE for both platforms
./vcpkg install opencascade:x64-windows opencascade:x86-windows --clean-after-build

cd -
git submodule update --init
