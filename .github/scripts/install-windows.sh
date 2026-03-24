#!/bin/sh -xe

# Install OpenCASCADE via vcpkg
git clone https://github.com/microsoft/vcpkg.git /c/vcpkg
/c/vcpkg/bootstrap-vcpkg.bat
/c/vcpkg/vcpkg install opencascade:x64-windows opencascade:x86-windows

git submodule update --init
