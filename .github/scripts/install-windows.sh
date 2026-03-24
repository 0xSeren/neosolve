#!/bin/sh -xe

# Install OpenCASCADE via vcpkg (pre-installed on GitHub Actions)
vcpkg install opencascade:x86-windows opencascade:x64-windows

git submodule update --init
