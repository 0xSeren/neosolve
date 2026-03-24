#!/bin/sh -xe

# Note: OpenCASCADE on Windows requires vcpkg and takes ~30 min to build
# Disabled for now - Windows builds don't include OCC features

git submodule update --init
