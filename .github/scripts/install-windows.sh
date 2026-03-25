#!/bin/sh -xe

# Download official pre-built OpenCASCADE binaries (x64 only)

OCC_VERSION="7.9.3"
OCC_URL="https://github.com/Open-Cascade-SAS/OCCT/releases/download/V7_9_3/opencascade-${OCC_VERSION}-vc14-64.zip"
OCC_3RDPARTY_URL="https://github.com/Open-Cascade-SAS/OCCT/releases/download/V7_9_3/3rdparty-vc14-64.zip"

mkdir -p /c/occ
cd /c/occ

echo "Downloading OpenCASCADE ${OCC_VERSION} x64 binaries..."
curl -L -o opencascade-x64.zip "$OCC_URL"
curl -L -o 3rdparty-x64.zip "$OCC_3RDPARTY_URL"

echo "Extracting OpenCASCADE..."
unzip -q -o opencascade-x64.zip
unzip -q -o 3rdparty-x64.zip

# Set up environment for CMake to find OpenCASCADE
echo "OpenCASCADE_DIR=/c/occ/opencascade-${OCC_VERSION}" >> $GITHUB_ENV

cd -
git submodule update --init
