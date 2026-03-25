#!/bin/sh -xe

mkdir build
cd build

# Use official pre-built OpenCASCADE (x64 only)
# OpenCASCADE_DIR is set by install-windows.sh via GITHUB_ENV
OCC_DIR="${OpenCASCADE_DIR:-C:/occ/opencascade-7.9.3-vc14-64/cmake}"
echo "Using OpenCASCADE from: $OCC_DIR"

if [ "$1" = "release" ]; then
    BUILD_TYPE=RelWithDebInfo
    cmake \
        -G "Visual Studio 17 2022" \
        -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
        -DCMAKE_PREFIX_PATH="${OCC_DIR}" \
        -DENABLE_OPENMP="ON" \
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

# Copy OpenCASCADE DLLs to output directory for release builds
if [ "$1" = "release" ]; then
    OCC_ROOT="${OpenCASCADE_DIR:-C:/occ/opencascade-7.9.3-vc14-64}"
    OCC_ROOT="${OCC_ROOT%/cmake}"  # Remove /cmake suffix if present
    OUTPUT_DIR="bin/${BUILD_TYPE}"

    echo "Copying OpenCASCADE DLLs from $OCC_ROOT to $OUTPUT_DIR"

    # Copy OCC DLLs
    if [ -d "$OCC_ROOT/win64/vc14/bin" ]; then
        cp "$OCC_ROOT/win64/vc14/bin/"*.dll "$OUTPUT_DIR/" 2>/dev/null || true
    fi

    # Copy 3rdparty DLLs (freetype, tbb, etc.)
    THIRDPARTY_DIR="C:/occ/3rdparty-vc14-64"
    if [ -d "$THIRDPARTY_DIR" ]; then
        # FreeType
        cp "$THIRDPARTY_DIR/freetype-2.5.5-vc14-64/bin/"*.dll "$OUTPUT_DIR/" 2>/dev/null || true
        # TBB
        cp "$THIRDPARTY_DIR/tbb_2021.5-vc14-64/bin/"*.dll "$OUTPUT_DIR/" 2>/dev/null || true
        # Other potential dependencies
        for dir in "$THIRDPARTY_DIR"/*/bin; do
            [ -d "$dir" ] && cp "$dir/"*.dll "$OUTPUT_DIR/" 2>/dev/null || true
        done
    fi

    echo "DLLs copied:"
    ls -la "$OUTPUT_DIR/"*.dll 2>/dev/null | head -20 || echo "No DLLs found"
fi
