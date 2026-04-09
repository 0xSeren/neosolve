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

# Copy only required OpenCASCADE DLLs to output directory for release builds
if [ "$1" = "release" ]; then
    OCC_ROOT="${OpenCASCADE_DIR:-C:/occ/opencascade-7.9.3-vc14-64}"
    OCC_ROOT="${OCC_ROOT%/cmake}"  # Remove /cmake suffix if present
    OUTPUT_DIR="bin/${BUILD_TYPE}"
    OCC_BIN="$OCC_ROOT/win64/vc14/bin"
    THIRDPARTY_DIR="C:/occ/3rdparty-vc14-64"

    echo "Copying required OpenCASCADE DLLs to $OUTPUT_DIR"

    # Only the OCC libraries we actually link against + their transitive deps
    OCC_DLLS="
        TKernel TKMath TKBRep TKTopAlgo TKPrim TKMesh
        TKFillet TKOffset TKBO TKShHealing TKG3d TKG2d TKGeomBase
        TKGeomAlgo TKHLR TKDESTEP TKDEIGES TKXSBase TKDE
        TKCDF TKLCAF TKService TKXCAF TKBinXCAF TKXmlXCAF
        TKBin TKBinL TKXml TKXmlL TKStd TKStdL TKTObj
        TKRWMesh
    "
    for lib in $OCC_DLLS; do
        if [ -f "$OCC_BIN/${lib}.dll" ]; then
            cp "$OCC_BIN/${lib}.dll" "$OUTPUT_DIR/"
        fi
    done

    # Required 3rdparty: TBB, FreeType, and OpenVR
    for pattern in "tbb_*/bin/tbb12.dll" "tbb_*/bin/tbbmalloc.dll" "freetype-*/bin/freetype.dll" "openvr-*/bin/win64/openvr_api.dll"; do
        for f in "$THIRDPARTY_DIR"/$pattern; do
            [ -f "$f" ] && cp "$f" "$OUTPUT_DIR/" 2>/dev/null || true
        done
    done

    # Remove debug DLLs that may have been pulled in by CMake
    rm -f "$OUTPUT_DIR"/tbb12_debug.dll "$OUTPUT_DIR"/tbbmalloc_debug.dll \
          "$OUTPUT_DIR"/tbbmalloc_proxy_debug.dll 2>/dev/null || true
    # Remove Qt debug DLLs (Qt5*d.dll pattern)
    rm -f "$OUTPUT_DIR"/Qt5*d.dll 2>/dev/null || true

    echo "DLLs in output:"
    ls "$OUTPUT_DIR/"*.dll 2>/dev/null | wc -l
    du -sh "$OUTPUT_DIR/"
fi
