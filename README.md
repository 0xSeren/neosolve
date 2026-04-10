# neosolve

<img src="res/freedesktop/solvespace-scalable.svg" width="70" height="70" alt="neosolve Logo" align="left">

[![Build Status](https://github.com/0xSeren/neosolve/workflows/CD/badge.svg)](https://github.com/0xSeren/neosolve/actions)

**neosolve** is an enhanced fork of [SolveSpace](https://solvespace.com), the parametric 2D/3D CAD tool. It integrates OpenCASCADE for modern solid modeling operations while preserving SolveSpace's lightweight, constraint-based workflow.

## Why neosolve?

SolveSpace's native geometric kernel is a remarkable piece of engineering - lean, educational, and fully self-contained. It demonstrates that you don't need a massive dependency to do real CAD work. We believe this kernel has a good reason to exist and deserves continued development.

However, for day-to-day CAD work, certain operations remain difficult or impossible with the native kernel: robust fillets, chamfers, shells, lofts, sweeps, and reliable STEP import/export. OpenCASCADE provides these capabilities today.

neosolve treats OpenCASCADE as a necessary bridge - enabling productive work now while the native kernel matures. Both kernels run side-by-side, with the OCC kernel handling operations that would otherwise be unavailable.

### Why not just contribute to SolveSpace?

Some changes are appropriate for upstream contribution, and we do contribute fixes back where possible. However:

- **Philosophical difference**: SolveSpace values being self-contained with minimal dependencies. Adding OpenCASCADE as a core dependency is a significant departure from this philosophy.
- **Scope of changes**: Many features (dockable panels, new constraint types, UI overhauls) involve invasive changes that may not align with upstream priorities.
- **Experimentation**: neosolve serves as a testing ground for ideas that may eventually inform upstream development.

We encourage users who prefer a dependency-free experience to use upstream SolveSpace.

## Features

### OpenCASCADE Integration

- **Solid Operations**: Extrusion, lathe, revolve, fillet, chamfer, shell, loft, sweep, mirror
- **STEP/BREP/IGES Import**: Import solid geometry as manipulable groups with cached meshes
- **STEP/BREP Export**: Export models using OCC's robust writers
- **Dual Kernel**: OCC runs alongside the native NURBS kernel with automatic fallback

### New Constraints

- **Circle-Line Tangent**: Constrain circles tangent to lines
- **Concentric**: Select multiple circles/arcs to make centers coincident
- **Point-on-Cubic**: Constrain points to lie on Bezier curves
- **Point-on-Segment**: Bound points to finite line segments (not infinite lines)
- **Inequality Constraints**: Minimum/maximum distance constraints (≥ and ≤)

### UI Improvements

- **Dockable Property Browser**: Detachable panel for the property browser (GTK)
- **Directional Marquee Selection**: Left-to-right for window selection, right-to-left for crossing selection
- **Arrow Key Nudging**: Move selected entities with arrow keys
- **Improved Themes**: Gruvbox-inspired color palette, Qt dark theme support
- **Double-Click Text Editing**: Edit TTF text inline by double-clicking
- **Independent Visibility Toggles**: Separate controls for REF constraints and comments

### Performance

- **O(n log n) Triangulation**: Monotone polygon triangulation for large polygons
- **Mesh Caching**: Cached OCC mesh generation for fast interactive updates
- **Spatial Hashing**: O(1) vertex deduplication for STL import
- **Built-in Profiler**: Profile command in CLI with JSON export

### Export Enhancements

- **G-Code Arcs**: Export arcs as G02/G03 commands instead of line segments
- **SVG Improvements**: Closed paths include Z command
- **CLI Group Export**: `--group` option for targeted export

## Installation

### Pre-built Packages

Pre-built packages for **Windows**, **Linux**, and **macOS** are available from [GitHub Releases](https://github.com/0xSeren/neosolve/releases).

Download the appropriate package for your platform and run the installer or extract the archive.

### Building from Source

Clone the repository with submodules:

```sh
git clone https://github.com/0xSeren/neosolve
cd neosolve
git submodule update --init
```

## Building on Linux

### Dependencies

On Debian/Ubuntu:

```sh
sudo apt install git build-essential cmake zlib1g-dev libpng-dev \
    libcairo2-dev libfreetype6-dev libjson-c-dev \
    libfontconfig1-dev libpangomm-1.4-dev libgl-dev \
    libglu-dev libspnav-dev libgtkmm-3.0-dev qt6-base-dev \
    libocct-modeling-algorithms-dev libocct-data-exchange-dev libtbb-dev
```

On Fedora:

```sh
sudo dnf install git gcc-c++ cmake zlib-devel libpng-devel \
    cairo-devel freetype-devel json-c-devel \
    fontconfig-devel pangomm-devel mesa-libGL-devel \
    mesa-libGLU-devel libspnav-devel gtkmm30-devel \
    qt6-qtbase-devel opencascade-devel
```

GTK and Qt dependencies can be omitted if only one GUI is needed.

### Build

```sh
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_OPENMP=ON
make -j$(nproc)

# Optional: install system-wide
sudo make install
```

### Arch Linux

A PKGBUILD is provided in `pkg/arch/`:

```sh
cd pkg/arch
makepkg -si
```

### NixOS / Nix

A flake is provided at the repository root:

```sh
# Run directly
nix run github:0xSeren/neosolve

# Install into profile
nix profile install github:0xSeren/neosolve

# Development shell
nix develop
```

## Building on macOS

Install dependencies via Homebrew:

```sh
brew install git cmake libomp opencascade
```

Build:

```sh
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_OPENMP=ON
make -j$(sysctl -n hw.ncpu)
```

The application is built as `build/bin/SolveSpace.app`.

## Building on Windows

### Visual Studio

1. Install [git](https://git-scm.com/download/win), [CMake](https://cmake.org/download/), and Visual Studio 2019 or later
2. Install OpenCASCADE via vcpkg: `vcpkg install opencascade:x64-windows`
3. Clone and build:

```bat
git clone https://github.com/0xSeren/neosolve
cd neosolve
git submodule update --init
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=[vcpkg root]/scripts/buildsystems/vcpkg.cmake
cmake --build . --config Release
```

### MSYS2/MinGW

```sh
pacman -S mingw-w64-x86_64-{git,gcc,cmake,ninja,opencascade}
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -GNinja
ninja
```

## Usage

neosolve is fully compatible with SolveSpace `.slvs` files. The OpenCASCADE operations are available through the standard group menu when OCC support is enabled.

For documentation on SolveSpace's core features, see the [SolveSpace reference manual](http://solvespace.com/ref.pl) and [tutorials](http://solvespace.com/tutorial.pl).

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for code style and contribution guidelines.

Bug reports and feature requests are welcome on the [GitHub issue tracker](https://github.com/0xSeren/neosolve/issues).

## License

neosolve is distributed under the terms of the [GPL v3](COPYING.txt) or later, the same license as SolveSpace.

## Acknowledgments

neosolve builds on the excellent work of the [SolveSpace](https://solvespace.com) project and its contributors. We are grateful for their continued development of the core parametric CAD system.
