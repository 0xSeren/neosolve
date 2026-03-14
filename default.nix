{ pkgs ? import <nixpkgs> {}, src ? ./. }:

pkgs.stdenv.mkDerivation {
  pname = "solvespace-themed";
  version = "3.2-themed";

  inherit src;

  nativeBuildInputs = with pkgs; [
    cmake
    pkg-config
    wrapGAppsHook3
  ];

  buildInputs = with pkgs; [
    at-spi2-core
    cairo
    dbus
    eigen
    freetype
    fontconfig
    glew
    gtkmm3
    json_c
    libdatrie
    libdeflate
    libepoxy
    libGLU
    libpng
    libselinux
    libsepol
    libspnav
    libthai
    libtiff
    libxkbcommon
    lerc
    opencascade-occt
    pangomm
    pcre
    pcre2
    util-linuxMinimal
    xorg.libpthreadstubs
    xorg.libXdmcp
    xorg.libXtst
    libsysprof-capture
    zlib
  ];

  prePatch = ''
    # Initialize submodules if not present
    mkdir -p extlib/libdxfrw extlib/mimalloc
    if [ ! -f extlib/libdxfrw/CMakeLists.txt ]; then
      cp -r ${pkgs.fetchFromGitHub {
        owner = "solvespace";
        repo = "libdxfrw";
        rev = "8359399ff3eb96aec14fb160c9a5bc4796b60717";
        hash = "sha256-tr8ISIxEj6nwMWaO4Vr9GtFFUaEEouAMev9fsNrmRqI=";
      }}/* extlib/libdxfrw/
      chmod -R u+w extlib/libdxfrw
    fi
    if [ ! -f extlib/mimalloc/CMakeLists.txt ]; then
      cp -r ${pkgs.fetchFromGitHub {
        owner = "microsoft";
        repo = "mimalloc";
        rev = "v2.2.2";
        hash = "sha256-sunmnMlPAjlYLU7434CowDiEkgGdzcyYh7PC2vH4TrA=";
      }}/* extlib/mimalloc/
      chmod -R u+w extlib/mimalloc
    fi
  '';

  postPatch = ''
    # Patch out git hash requirement since we're building from local source
    substituteInPlace CMakeLists.txt \
      --replace "include(GetGitCommitHash)" "# include(GetGitCommitHash)" \
      --replace "# set(GIT_COMMIT_HASH 0000000000000000000000000000000000000000)" "set(GIT_COMMIT_HASH themed-fork)"
  '';

  cmakeFlags = [
    "-DENABLE_OPENMP=ON"
    "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
  ];

  meta = with pkgs.lib; {
    description = "Parametric 3D CAD program with themeable text window";
    license = licenses.gpl3Plus;
    platforms = platforms.linux;
    homepage = "https://solvespace.com";
  };
}
