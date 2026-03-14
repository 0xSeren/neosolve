{
  description = "SolveSpace with themeable text window";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = nixpkgs.legacyPackages.${system};
      solvespace-themed = pkgs.callPackage ./default.nix { src = self; };
    in {
      packages.${system} = {
        default = solvespace-themed;
        solvespace-themed = solvespace-themed;
      };

      devShells.${system}.default = pkgs.mkShell {
        inputsFrom = [ solvespace-themed ];

        packages = with pkgs; [
          gdb
          clang-tools
          qt6.qtbase
          qt6.wrapQtAppsHook
          opencascade-occt
          gsettings-desktop-schemas
          dconf
        ];

        # GSettings environment for running unwrapped binaries during development
        GIO_EXTRA_MODULES = "${pkgs.dconf.lib}/lib/gio/modules";
        XDG_DATA_DIRS = "${pkgs.gsettings-desktop-schemas}/share/gsettings-schemas/${pkgs.gsettings-desktop-schemas.name}:${pkgs.gtk3}/share/gsettings-schemas/${pkgs.gtk3.name}";
      };
    };
}
