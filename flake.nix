{
  description = "Conagotchi 2026 badge firmware and badgectl host tools";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs = { self, nixpkgs }:
    let
      systems = [ "x86_64-linux" "aarch64-linux" "x86_64-darwin" "aarch64-darwin" ];
      forAllSystems = f: nixpkgs.lib.genAttrs systems (system: f nixpkgs.legacyPackages.${system});
    in
    {
      # Tools for building and flashing. The Xtensa Rust toolchain itself is
      # not in nixpkgs: run `espup install --name esp-new --toolchain-version 1.97.0.0
      # --targets esp32s3` once inside this shell, then `. ~/export-esp.sh`.
      devShells = forAllSystems (pkgs: {
        default = pkgs.mkShell {
          packages = [
            pkgs.espflash
            pkgs.probe-rs-tools
            pkgs.espup
            pkgs.rustup
            pkgs.protobuf
            pkgs.python3
            pkgs.pkg-config
          ] ++ pkgs.lib.optionals pkgs.stdenv.isLinux [ pkgs.udev ];
        };
      });

      # `nix run .#flash -- conagotchi-merged.bin [espflash options]` writes a
      # merged release image at 0x0. The image ends below 0x200000, so the
      # conag (contacts, radio key), vfs and doom partitions are kept.
      apps = forAllSystems (pkgs:
        let
          flash = pkgs.writeShellApplication {
            name = "conagotchi-flash";
            runtimeInputs = [ pkgs.espflash ];
            text = ''
              if [ "$#" -lt 1 ]; then
                echo "usage: nix run .#flash -- <conagotchi-merged.bin> [espflash write-bin options]" >&2
                exit 64
              fi
              image="$1"
              shift
              if [ ! -f "$image" ]; then
                echo "error: $image not found" >&2
                exit 66
              fi
              size=$(wc -c < "$image")
              if [ "$size" -gt 2097152 ]; then
                echo "error: $image is $size bytes; a Conagotchi image must end below 0x200000" >&2
                exit 65
              fi
              exec espflash write-bin "$@" 0x0 "$image"
            '';
          };
        in
        {
          flash = {
            type = "app";
            program = "${flash}/bin/conagotchi-flash";
          };
          default = self.apps.${pkgs.stdenv.hostPlatform.system}.flash;
        });
    };
}
