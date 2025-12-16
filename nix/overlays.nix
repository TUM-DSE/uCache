{ inputs, ... }:

final: _prev: {
  capstan = _prev.callPackage ./pkgs/capstan.nix { };
  osv-boost = _prev.boost175.override { enableStatic = true; enableShared = false; };
  osv-ssl = inputs.nixpkgs-2211.legacyPackages.${_prev.system}.openssl_1_1.out;
  osv-ssl-hdr = inputs.nixpkgs-2211.legacyPackages.${_prev.system}.openssl_1_1.dev;
  mdadm-44 = _prev.mdadm.overrideAttrs (
    old: rec{ 
      version = "4.4"; 
      src = _prev.fetchurl { 
        url = "https://git.kernel.org/pub/scm/utils/mdadm/mdadm.git/snapshot/mdadm-${version}.tar.gz";
        sha256 = "sha256-aA/tUyhXCI4M2HxWwAAzrjXq4KP5y34VI7NFuocX+5M="; 
      };
    }
    );
}
