{ pkgs, kernelPackages, ... }:

pkgs.stdenv.mkDerivation {
    name = "exmap";
    
    src = pkgs.fetchFromGitHub {
      owner = "tuhhosg";
      repo = "exmap";
      /*rev="6ea8f03362bc67e66196306294008b1470f03062";
      hash = "sha256-pU5do1DnyT+3NkzlRDHPKu84UEklAJhfQkaLtsCz1z8=";*/
      rev = "48bef8f1843b136d34e9543b2fa68965cb1961d9";
      hash = "sha256-xjn096gsYPpaff9h1lD6Zpx8VozwQ/oK65UB/N0H/oY=";
    };
    
    kernel = kernelPackages.kernel;
    hardeningDisable = [ "pic" "format" ];
    buildInputs = [ pkgs.nukeReferences ];
    nativeBuildInputs = kernelPackages.kernel.moduleBuildDependencies;
    
    makeFlags = [
      "-C"
      "${kernelPackages.kernel.dev}/lib/modules/${kernelPackages.kernel.modDirVersion}/build"
      "M=$(sourceRoot)"
      "VERSION=${kernelPackages.kernel.version}"
    ];
    
    buildPhase = ''
      make modules KDIR=${kernelPackages.kernel.dev}/lib/modules/${kernelPackages.kernel.modDirVersion}/build -j $NIX_BUILD_CORES
    '';
    
    installPhase = ''
      mkdir -p $out/lib/modules/${kernelPackages.kernel.modDirVersion}/misc
      for x in $(find . -name '*.ko'); do
        nuke-refs $x
        cp $x $out/lib/modules/${kernelPackages.kernel.modDirVersion}/misc/
      done
    
      mkdir -p $out/include
      cp -r module/linux $out/include
    '';
    
    meta = with pkgs.lib; {
      description = "ExMap - Fully explicit memory-mapped file I/O";
      homepage = "https://github.com/tuhhosg/exmap.git";
    };
}

