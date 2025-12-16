{
    description = "udos";
    
    inputs =
    {
        nixpkgs-unstable.url = "github:NixOS/nixpkgs/nixos-unstable";
        nixpkgs-stable.url = "github:NixOS/nixpkgs/nixos-24.11";
        nixpkgs-2311.url = "github:NixOS/nixpkgs/nixos-23.11";
        nixpkgs-2211.url = "github:nixos/nixpkgs?ref=22.11";
        nixos-generators = {
            url = "github:nix-community/nixos-generators";
            inputs.nixpkgs.follows = "nixpkgs-2311";
        };
        flake-utils.url = "github:numtide/flake-utils";
        nur-niwa.url = "github:Meandres/nur-niwa";
    };
    
    outputs = 
    {
        self
        , nixpkgs-unstable
        , nixpkgs-stable
        , nixpkgs-2311
        , nixpkgs-2211
        , nixos-generators
        , flake-utils
        , nur-niwa
    } @ inputs:
   (flake-utils.lib.eachSystem ["x86_64-linux"](system:
    let
        unstable-pkgs = nixpkgs-unstable.legacyPackages.${system};
        pkgs = import nixpkgs-2311 {
            inherit system;
            overlays = [ (import ./nix/overlays.nix {inherit inputs; }) ];
        };
        pkgs-stable = nixpkgs-stable.legacyPackages.${system};
        pkgs-2211 = nixpkgs-2211.legacyPackages.${system};
        make-disk-image = import (./nix/make-disk-image.nix);
        selfpkgs = self.packages.x86_64-linux;
        niwa-pkgs = nur-niwa.packages.x86_64-linux;
        python3 = nixpkgs-unstable.legacyPackages.${system}.python3;
        kernelPackages = pkgs.linuxKernel.packages.linux_6_5;
      in {
            packages =
            {
                vmcache = (import ./nix/vmcache.nix { inherit pkgs;});
                duckdb-newest = (import ./nix/duckdb.nix { inherit pkgs;});
                mmapbench = (import ./nix/mmapbench.nix { inherit pkgs;}); 
                kreon = (import ./nix/kreon.nix { inherit pkgs; }); 
                rocksdb_bench = (import ./nix/rocksdb_bench.nix { inherit pkgs; });

                specificKernelPackages = kernelPackages;

                exmap = (import ./nix/exmap.nix { inherit pkgs; inherit kernelPackages; });

                umap-apps = niwa-pkgs.umap-apps;

                linux-image = make-disk-image {
                    config = self.nixosConfigurations.linux-image.config;
                    inherit (pkgs) lib;
                    inherit pkgs;
                    format = "qcow2";
                };
                linux-image-newduckdb = make-disk-image {
                    config = self.nixosConfigurations.linux-image-newduckdb.config;
                    inherit (pkgs) lib;
                    inherit pkgs;
                    format = "qcow2";
                };
    
            };
            
            devShells = {
                default = (pkgs.mkShell #.override { stdenv = pkgs.gcc13Stdenv; }
                {
                    name = "benchmark-devshell";
                    buildInputs =
                    with pkgs;
                    [
                        # tasks
                        ack
                        python3
                        python3.pkgs.pexpect
                        python3.pkgs.colorama
                        gdb
                        pkgs.qemu_full
                        pkgs-stable.just

                        python3.pkgs.click
                        python3.pkgs.seaborn
                        python3.pkgs.pandas
                        python3.pkgs.tqdm
                        python3.pkgs.binary
                        pkgs.libaio
                        pkgs.R
                        pkgs.rPackages.tidyverse
                        pkgs.rPackages.scales
                        niwa-pkgs.driverctl

                        # duckdb
                        cmake
                        ninja
                        openssl
                        gnumake
                        jq
                        sbcl
                        clang_16
                        liburing
                        jemalloc
                        bzip2
                        snappy
                        zstd
                        #lz4
                        pkgs-stable.tbb_2021_11
                        boost

                        # leanstore
                        gflags
                        gtest
                        libgcrypt
                        gbenchmark
                        postgresql
                        fmt
                        wiredtiger
                        sqlite
                        mysql80
                        libmysqlconnectorcpp

                        # OSv
                        osv-boost
                        readline
                        osv-ssl
                        gcc13
                        libgcc
                        niwa-pkgs.tpchgen-rs
                        parallel
                        scc
                        numactl
                        dos2unix

                        niwa-pkgs.umap-apps
                    ];
                    nativeBuildInputs = with pkgs; [
                        autoconf
                        automake
                        binutils
                        bisoncpp
                        gcc13
                        libgcc
                        cmake
                        gnumake
                        gnupatch
                        libedit
                        libtool
                        libvirt
                        ncurses
                        pax-utils # elf security library
                        python311Packages.requests
                        p11-kit # PKCS#11 loader
                        unzip
                        osv-ssl
                        osv-ssl-hdr
                        yaml-cpp
                        xz.out
                        krb5.out
                        libselinux.out
                        libz
                        pkgs.boost175
                        flex
                        bison
                        ninja
                        tbb
                    ];
                    LD_LIBRARY_PATH = "${pkgs.readline}/lib";
                    LUA_LIB_PATH = "${pkgs.lua53Packages.lua}/lib";
                    GOMP_DIR = pkgs.libgcc.out;
                    boost_base = "${pkgs.osv-boost}";
                    BOOST_SO_DIR="${pkgs.boost175}/lib";
                    OPENSSL_DIR="${pkgs.osv-ssl}";
                    OPENSSL_HDR="${pkgs.osv-ssl-hdr}/include";
                    KRB5_DIR="${pkgs.krb5.out}";
                    XZ_DIR="${pkgs.xz.out}";
                    LIBZ_DIR="${pkgs.libz}";
                    LIBSELINUX_DIR="${pkgs.libselinux.out}";

                    CAPSTAN_QEMU_PATH = "${pkgs.qemu}/bin/qemu-system-x86_64";
                });
            };
        }
    )
  ) // (let
      pkgs = import nixpkgs-unstable {
            system = "x86_64-linux";
            overlays = [ (import ./nix/overlays.nix {inherit inputs; }) ];
        };
        stablepkgs = nixpkgs-stable.legacyPackages.x86_64-linux;
        selfpkgs = self.packages.x86_64-linux;
        kernelPackages = selfpkgs.specificKernelPackages;
    in {
        nixosConfigurations = {
            linux-image = nixpkgs-2311.lib.nixosSystem {
                system = "x86_64-linux";
                modules = [ 
                    (import ./nix/image.nix
                    {
                        inherit pkgs;
                        inherit stablepkgs; 
                        inherit (pkgs) lib;
                        inherit selfpkgs;
                        inherit kernelPackages;
                        duckdb = pkgs.duckdb;
                        extraEnvPackages = [ pkgs.mdadm-44 ];
                    })
                    ./nix/nixos-generators-qcow.nix
                ];
            };
            linux-image-newduckdb = nixpkgs-2311.lib.nixosSystem {
                system = "x86_64-linux";
                modules = [ 
                    (import ./nix/image.nix
                    {
                        inherit pkgs;
                        inherit stablepkgs; 
                        inherit (pkgs) lib;
                        inherit selfpkgs;
                        inherit kernelPackages;
                        extraEnvPackages = [ pkgs.mdadm-44 ];
                        duckdb = selfpkgs.duckdb-newest;
                    })
                    ./nix/nixos-generators-qcow.nix
                ];
            };
        };
    });
}
