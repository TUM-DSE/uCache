# This configuration describes a VM image with vmcache and exmap
{ selfpkgs, lib, pkgs, stablepkgs, kernelPackages, duckdb, extraEnvPackages ? [], ... }:
with lib;
let
  resize = pkgs.writeShellScriptBin "resize" ''
    export PATH=${pkgs.coreutils}/bin
    if [ ! -t 0 ]; then
      # not a interactive...
      exit 0
    fi
    TTY="$(tty)"
    if [[ "$TTY" != /dev/ttyS* ]] && [[ "$TTY" != /dev/ttyAMA* ]] && [[ "$TTY" != /dev/ttySIF* ]]; then
      # probably not a known serial console, we could make this check more
      # precise by using `setserial` but this would require some additional
      # dependency
      exit 0
    fi
    old=$(stty -g)
    stty raw -echo min 0 time 5

    printf '\0337\033[r\033[999;999H\033[6n\0338' > /dev/tty
    IFS='[;R' read -r _ rows cols _ < /dev/tty

    stty "$old"
    stty cols "$cols" rows "$rows"
  '';
in
{
  networking = {
    hostName = "guest";
  };
  services.sshd.enable = true;
  networking.firewall.enable = true;

  users.users.root.password = "password";
  services.openssh.settings.PermitRootLogin = lib.mkDefault "yes";
  users.users.root.openssh.authorizedKeys.keys = [
    (builtins.readFile ./keyfile.pub)
  ];
  services.getty.autologinUser = lib.mkDefault "root";

  fileSystems."/root" = {
      device = "home";
      fsType = "9p";
      options = [ "trans=virtio" "nofail" "msize=104857600" ];
  };

  fileSystems."/scratch" = {
    device = "scratch";
    fsType = "9p";
    options = [ "trans=virtio" "nofail" "msize=104857600" ];
  };

  # mount host nix store, but use overlay fs to make it writeable
  fileSystems."/nix/.ro-store-vmux" = {
    device = "nixstore";
    fsType = "9p";
    options = [ "ro" "trans=virtio" "nofail" "msize=104857600" ];
    neededForBoot = true;
  };

  fileSystems."/nix/store" = {
    device = "overlay";
    fsType = "overlay";
    options = [
      "lowerdir=/nix/.ro-store-vmux"
      "upperdir=/nix/.rw-store/store"
      "workdir=/nix/.rw-store/work"
    ];
    neededForBoot = true;
    depends = [
      "/nix/.ro-store-vmux"
      "/nix/.rw-store/store"
      "/nix/.rw-store/work"
    ];
  };

  boot.initrd.availableKernelModules = [ "overlay" ];

  nix.extraOptions = ''
      experimental-features = nix-command flakes
  '';
  environment.systemPackages = [
    pkgs.vim
	  pkgs.git
	  pkgs.libaio
	  pkgs.gnumake
	  pkgs.gnat13
	  pkgs.fio
    pkgs.nvme-cli
	  selfpkgs.vmcache
    selfpkgs.mmapbench
    pkgs.just
    pkgs.python3
    pkgs.jq
    pkgs.parted
    pkgs.gptfdisk
    kernelPackages.perf
    stablepkgs.disko
    resize
    pkgs.spdk
    duckdb
    selfpkgs.rocksdb_bench
    selfpkgs.umap-apps
  ] ++ extraEnvPackages;

  boot.kernelPackages = kernelPackages;
  boot.extraModulePackages = [ selfpkgs.exmap ];
  boot.kernelModules = ["vfio" "vfio-pci" "exmap"];
  boot.kernelParams = [
      "nokaslr"
      "iomem=relaxed"
      "default_hugepagesz=2MB"
      "hugepagesz=2MB"
      "hugepages=2048"
      "nvme_core.multipath=0"
  ];
  boot.kernel.sysctl = {
    "vm.overcommit_memory" = 1;
  };
  boot.swraid.enable = true;
  systemd.services.growpart.enable = false;

  system.stateVersion = "24.05";

  console.enable = true;
  environment.loginShellInit = "${resize}/bin/resize";
  systemd.services."serial-getty@".environment.TERM = "xterm-256color";
  systemd.services."serial-getty@ttys0".enable = true;
  services.qemuGuest.enable = true;
}
