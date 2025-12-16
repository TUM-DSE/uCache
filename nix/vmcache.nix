{ pkgs, ... }:

pkgs.stdenv.mkDerivation {
    name = "vmcache";

	  src = pkgs.fetchgit {
			url = "https://github.com/viktorleis/vmcache";
	    rev = "5a04fd6eaf32a25ec60c7db3093275893ff21bbb";
			hash = "sha256-CtC34i7I5WBgGqblT1irfpK0ECthhFEujP69PLFisuU=";
	  };

	  nativeBuildInputs = [ pkgs.libaio ];

	  installPhase = ''
	    mkdir -p $out/bin
	    cp vmcache $out/bin/vmcache
	  '';

	  meta = with pkgs.lib; {
	    description = "Virtual-Memory Assisted Buffer Management";
	    homepage = "https://github.com/viktorleis/vmcache";
	  };
}
