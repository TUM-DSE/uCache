{ pkgs, ... }:

pkgs.stdenv.mkDerivation {
    name = "mmapbench";

	  src = pkgs.fetchgit {
			url = "https://github.com/Meandres/mmapbench.git";
	    rev = "61062b7537b6db5d629b4a5a44313fa0b8d55103";
			hash = "sha256-cGMhNWoyaw/G59tBkNydQHV7n3WLsRwDOz5oBvL47ZY=";
	  };

	  nativeBuildInputs = [ pkgs.tbb pkgs.boost ];

		buildPhase = ''
			g++ -O3 mmapbench.cpp -o mmapbench -ltbb -pthread
		'';

	  installPhase = ''
	    mkdir -p $out/bin
	    cp mmapbench $out/bin/mmapbench
	  '';

	  meta = with pkgs.lib; {
	    description = "Benchmarks and scripts from the CIDR 2022 paper Are You Sure You Want to Use MMAP in Your Database Management System?";
	    homepage = "https://github.com/viktorleis/mmapbench.git";
	  };
}
