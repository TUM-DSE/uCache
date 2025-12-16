.PHONY: module clean

SRC = $(shell readlink -f ../..)

nginx_exe_path = $(shell realpath $$(which nginx)) 

module:
	$(SRC)/scripts/manifest_from_host.sh $(nginx_exe_path) > usr.manifest
	echo '/nginx.conf: $${MODULE_DIR}/nginx.conf' >> usr.manifest
	echo '/var/log/nginx: ->/tmp' >> usr.manifest
	echo '/var/lib/nginx: ->/tmp' >> usr.manifest

clean:
	rm -rf usr.manifest
