.PHONY: module
module: net_iterate.so
	echo '/net_iterate.so: $${MODULE_DIR}/net_iterate.so' > usr.manifest

net_iterate.so: net_iterate.go
	go build -buildmode=pie -ldflags "-linkmode external" -o net_iterate.so net_iterate.go

clean:
	rm -f net_iterate*.so usr.manifest
