.PHONY: module
module: httpserver.so
	echo '/httpserver.so: $${MODULE_DIR}/httpserver.so' > usr.manifest

httpserver.so: httpserver.go
	go build -buildmode=pie -ldflags "-linkmode external" -o httpserver.so httpserver.go
# Please note that executable built following methods would work as well:
#  'go build -buildmode=exe -ldflags "-linkmode external" -o httpserver.so httpserver.go'
#   - on both older 1.12.6 and newer 1.15.8 version of Golang
#  'go build -buildmode=pie -o httpserver.so httpserver.go' - on older 1.12.6
# The following method does NOT work until we fix the issue #1141
#  'go build -buildmode=pie -o httpserver.so httpserver.go' - on newer 1.15.8
#  'go build -buildmode=exe -o httpserver.so httpserver.go'
#   - on both older 1.12.6 and newer 1.15.8 version of Golang

clean:
	rm -f httpserver*.so usr.manifest
