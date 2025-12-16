.PHONY: module
module: hello.so hello-static-non-pie hello-static-pie
	echo '/hello.so: $${MODULE_DIR}/hello.so' > usr.manifest
	echo '/hello-static-non-pie: $${MODULE_DIR}/hello-static-non-pie' >> usr.manifest
	echo '/hello-static-pie: $${MODULE_DIR}/hello-static-pie' >> usr.manifest

hello.so: hello.go
	go build -buildmode=pie -ldflags "-linkmode external" -o hello.so hello.go

hello-static-pie: hello.go
	go build -buildmode=pie -ldflags '-linkmode external -extldflags "--static-pie"' -o hello-static-pie hello.go

hello-static-non-pie: hello.go
	go build --ldflags '-extldflags "-static"' -o hello-static-non-pie hello.go

# Please note that executable built following methods would work as well:
#  'go build -buildmode=exe -ldflags "-linkmode external" -o hello.so hello.go'
#   - on both older 1.12.6 and newer 1.15.8 version of Golang
#  'go build -buildmode=pie -o hello.so hello.go' - on older 1.12.6

clean:
	rm -f hello*.so hello-static* usr.manifest
