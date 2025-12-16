.PHONY: module
module: httpclient
	echo '/httpclient: $${MODULE_DIR}/httpclient' > usr.manifest

httpclient: httpclient.go
	go build -buildmode=pie -ldflags "-linkmode external" -o httpclient httpclient.go

clean:
	rm -f httpclient usr.manifest
