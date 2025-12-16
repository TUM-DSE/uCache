.PHONY: module
module: upstream/seaweedfs/weed/weed
	echo '/weed: $${MODULE_DIR}/upstream/seaweedfs/weed/weed' > usr.manifest

go_version=$(shell go version | grep -o 1.[0-9][0-9])
go_version_new_enough=$(shell echo "$(go_version) >= 1.18" | bc)

ifneq (1,$(go_version_new_enough))
  $(error "Golang version needs to be at least 1.18")
endif

upstream/seaweedfs/weed/weed:
	./GET
	cd upstream/seaweedfs/weed && go build -buildmode=pie -ldflags "-linkmode external"

clean:
	rm -rf ./upstream usr.manifest
