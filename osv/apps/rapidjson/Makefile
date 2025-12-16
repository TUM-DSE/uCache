SRC = $(shell readlink -f ../..)

upstream/rapidjson/.git:
	mkdir -p upstream
	cd upstream && git clone --depth 1 https://github.com/Tencent/rapidjson.git
	cd upstream/rapidjson && git submodule update --init

upstream/rapidjson/build/bin/unittest: upstream/rapidjson/.git
	mkdir -p upstream/rapidjson/build
	cd upstream/rapidjson/build && cmake ..
	cd upstream/rapidjson/build && $(MAKE)

module: upstream/rapidjson/build/bin/unittest
	$(SRC)/scripts/manifest_from_host.sh upstream/rapidjson/build/bin/unittest > usr.manifest
	echo "/bin/**: ${SRC}/apps/rapidjson/upstream/rapidjson/bin/**" >> usr.manifest
	strip upstream/rapidjson/build/bin/unittest

clean:
	rm -rf upstream usr.manifest
