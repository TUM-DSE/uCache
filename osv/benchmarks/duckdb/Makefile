.PHONY: module clean distclean

module: duckdb/duckdb

duckdb/duckdb: duckdb
	cd duckdb; CXX_STANDARD=17 DUCKDB_PLATFORM=linux_amd64 BUILD_BENCHMARK=1 BUILD_TPCH=1 make -j
	#cd duckdb; EXTENSION_CONFIGS=.github/config/bundled_extensions.cmake CXX_STANDARD=17 DUCKDB_PLATFORM=linux_amd64 BUILD_BENCHMARK=1 BUILD_HTTPFS=1 make reldebug

clean:
	if [ -d duckdb ]; then cd duckdb && make clean; fi

distclean:
	rm -rf duckdb
