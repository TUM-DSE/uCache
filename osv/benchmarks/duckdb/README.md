# Duckdb port to OSv

## To build:
- For now, the manifest only includes tpch/sf1 benchmarks. To extend the benchmarks, add new ones to the `usr.manifest` file (don't forget the corresponding answer/ folder too).
`./scripts/build -j fs_size_mb=4096 image=duckdb`

## To run:
`./scripts/run.py -e "/benchmark_runner benchmark/tpch/sf1/q01.benchmark"`


## Misc
if linking errors when running: add `__isoc23_strtol = strtol;` to `libc/aliases.ld`
fyi: `libc/aliases.ld` is not tracked by the Makefile. You have to `./scripts/build clean`
