#!/usr/bin/env bash

python microbench.py
python duckdb.py
python io_perf.py  
python kvs.py  
python vmcache.py
