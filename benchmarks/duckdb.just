import? "../justfile"
result_file := proot+"/benchmarks/results/duckdb.csv"

cores := '64'
queries := "01 02 03 04 05 06 07 08 09 10 11 12 13 14 16 17 18 19 20 21 22"
vm_phys_mem := '100G'
cache_mem := '20'
linux_phys_mem := '25G'

run:
  #!/usr/bin/env bash
  #cat /dev/null > {{result_file}}
  #echo "query,repetition,time,system" >> {{result_file}}
  # 
  #just linux_vm {{cores}} {{linux_phys_mem}} > /dev/null 2> /dev/null & 
  #sleep 30
  #
  #just ssh "mkdir -p /nvme; mount /dev/nvme0n1 /nvme"
  #
  #for i in `seq 1 {{rep}}`
  #do
  #  for q in {{queries}}
  #  do
  #    just ssh "echo 1 > /proc/sys/vm/drop_caches; rm -f /nvme/db.tmp; duckdb -no-stdin -c 'set enable_progress_bar = false' -c '.timer on' -c '.read /root/osv/benchmarks/duckdb/tpch/q'${q}'_repeat.sql' /nvme/db.tmp" | grep "Run Time" | awk -v query=$q '{printf "%s,%s,%.3f,duckdb\n", query, NR, $5}' >> {{result_file}}

  #    sleep 10
  #  done
  #done
  #
  #just ssh "poweroff" > /dev/null 2> /dev/null
  #sleep 30
  #just check_downgraded_link 
  #
  for i in `seq 1 {{rep}}`
  do
    for q in {{queries}}
    do
      just osv_vm {{cores}} {{vm_phys_mem}} duckdb '--mount-nvme-ext --env=PHYSGB={{cache_mem}} /duckdb -no-stdin -c \"set enable_progress_bar = false\" -c \".timer on\" -c \".read /tpch/q'$q'_repeat.sql\"' | grep "Run Time" | awk -v query=$q '{printf "%s,%s,%.3f,ucache\n", query, NR, $5}' >> {{result_file}}
      sleep 10
      just check_downgraded_link
  done
  done

  dos2unix {{result_file}}

# minio server /scratch/ilya/minio
