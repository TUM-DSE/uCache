import? "../justfile"
result_file := proot+"/benchmarks/results/microbench.csv"

cores := '64'
threads := "1 16 32 64"
page_size := "4096 8192 16384 32768 65536"
virtSize := '1000'
physSize := '100'
ssd_path := "/dev/nvme0n1"
linux_vm_phys_mem := '110G'
osv_vm_phys_mem := '150G'
runfor := '120'
mem_size := '200'
phys_quota := '128 64 32 16'
big_osv_vm_phys_mem := '200G'
runfor_quota := '120'

run:
  #!/usr/bin/env bash
  cat /dev/null > {{result_file}}
   
  just linux_vm {{cores}} {{linux_vm_phys_mem}} > /dev/null 2> /dev/null &
  sleep 30
  
  echo "vm_size,pm_size,system,workload,pageSize,thread,time,throughput" > {{result_file}}
  
  total_rep=$(expr $(echo {{threads}} | wc -w) \* {{rep}} \+ $(echo {{phys_quota}} | wc -w) \* {{rep}})
  cnt=1
  
  for i in `seq 1 {{rep}}`
  do
    for t in {{threads}}
    do
      echo "Linux: $cnt/$total_rep"
      let cnt++
      just ssh "echo 1 > /proc/sys/vm/drop_caches; mmapbench {{ssd_path}} {{virtSize}} 1 $t {{runfor}}" | grep -v system | sed "s/^/{{virtSize}},{{physSize}},/" >> {{result_file}} 
      sleep 5
    done
  done
  just ssh "poweroff"
  sleep 20
  just check_downgraded_link
   
  for quota in {{phys_quota}}
  do
    just linux_vm {{cores}} ${quota}G > /dev/null 2> /dev/null &
    sleep 30
    for i in `seq 1 {{rep}}`
    do
      echo "Linux: $cnt/$total_rep"
      let cnt++
      just ssh "echo 1 > /proc/sys/vm/drop_caches; mmapbench {{ssd_path}} {{mem_size}} 1 64 {{runfor_quota}}" | grep -v system | sed "s/^/{{mem_size}},$quota,/" >> {{result_file}} 
      sleep 5
    done
    just ssh "poweroff"
    sleep 20
    just check_downgraded_link
  done
  
  total_rep=$(expr $(echo {{threads}} | wc -w) \* {{rep}} \+ $(echo {{phys_quota}} | wc -w) \* {{rep}} \+ $(echo {{page_size}} | wc -w) \* {{rep}} )
  cnt=1

  for i in `seq 1 {{rep}}`
  do
    for t in {{threads}}
    do
      echo "uCache: $cnt/$total_rep"
      let cnt++
      just osv_vm $t {{osv_vm_phys_mem}} mmapbench "--mount-nvme-ext /mmapbench {{virtSize}} {{physSize}} 1 4096 {{runfor}}" | grep -v system | sed "s/^/{{virtSize}},{{physSize}},/" >> {{result_file}}
      sleep 5
      just check_downgraded_link
    done
    for quota in {{phys_quota}}
    do
      echo "uCache: $cnt/$total_rep"
      let cnt++
      just osv_vm $t {{big_osv_vm_phys_mem}} mmapbench "--mount-nvme-ext /mmapbench {{mem_size}} $quota 1 4096 {{runfor_quota}}" | grep -v system  | sed "s/^/{{mem_size}},$quota,/" >> {{result_file}}
      sleep 5
      just check_downgraded_link
    done
    for psize in {{page_size}}
    do
      echo "uCache: $cnt/$total_rep"
      let cnt++
      just osv_vm 64 {{osv_vm_phys_mem}} mmapbench "--mount-nvme-ext /mmapbench {{virtSize}} {{physSize}} 1 $psize {{runfor}}" | grep -v system  | sed "s/^/{{virtSize}},{{physSize}},/" >> {{result_file}}
      sleep 5
      just check_downgraded_link
    done
  done
  
  dos2unix {{result_file}}
  sed -i '/system\|mmap,\|ucache/!d' {{result_file}}
