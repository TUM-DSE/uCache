import? "../justfile"
result_file := proot+"/benchmarks/results/vmcache.csv"

cores := '64'
taskset_cores := "$(( {{cores}} - 1 ))"
virtgb := '1152'
physgb := '128'
vm_phys_mem := '200G'
datasize := '5120'
runfor := '300'

run:
  #!/usr/bin/env bash
  cat /dev/null > {{result_file}}
  let "taskset_cores = {{cores}}-1"

  just linux_vm {{cores}} {{vm_phys_mem}} > /dev/null 2> /dev/null &
  sleep 30

  for i in `seq 1 {{rep}}`
  do 
    just ssh "THREADS={{cores}} BLOCK=/dev/nvme0n1 VIRTGB={{virtgb}} PHYSGB={{physgb}} DATASIZE={{datasize}} RUNFOR={{runfor}} EXMAP=0 taskset -c 0-$taskset_cores vmcache" | grep "tpcc" >> {{result_file}}
    sleep 10
    just ssh "THREADS={{cores}} BLOCK=/dev/nvme0n1 VIRTGB={{virtgb}} PHYSGB={{physgb}} DATASIZE={{datasize}} RUNFOR={{runfor}} EXMAP=1 taskset -c 0-$taskset_cores vmcache" | grep "tpcc" >> {{result_file}}
    sleep 10
  done

  just ssh "poweroff"
  just check_downgraded_link

  sleep 20

  just reset_fs no

  sleep 20
  
  for i in `seq 1 {{rep}}`
  do
    just osv_vm {{cores}} {{vm_phys_mem}} integrated_vmcache "--mount-nvme-ext --env=VIRTGB={{virtgb}} --env=PHYSGB={{physgb}} --env=DATASIZE={{datasize}} --env=RUNFOR={{runfor}} integrated_vmcache" >> {{result_file}}
    sleep 10
    just check_downgraded_link
  done

  #sed -i -e '/^[0-9]/!d' -e '2,${/^ts/d}' filename
