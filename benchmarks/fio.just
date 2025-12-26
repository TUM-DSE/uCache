import? "../justfile"
result_file := proot+"/benchmarks/results/io.csv"

cores := '64'
threads_str := "0xf 0xffffffff 0xffffffffffff"
threads := "4 32 64"
batch_size := "1 32 128"
ssd_path := "/dev/nvme0n1"
vm_phys_mem := '150G'
ramp_time := '10'
runtime := '20'

run:
  #!/usr/bin/env bash
  cat /dev/null > {{result_file}}

  declare -A thread_map
  thread_map[4]="0xf"
  thread_map[32]="0xffffffff"
  thread_map[64]="0xffffffffffffffff"
  echo "system,workload,ioSize,threads,time,batchSize,bandwidth" > {{result_file}}
  
  just linux_vm {{cores}} {{vm_phys_mem}} > /dev/null 2> /dev/null &
  sleep 30

  total_rep=$(expr $(echo {{threads_str}} | wc -w) \* $(echo {{batch_size}} | wc -w) \* {{rep}} \* 2)
  cnt=0

  for i in `seq 1 {{rep}}`
  do
    for t in {{threads}}
    do
      for qd in {{batch_size}}
      do
        iosize=$(expr $qd \* 4096)
        echo -ne "Linux: $cnt/$total_rep\r"
        just ssh "echo 1 > /proc/sys/vm/drop_caches"
        sleep 5
        just ssh "spdk_nvme_perf -q 1 -o $iosize -w randread -t {{runtime}} -a {{ramp_time}} -c ${thread_map[$t]} {{ssd_path}} 2> /dev/null" | grep "Total" | awk -v depth=$qd -v threads=$t '{printf "spdk,read,4096,%s,0,%s,%.3f\n", threads, depth, $4}' >> {{result_file}}
        let cnt++
        sleep 5
        just ssh "echo 1 > /proc/sys/vm/drop_caches"
        sleep 5
        echo -ne "Linux: $cnt/$total_rep\r"
        echo -n "libaio,read,4096,$t,0,$qd," >> {{result_file}}
        just ssh "fio --name=libaio --ioengine=libaio --iodepth=1 --rw=randread --bs=$iosize --ramp_time={{ramp_time}} --runtime={{runtime}} --thread=1 --group_reporting=1 --numjobs=$t --cpus_allowed=0-63 --cpus_allowed_policy=split --filename={{ssd_path}}" | grep "bw=" | awk '{print $2;}' >> {{result_file}}
        let cnt++
      done
    done
  done

  just ssh "poweroff"
  sleep 30
  just check_downgraded_link

  total_rep=$(expr $(echo {{threads_str}} | wc -w) \* $(echo {{batch_size}} | wc -w) \* {{rep}})
  cnt=0
  for i in `seq 1 {{rep}}`
  do
    for t in {{threads}}
    do
      for qd in {{batch_size}}
      do
        echo -ne "uCache: $cnt/$total_rep\r"
        just osv_vm $t {{vm_phys_mem}} nvmebench "--mount-nvme-ext /nvmebench r 4096 $qd {{ramp_time}} {{runtime}}" | grep "ufs" >> {{result_file}}
        sleep 5
        let cnt++
        just check_downgraded_link
      done
    done
  done
