import? "../justfile"

build_images:
  just linux-image-init
  just osv-image-init native-example
  just osv-image-init mmapbench 
  just osv-image-init integrated_vmcache
  just osv-image-init duckdb

check_vms:
  just linux_vm > /dev/null 2> /dev/null &
  sleep 30
  just ssh "echo 'Ssh into the Linux VM works'"
  just ssh "poweroff"
  just osv_vm 1 4G native-example "/hello"

precondition_ssd:
  #!/usr/bin/env bash
  just linux_vm 16 32768 > /dev/null 2> /dev/null &
  sleep 30
  just ssh "spdk_nvme_perf -c 0xff -w randwrite -o 32768 -t 20 -q 128 /dev/nvme0n1"
  just ssh "poweroff"
