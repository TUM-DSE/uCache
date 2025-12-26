import? "../justfile"

build_images:
  just linux-image-init
  just osv-image-init mmapbench 
  just osv-image-init nvmebench 
  just osv-image-init integrated_vmcache
  just osv-image-init duckdb

check_vms:
  just linux_vm > /dev/null 2> /dev/null &
  sleep 30
  just ssh "echo 'Ssh into the Linux VM works'"
  just ssh "poweroff"
  sleep 20
  just osv_vm 1 4G mmapbench "--mount-nvme-ext /mmapbench 2 2 1 4096 5"

copy_tpch_files folder="":
  #!/usr/bin/env bash
  current_driver=$(sudo driverctl list-devices | grep {{ssd_id}} | awk '{print $2 }')
  if [ "$current_driver" != "nvme" ]
  then
  sudo driverctl set-override 0000:{{ssd_id}} nvme
  fi
  if [ "{{folder}}" == "" ]
  then
  echo "Please specify a folder containing the TPCH parquet files."
  exit 1
  fi
  nvme_path="/dev/$(ls /sys/bus/pci/devices/0000\:{{ssd_id}}/nvme)n1"
  read -p "This will overwrite SSD {{ssd_id}} (Host block device path: $nvme_path). Are you sure you want to continue? (y/n) " -n 1 -r
  echo
  if [[ ! $REPLY =~ ^[Yy]$ ]]; then
  exit 1
  fi
  sudo mke2fs -F -t ext4 -O ^metadata_csum ${nvme_path}
  tmpdir=$(mktemp -d)
  sudo mount ${nvme_path} $tmpdir
  sudo mkdir -p $tmpdir/tpch
  sudo rsync -r --info=progress2 --info=name0 {{folder}}/* $tmpdir/tpch
  sudo umount $tmpdir
  sudo driverctl set-override 0000:{{ssd_id}} vfio-pci
