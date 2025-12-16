proot := source_dir()
qemu_ssh_port := "2222"
user := `whoami`
rep := '1'
ssd_id := '84:00.0'

help:
    just --list

ssh COMMAND="":
    @ ssh \
    -i {{proot}}/nix/keyfile \
    -o StrictHostKeyChecking=no \
    -o UserKnownHostsFile=/dev/null \
    -o IdentityAgent=/dev/null \
    -o LogLevel=ERROR \
    -F /dev/null \
    -p {{qemu_ssh_port}} \
    root@localhost -- "{{COMMAND}}"

linux_vm nb_cpu="1" size_mem="16384": (bind-ssd-vfio ssd_id)
    #!/usr/bin/env bash
    output=$( sudo lspci -vv | grep -A 60 "{{ssd_id}}"  | grep "LnkSta:" )
    if [[ "$my_variable" == *"downgraded"* ]]; then
        echo "Downgraded " $output
    fi
    let "taskset_cores = {{nb_cpu}}-1"
    sudo taskset -c 0-$taskset_cores qemu-system-x86_64 \
        -cpu host \
        -smp {{nb_cpu}} \
        -enable-kvm \
        -m {{size_mem}} \
        -machine q35,accel=kvm,kernel-irqchip=split \
        -device intel-iommu,intremap=on,device-iotlb=on,caching-mode=on \
        -device virtio-serial \
        -fsdev local,id=home,path={{proot}},security_model=none \
        -device virtio-9p-pci,fsdev=home,mount_tag=home,disable-modern=on,disable-legacy=off \
        -fsdev local,id=scratch,path=/scratch/{{user}},security_model=none \
        -device virtio-9p-pci,fsdev=scratch,mount_tag=scratch,disable-modern=on,disable-legacy=off \
        -fsdev local,id=nixstore,path=/nix/store,security_model=none \
        -device virtio-9p-pci,fsdev=nixstore,mount_tag=nixstore,disable-modern=on,disable-legacy=off \
        -drive file={{proot}}/VMs/linux-image.qcow2 \
        -net nic,netdev=user.0,model=virtio \
        -netdev user,id=user.0,hostfwd=tcp:127.0.0.1:{{qemu_ssh_port}}-:22 \
        -nographic \
        -device vfio-pci,host={{ssd_id}}

linux-image-init:
    #!/usr/bin/env bash
    set -x
    set -e
    echo "Initializing disk for the VM"
    mkdir -p {{proot}}/VMs

    # build images fast
    overwrite() {
        install -D -m644 {{proot}}/VMs/ro/nixos.qcow2 {{proot}}/VMs/$1.qcow2
        qemu-img resize {{proot}}/VMs/$1.qcow2 +8g
    }

    nix build .#linux-image --out-link {{proot}}/VMs/ro
    overwrite linux-image

osv-image-init image="":
    #!/usr/bin/env bash
    cd {{proot}}/osv
    ./scripts/build -j fs=ramfs image={{image}}
    if [[ "{{image}}" == *"duckdb"* ]]; then
        ./scripts/build -j fs=ramfs image={{image}}
    fi
    cp {{proot}}/osv/build/last/usr.img {{proot}}/VMs/osv_{{image}}.img

osv_vm nb_cpu="1" size_mem_giga="4G" image="" extra_args="": (bind-ssd-vfio ssd_id)
    #!/usr/bin/env bash
    output=$( sudo lspci -vv | grep -A 60 "{{ssd_id}}"  | grep "LnkSta:" )
    if [[ "$my_variable" == *"downgraded"* ]]; then
        echo "Downgraded " $output
    fi
    {{proot}}/osv/scripts/imgedit.py setargs {{proot}}/VMs/osv_{{image}}.img "{{extra_args}}"
    let "taskset_cores = {{nb_cpu}}-1"
    sudo taskset -c 0-$taskset_cores qemu-system-x86_64 \
    -m {{size_mem_giga}} \
    -smp {{nb_cpu}} \
    -vnc :1 \
    -gdb tcp::1234,server,nowait \
    -device virtio-blk-pci,id=blk0,drive=hd0,scsi=off,bootindex=0 \
    -drive file={{proot}}/VMs/osv_{{image}}.img,if=none,id=hd0,cache=none,aio=native \
    -device vfio-pci,host={{ssd_id}} \
    -netdev user,id=un0,net=192.168.122.0/24,host=192.168.122.1 \
    -device virtio-net-pci,netdev=un0 \
    -device virtio-rng-pci \
    -enable-kvm \
    -cpu host,+x2apic \
    -chardev stdio,mux=on,id=stdio,signal=off \
    -mon chardev=stdio,mode=readline \
    -device isa-serial,chardev=stdio

bind-ssd-vfio pci_id="":
    #!/usr/bin/env bash
    current_driver=$(sudo driverctl list-devices | grep {{pci_id}} | awk '{print $2 }')
    [ $current_driver = "vfio-pci" ] || sudo driverctl set-override 0000:{{pci_id}} vfio-pci

bind-ssd-nvme pci_id="":
    #!/usr/bin/env bash
    current_driver=$(sudo driverctl list-devices | grep {{pci_id}} | awk '{print $2 }')
    if [ "$current_driver" != "nvme" ]
    then
        sudo driverctl set-override 0000:{{pci_id}} nvme
    fi

reset_fs:
    #!/usr/bin/env bash
    current_driver=$(sudo driverctl list-devices | grep {{ssd_id}} | awk '{print $2 }')
    if [ "$current_driver" != "nvme" ]
    then
        sudo driverctl set-override 0000:{{ssd_id}} nvme
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
    sudo touch $tmpdir/cache
    sudo fallocate -z -l 1500G $tmpdir/cache
    sudo dd if=/dev/zero of=$tmpdir/cache bs=1M count=1331200 oflag=nonblock,direct status=progress
    sudo umount $tmpdir
    sudo driverctl set-override 0000:{{ssd_id}} vfio-pci

check_downgraded_link:
    #!/usr/bin/env bash
    output=$( sudo lspci -vv | grep -A 60 "{{ssd_id}}"  | grep "LnkSta:" )
    if [[ "$my_variable" == *"downgraded"* ]]; then
        echo "Downgraded " $output
    fi
