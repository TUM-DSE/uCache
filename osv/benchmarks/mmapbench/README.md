## Run in OSv
```bash
./scripts/build -j $(nproc) fs=ramfs image=mmapbench
sudo ./scripts/bind_nvme.sh vfio c3:00.0
./scripts/run.py --pass-pci c3:00.0 -e "/mmapbench /none 20 1 0"
```


### Current Issues
```
OSv v0.57.0-411-g7dbcdea1
model: 'KIOXIA KCMYXRUG3T84' sn: 'XDL0A0BH0LM3' fr: '1UETE103' page size = 4096, queue count = 128/128 (max queue count), queue size = 256/16384 (max queue size), block count = 0x1bf1f72b0, block size = 512, max block io = 4096
eth0: 192.168.122.15
Booted up in 1331.94 ms
Cmdline: /mmapbench /none 20 1 0
uCache initialized with 64 GB of physical memory available
Available files on the ssd:
Creating a file at 0 with 4294967296 blocks (of size 512)
Added a vm_area @ 0x200040000000 of size: 2048GB, with pageSize: 4096
dev,seq,hint,threads,time,workGB,tlb,readGB,CPUwork
/none,1,0,20,1,0.0868378,243
Assertion failed: ret == 0 (core/ucache.cc: readBufferToTmp: 687)
```
