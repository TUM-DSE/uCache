This is an example of [SeaweedFS](https://github.com/chrislusf/seaweedfs) golang app running on OSv.

Here are the examples on how to build and run it:
* master
```
./scripts/build image=seaweedfs
./scripts/run.py --forward 'tcp::9333-:9333'
```

* server
```
./scripts/build image=seaweedfs.server
./scripts/run.py --forward 'tcp::9333-:9333'
```

### Running SeaweedFS master on OSv and volume on Linux host
This is more practical setup that requires master on OSv to be able to
communicate with the volume component running on the host. The easiest way
to achieve it is to setup a TAP device:
```
./scripts/create_tap_device.sh natted qemu_tap0 172.18.0.1 #You can pick different address but then update all IPs below
```

This experiment is based on https://github.com/chrislusf/seaweedfs/wiki/Benchmarks#unscientific-single-machine-benchmarking

Steps:
- build OSv image with separate ZFS disk and create necessary directories
```
./scripts/build -j4 image=seaweedfs,zfs,httpserver-monitoring-api fs=rofs --create-zfs-disk

./scripts/zfs-image-on-host.sh mount build/last/zfs_disk.img
sudo mkdir -p /zfs/seaweedfs/master /zfs/seaweedfs/logs
./scripts/zfs-image-on-host.sh unmount
```

- run master on OSv
```
./scripts/run.py -n -t qemu_tap0 --execute='--ip=eth0,172.18.0.2,255.255.255.252 --defaultgw=172.18.0.1 --nameserver=172.18.0.1 /weed -logdir /data/seaweedfs/logs master -mdir=/data/seaweedfs/master' --second-disk-image build/last/zfs_disk.img

OSv v0.56.0-155-gee7f8b35
eth0: 172.18.0.2
Booted up in 245.21 ms
Cmdline: /weed -logdir /data/seaweedfs/logs master -mdir=/data/seaweedfs/master
Rest API server running on port 8000
I0726 18:01:55     2 file_util.go:23] Folder /data/seaweedfs/master Permission: -rwxr-xr-x
I0726 18:01:55     2 master.go:232] current: 172.18.0.2:9333 peers:
I0726 18:01:55     2 master_server.go:122] Volume Size Limit is 30000 MB
I0726 18:01:55     2 master.go:143] Start Seaweed Master 30GB 2.96  at 172.18.0.2:9333
I0726 18:01:55     2 raft_server.go:80] Starting RaftServer with 172.18.0.2:9333
I0726 18:01:55     2 raft_server.go:129] current cluster leader: 
I0726 18:02:13     2 master.go:176] Start Seaweed Master 30GB 2.96  grpc server at 172.18.0.2:19333
I0726 18:02:15     2 masterclient.go:80] No existing leader found!
I0726 18:02:15     2 raft_server.go:146] Initializing new cluster
I0726 18:02:15     2 master_server.go:165] leader change event:  => 172.18.0.2:9333
I0726 18:02:15     2 master_server.go:168] [ 172.18.0.2:9333 ] 172.18.0.2:9333 becomes leader.
I0726 18:02:18     2 master_grpc_server.go:278] + client master@172.18.0.2:9333
```

In a separate terminal run volume component on the host. For that you also need to create a
data directory, in this example 4:
```
mkdir ./4
./apps/seaweedfs/upstream/seaweedfs/weed/weed volume -mserver="172.18.0.2:9333" -dir=./4 -port=8084

I0726 14:05:51 55733 file_util.go:23] Folder ./4 Permission: -rwxr-xr-x
I0726 14:05:51 55733 disk_location.go:182] Store started on dir: ./4 with 0 volumes max 8
I0726 14:05:51 55733 disk_location.go:185] Store started on dir: ./4 with 0 ec shards
I0726 14:05:51 55733 volume_grpc_client_to_master.go:50] Volume server start with seed master nodes: [172.18.0.2:9333]
I0726 14:05:51 55733 volume.go:364] Start Seaweed volume server 30GB 2.96  at 192.168.1.81:8084
I0726 14:05:51 55733 volume_grpc_client_to_master.go:107] Heartbeat to: 172.18.0.2:9333
```

After that you should see info on the master running on OSv that the volume component joined
the cluster:
```
I0726 18:05:51     2 node.go:222] topo adds child DefaultDataCenter
I0726 18:05:51     2 node.go:222] topo:DefaultDataCenter adds child DefaultRack
I0726 18:05:51     2 node.go:222] topo:DefaultDataCenter:DefaultRack adds child 192.168.1.81:8084
I0726 18:05:51     2 node.go:222] topo:DefaultDataCenter:DefaultRack:192.168.1.81:8084 adds child 
I0726 18:05:51     2 master_grpc_server.go:72] added volume server 0: 192.168.1.81:8084
```

Now in a 3rd terminal we will run the benchmark utility pointing to the master on OSv like
so:
```
./apps/seaweedfs/upstream/seaweedfs/weed/weed benchmark -master="172.18.0.2:9333"
```

On the master on OSv you should see this:
```
I0726 18:07:17     2 master_grpc_server.go:278] + client client@
I0726 18:07:18     2 volume_layout.go:391] Volume 1 becomes writable
I0726 18:07:18     2 volume_growth.go:235] Created Volume 1 on topo:DefaultDataCenter:DefaultRack:192.168.1.81:8084
I0726 18:07:18     2 volume_layout.go:391] Volume 2 becomes writable
I0726 18:07:18     2 volume_growth.go:235] Created Volume 2 on topo:DefaultDataCenter:DefaultRack:192.168.1.81:8084
I0726 18:07:18     2 volume_layout.go:391] Volume 3 becomes writable
I0726 18:07:18     2 volume_growth.go:235] Created Volume 3 on topo:DefaultDataCenter:DefaultRack:192.168.1.81:8084
I0726 18:07:18     2 volume_layout.go:391] Volume 4 becomes writable
I0726 18:07:18     2 volume_growth.go:235] Created Volume 4 on topo:DefaultDataCenter:DefaultRack:192.168.1.81:8084
I0726 18:07:18     2 volume_layout.go:391] Volume 5 becomes writable
I0726 18:07:18     2 volume_growth.go:235] Created Volume 5 on topo:DefaultDataCenter:DefaultRack:192.168.1.81:8084
I0726 18:07:18     2 volume_layout.go:391] Volume 6 becomes writable
I0726 18:07:18     2 volume_growth.go:235] Created Volume 6 on topo:DefaultDataCenter:DefaultRack:192.168.1.81:8084
I0726 18:07:18     2 volume_layout.go:391] Volume 7 becomes writable
I0726 18:07:18     2 volume_growth.go:235] Created Volume 7 on topo:DefaultDataCenter:DefaultRack:192.168.1.81:8084
```

If everything goes fine, in the 3rd terminal after benchark finishes you should see some statistics:
```
This is SeaweedFS version 30GB 2.96  linux amd64

------------ Writing Benchmark ----------
Completed 10970 of 1048576 requests, 1.0% 10969.7/s 11.0MB/s
Completed 25308 of 1048576 requests, 2.4% 14325.4/s 14.4MB/s
Completed 39669 of 1048576 requests, 3.8% 14374.0/s 14.5MB/s
Completed 54169 of 1048576 requests, 5.2% 14500.2/s 14.6MB/s
Completed 68653 of 1048576 requests, 6.5% 14484.1/s 14.6MB/s
...
Completed 993113 of 1048576 requests, 94.7% 12698.6/s 12.8MB/s
Completed 1005838 of 1048576 requests, 95.9% 12725.0/s 12.8MB/s
Completed 1018389 of 1048576 requests, 97.1% 12550.9/s 12.6MB/s
Completed 1030852 of 1048576 requests, 98.3% 12460.6/s 12.5MB/s
Completed 1043568 of 1048576 requests, 99.5% 12716.6/s 12.8MB/s

Concurrency Level:      16
Time taken for tests:   79.380 seconds
Complete requests:      1048576
Failed requests:        0
Total transferred:      1106771143 bytes
Requests per second:    13209.56 [#/sec]
Transfer rate:          13615.90 [Kbytes/sec]

Connection Times (ms)
              min      avg        max      std
Total:        0.3      1.1       213.5      1.0

Percentage of the requests served within a certain time (ms)
   50%      1.1 ms
   66%      1.2 ms
   80%      1.3 ms
   90%      1.5 ms
   95%      1.9 ms
   98%      2.4 ms
   99%      2.9 ms
  100%    213.5 ms

------------ Randomly Reading Benchmark ----------
Completed 27403 of 1048576 requests, 2.6% 27403.0/s 27.6MB/s
Completed 58191 of 1048576 requests, 5.5% 30787.9/s 31.0MB/s
Completed 88762 of 1048576 requests, 8.5% 30561.9/s 30.8MB/s
Completed 119771 of 1048576 requests, 11.4% 31017.9/s 31.2MB/s
Completed 151160 of 1048576 requests, 14.4% 31389.3/s 31.6MB/s
...
Completed 913774 of 1048576 requests, 87.1% 18601.1/s 18.7MB/s
Completed 945548 of 1048576 requests, 90.2% 31774.6/s 32.0MB/s
Completed 976864 of 1048576 requests, 93.2% 31315.7/s 31.5MB/s
Completed 1000917 of 1048576 requests, 95.5% 24053.1/s 24.2MB/s
Completed 1025362 of 1048576 requests, 97.8% 24445.9/s 24.6MB/s

Concurrency Level:      16
Time taken for tests:   35.736 seconds
Complete requests:      1048576
Failed requests:        0
Total transferred:      1106780708 bytes
Requests per second:    29341.98 [#/sec]
Transfer rate:          30244.83 [Kbytes/sec]

Connection Times (ms)
              min      avg        max      std
Total:        0.1      0.5       23.0      0.4

Percentage of the requests served within a certain time (ms)
   50%      0.4 ms
   66%      0.5 ms
   75%      0.6 ms
   90%      0.8 ms
   95%      1.1 ms
   98%      1.6 ms
   99%      2.2 ms
  100%     23.0 ms
```

Comparing to the case when both master and volume run on the same native Linux host (not guest)
the write transfer rate is ~75% and read transfer rate is pretty much the same.

Also, if you want to inspect the ZFS filesystem you can mount it on the Linux host like
so (you need to stop OSv before it as the image file is locked):
```
./scripts/zfs-image-on-host.sh mount build/last/zfs_disk.img
Connected device /dev/nbd0 to the image build/last/zfs_disk.img
Imported pool osv
Mounted osv/zfs at /zfs

[wkozaczuk@fedora-mbpro osv]$ find /zfs/
/zfs/
/zfs/seaweedfs
/zfs/seaweedfs/logs
/zfs/seaweedfs/logs/weed.osv.osv.log.WARNING.20220726-181118.2
/zfs/seaweedfs/logs/weed.osv.osv.log.INFO.20220726-180155.2
/zfs/seaweedfs/logs/weed.WARNING
/zfs/seaweedfs/logs/weed.INFO
/zfs/seaweedfs/master
/zfs/seaweedfs/master/snapshot
find: ‘/zfs/seaweedfs/master/snapshot’: Permission denied
/zfs/seaweedfs/master/log
/zfs/seaweedfs/master/conf

./scripts/zfs-image-on-host.sh unmount
```

For more details about mounting and manipulating OSv ZFS image on host
read this Wiki - https://github.com/cloudius-systems/osv/wiki/Filesystems#creating-and-manipulating-zfs-disks-on-host.

Alternatevily, you can use the OSv `cli` app to connect to the running OSv before you shutdown master
(this assumes you build the cli module before):
```
cd modules/cli
LD_LIBRARY_PATH=../lua/upstream/lua5.3 ./cli -a http://172.18.0.2:8000

/# cd /data/seaweedfs/master
/data/seaweedfs/master# cat conf
{"commitIndex":1,"peers":[]}
```

### Running SeaweedFS volume on OSv and master on Linux host
This is similar to above but with the reversed setup where master runs on Linux
host and volume on OSv.

The test volume requires as least 1.2GB so we will build larger 5GB ZFS disk:
```
./scripts/build -j4 image=seaweedfs,zfs,httpserver-monitoring-api fs=rofs --create-zfs-disk fs_size_mb=5000

./scripts/zfs-image-on-host.sh mount build/last/zfs_disk.img
sudo mkdir -p /zfs/seaweedfs/logs /zfs/5
./scripts/zfs-image-on-host.sh unmount
```

- run master on Linux host
```
./apps/seaweedfs/upstream/seaweedfs/weed/weed master

I0726 14:29:17 57283 file_util.go:23] Folder /tmp Permission: -rwxrwxrwx
I0726 14:29:17 57283 master.go:232] current: 192.168.1.81:9333 peers:
I0726 14:29:17 57283 master_server.go:122] Volume Size Limit is 30000 MB
I0726 14:29:17 57283 master.go:143] Start Seaweed Master 30GB 2.96  at 192.168.1.81:9333
I0726 14:29:17 57283 raft_server.go:80] Starting RaftServer with 192.168.1.81:9333
I0726 14:29:17 57283 raft_server.go:129] current cluster leader: 
I0726 14:29:35 57283 master.go:176] Start Seaweed Master 30GB 2.96  grpc server at 192.168.1.81:19333
I0726 14:29:36 57283 masterclient.go:80] No existing leader found!
I0726 14:29:36 57283 raft_server.go:146] Initializing new cluster
I0726 14:29:36 57283 master_server.go:165] leader change event:  => 192.168.1.81:9333
I0726 14:29:36 57283 master_server.go:168] [ 192.168.1.81:9333 ] 192.168.1.81:9333 becomes leader.
I0726 14:29:40 57283 master_grpc_server.go:278] + client master@192.168.1.81:9333
```

In a separate terminal run volume component on OSv this time. Please not in this case
we have already created the data directory 5 above:
```
./scripts/run.py -n -t qemu_tap0 --execute='--ip=eth0,172.18.0.2,255.255.255.252 --defaultgw=172.18.0.1 --nameserver=172.18.0.1 /weed -logdir /data/seaweedfs/logs volume -mserver=192.168.1.81:9333 -dir=/data/5 -port=8085' --second-disk-image build/last/zfs_disk.img

OSv v0.56.0-155-gee7f8b35
eth0: 172.18.0.2
Booted up in 236.74 ms
Cmdline: /weed -logdir /data/seaweedfs/logs volume -mserver=192.168.1.81:9333 -dir=/data/5 -port=8085
Rest API server running on port 8000
I0726 18:36:12     2 file_util.go:23] Folder /data/5 Permission: -rwxr-xr-x
I0726 18:36:12     2 disk_location.go:182] Store started on dir: /data/5 with 0 volumes max 8
I0726 18:36:12     2 disk_location.go:185] Store started on dir: /data/5 with 0 ec shards
I0726 18:36:12     2 volume_grpc_client_to_master.go:50] Volume server start with seed master nodes: [192.168.1.81:9333]
I0726 18:36:12     2 volume.go:364] Start Seaweed volume server 30GB 2.96  at 172.18.0.2:8085
I0726 18:36:12     2 volume_grpc_client_to_master.go:107] Heartbeat to: 192.168.1.81:9333
```

After that you should see info on the master running on Linux that the volume component joined
the cluster:
```
I0726 14:36:12 57283 node.go:222] topo adds child DefaultDataCenter
I0726 14:36:12 57283 node.go:222] topo:DefaultDataCenter adds child DefaultRack
I0726 14:36:12 57283 node.go:222] topo:DefaultDataCenter:DefaultRack adds child 172.18.0.2:8085
I0726 14:36:12 57283 node.go:222] topo:DefaultDataCenter:DefaultRack:172.18.0.2:8085 adds child 
I0726 14:36:12 57283 master_grpc_server.go:72] added volume server 0: 172.18.0.2:8085
```

Just like in the 1st example, in a 3rd terminal we will run the benchmark utility pointing to the master
but this time running on host:
```
./apps/seaweedfs/upstream/seaweedfs/weed/weed benchmark -master="192.168.1.81:9333"
```

On the master on Linux host you should see this:
```
I0726 14:40:03 57283 master_grpc_server.go:278] + client client@
I0726 14:40:03 57283 volume_layout.go:391] Volume 1 becomes writable
I0726 14:40:03 57283 volume_growth.go:235] Created Volume 1 on topo:DefaultDataCenter:DefaultRack:172.18.0.2:8085
I0726 14:40:03 57283 volume_layout.go:391] Volume 2 becomes writable
I0726 14:40:03 57283 volume_growth.go:235] Created Volume 2 on topo:DefaultDataCenter:DefaultRack:172.18.0.2:8085
I0726 14:40:03 57283 volume_layout.go:391] Volume 3 becomes writable
I0726 14:40:03 57283 volume_growth.go:235] Created Volume 3 on topo:DefaultDataCenter:DefaultRack:172.18.0.2:8085
I0726 14:40:03 57283 volume_layout.go:391] Volume 4 becomes writable
I0726 14:40:03 57283 volume_growth.go:235] Created Volume 4 on topo:DefaultDataCenter:DefaultRack:172.18.0.2:8085
I0726 14:40:03 57283 volume_layout.go:391] Volume 5 becomes writable
I0726 14:40:03 57283 volume_growth.go:235] Created Volume 5 on topo:DefaultDataCenter:DefaultRack:172.18.0.2:8085
I0726 14:40:03 57283 volume_layout.go:391] Volume 6 becomes writable
I0726 14:40:03 57283 volume_growth.go:235] Created Volume 6 on topo:DefaultDataCenter:DefaultRack:172.18.0.2:8085
I0726 14:40:03 57283 volume_layout.go:391] Volume 7 becomes writable
I0726 14:40:03 57283 volume_growth.go:235] Created Volume 7 on topo:DefaultDataCenter:DefaultRack:172.18.0.2:8085
```

Again, if everything goes fine, on the 3 terminal after benchark finishes you should see some statistics:
```
This is SeaweedFS version 30GB 2.96  linux amd64

------------ Writing Benchmark ----------
Completed 7954 of 1048576 requests, 0.8% 7953.9/s 8.0MB/s
Completed 19177 of 1048576 requests, 1.8% 11223.0/s 11.3MB/s
Completed 31011 of 1048576 requests, 3.0% 11834.3/s 11.9MB/s
Completed 43202 of 1048576 requests, 4.1% 12190.2/s 12.3MB/s
Completed 54793 of 1048576 requests, 5.2% 11591.8/s 11.7MB/s
...
Completed 997683 of 1048576 requests, 95.1% 9933.1/s 10.0MB/s
Completed 1007292 of 1048576 requests, 96.1% 9615.8/s 9.7MB/s
Completed 1017770 of 1048576 requests, 97.1% 10469.1/s 10.5MB/s
Completed 1027927 of 1048576 requests, 98.0% 10165.7/s 10.2MB/s
Completed 1038343 of 1048576 requests, 99.0% 10416.0/s 10.5MB/s

Concurrency Level:      16
Time taken for tests:   99.990 seconds
Complete requests:      1048576
Failed requests:        0
Total transferred:      1106817524 bytes
Requests per second:    10486.83 [#/sec]
Transfer rate:          10809.87 [Kbytes/sec]

Connection Times (ms)
              min      avg        max      std
Total:        0.3      1.5       263.7      1.6

Percentage of the requests served within a certain time (ms)
   50%      1.2 ms
   66%      1.5 ms
   75%      1.7 ms
   80%      1.9 ms
   90%      2.4 ms
   95%      2.9 ms
   98%      3.7 ms
   99%      4.7 ms
  100%    263.7 ms

------------ Randomly Reading Benchmark ----------
Completed 13503 of 1048576 requests, 1.3% 13502.5/s 13.6MB/s
Completed 31990 of 1048576 requests, 3.1% 18486.5/s 18.6MB/s
Completed 48568 of 1048576 requests, 4.6% 16578.0/s 16.7MB/s
Completed 64199 of 1048576 requests, 6.1% 15631.3/s 15.7MB/s
Completed 81538 of 1048576 requests, 7.8% 17339.3/s 17.5MB/s
...
Completed 988100 of 1048576 requests, 94.2% 10889.1/s 11.0MB/s
Completed 1003554 of 1048576 requests, 95.7% 15454.2/s 15.6MB/s
Completed 1018227 of 1048576 requests, 97.1% 14672.9/s 14.8MB/s
Completed 1032749 of 1048576 requests, 98.5% 14519.3/s 14.6MB/s
Completed 1040848 of 1048576 requests, 99.3% 8099.8/s 8.2MB/s

Concurrency Level:      16
Time taken for tests:   71.460 seconds
Complete requests:      1048576
Failed requests:        0
Total transferred:      1106811091 bytes
Requests per second:    14673.55 [#/sec]
Transfer rate:          15125.47 [Kbytes/sec]

Connection Times (ms)
              min      avg        max      std
Total:        0.1      1.0       78.5      1.9

Percentage of the requests served within a certain time (ms)
   50%      0.7 ms
   66%      0.9 ms
   75%      1.1 ms
   80%      1.3 ms
   90%      1.8 ms
   95%      2.7 ms
   98%      4.4 ms
   99%      6.4 ms
  100%     78.5 ms
```

Unfortunately, OSv is not known for good disk IO performance and this setup involves heavy disk IO
on OSv side plus again we are comparing against Linux native host (no overhead of virtualization),
so the results are worse than above - the write transfer rate is ~60% and similarly the read transfer
rate is also ~60%. It would be nice to compare against Linux guest.

Again if you want to inspect the disk after you stop OSv:
```
./scripts/zfs-image-on-host.sh mount build/last/zfs_disk.img

find /zfs/
/zfs/
/zfs/5
/zfs/5/benchmark_7.dat
/zfs/5/benchmark_2.idx
/zfs/5/benchmark_5.idx
/zfs/5/benchmark_7.vif
/zfs/5/benchmark_4.idx
/zfs/5/benchmark_3.idx
/zfs/5/benchmark_6.dat
/zfs/5/benchmark_1.dat
/zfs/5/benchmark_6.vif
/zfs/5/benchmark_1.vif
/zfs/5/benchmark_2.vif
/zfs/5/benchmark_5.vif
/zfs/5/benchmark_7.idx
/zfs/5/benchmark_2.dat
/zfs/5/benchmark_5.dat
/zfs/5/benchmark_4.vif
/zfs/5/benchmark_3.vif
/zfs/5/benchmark_4.dat
/zfs/5/benchmark_3.dat
/zfs/5/benchmark_6.idx
/zfs/5/benchmark_1.idx
/zfs/seaweedfs
/zfs/seaweedfs/logs
/zfs/seaweedfs/logs/weed.INFO
/zfs/seaweedfs/logs/weed.osv.osv.log.INFO.20220726-183612.2

./scripts/zfs-image-on-host.sh umount
```
