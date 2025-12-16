#include <atomic>
#include <boost/algorithm/string.hpp>
#include <cassert>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <osv/ucache.hh>

using namespace std;
using namespace ucache;

atomic<bool> keepGoing;

int stick_this_thread_to_core(int core_id) {
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core_id, &cpuset);

  pthread_t current_thread = pthread_self();
  return pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
}

double gettime() {
  struct timeval now_tv;
  gettimeofday(&now_tv, NULL);
  return ((double)now_tv.tv_sec) + ((double)now_tv.tv_usec) / 1000000.0;
}

#define WORK_READ 1
#define WORK_WRITE 2

void do_io(VMA* vma, std::vector<std::vector<Buffer*>::iterator> bufs, int mode){
  for(const auto i: bufs){
    Buffer* b = *i;
    BufferSnapshot* bs = new BufferSnapshot(vma->nbPages);
    b->updateSnapshot(bs);
    b->snap = bs;
    if(mode == WORK_READ){
      b->snap->reqs = vma->file->aread(b->baseVirt, (u64)b->baseVirt-(u64)vma->start, b->vma->pageSize, false);
    }else{
      b->snap->reqs = vma->file->awrite(b->baseVirt, (u64)b->baseVirt-(u64)vma->start, b->vma->pageSize, false);
    }
  }
  for(const auto i: bufs){
    Buffer* b = *i;
    vma->file->poll(b->snap->reqs);
    delete b->snap->reqs;
    delete b->snap;
  }
}


void print_usage(){
    cerr << "workload(r/w) ioSize(B) queueDepth ramptime(s) recordtime(s)" << endl;
}

int main(int argc, char **argv) {
  if (argc != 6) {
    print_usage();
    return 1;
  }

  if(strcmp(argv[1], "r") != 0 && strcmp(argv[1], "w") != 0){
    print_usage();
    return 1;
  }
  int mode = WORK_WRITE;
  if(strcmp(argv[1], "r") == 0){
    mode = WORK_READ;
  }
 
  u64 ioSize = atoi(argv[2]);
  u64 queueDepth = atoi(argv[3]);
  u64 rampTime = atoi(argv[4]);
  u64 recordTime = atoi(argv[5]);
  unsigned threads = sched::cpus.size();
  
  u64 memSize = 100ul * 1024 * 1024 * 1024;
  u64 nbBuffers = memSize / ioSize;
  createCache(150ul * 1024 * 1024 * 1024, 64);
  VMA* vma = uCacheManager->mmap("/nvme/cache", memSize, ioSize);
  
  struct atomic_u64_padded{
    std::atomic<uint64_t> val;
    char padding[64 - sizeof(std::atomic<uint64_t>)];
  };

  std::vector<atomic_u64_padded> counts(threads);
  std::vector<std::vector<Buffer*>*> buffers;
  for(int i = 0; i < nbBuffers; i++){
    uCacheManager->handleFault(vma, vma->buffers[i], true);
  }

  keepGoing.store(true);
  atomic<uint64_t> seqScanPos(0);

  vector<thread> t;
  for (unsigned i = 0; i < threads; i++) {
    t.emplace_back([&, i]() {
      stick_this_thread_to_core(i);
      atomic<uint64_t> &count = counts[i].val;
      count = 0;
      while (keepGoing.load()) {
        uint64_t nbBuffersBlock = 512ul*1024*1024 / ioSize;
	      uint64_t pos = (seqScanPos += nbBuffersBlock) % (nbBuffers);

	      for (uint64_t j=0; j<nbBuffersBlock; j+=queueDepth) {
          std::vector<std::vector<Buffer*>::iterator> l(queueDepth);
          std::iota(l.begin(), l.end(), vma->buffers.begin()+pos+j);
          do_io(vma, l, mode);
          count+=queueDepth;
	      }
      }
    });
  }

  const char* modeStr = mode == WORK_READ ? "read" : "write";
  cout << "system,workload,ioSize,threads,time,queueDepth,throughput" << endl;
  double start = gettime();
  while (true) {
    sleep(1);
    double ti = gettime() - start;
    uint64_t workCount = 0;
    for (auto &x : counts)
      workCount += x.val.exchange(0);
    if(ti < rampTime){
      continue;
    }
    u64 size = workCount * ioSize;
    cout << 
      "ufs," << modeStr << "," << ioSize << "," << threads << "," << ti  << "," << queueDepth << "," << size / (1024 * 1024+0.0) << "," << workCount << endl;
    if(ti >= rampTime+recordTime){
      keepGoing.store(false);
      break;
    }
  }
  for(auto& t: t)
    t.join();

  close(vma->file->fd);

  return 0;
}
