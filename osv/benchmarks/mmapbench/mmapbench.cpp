// sudo apt install libtbb-dev
// g++ -O3 -g mmapbench.cpp -o mmapbench -ltbb -pthread

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

int main(int argc, char **argv) {
  /*createCache(1000*4096, 32);
  VMA* vma = uCacheManager->mmap("/nvme/cache", 2ul*1024*1024, 4096);
  char* p = (char*)vma->start;
  char a;
  for(u64 i= 0; i< (2ul*1024*1024)/4096; i++){
    a += p[i*4096];
  }
  printf("a: %c\n", a);
  return 0;*/
  if (argc != 6) {
    cerr << "virtSize(in GiB) physSize(in GiB) workload pageSize timetorun(in sec)" << endl;
    return 1;
  }

  u64 virtSize = atoi(argv[1]) * 1024ul * 1024 * 1024;
  u64 physSize = atoi(argv[2]) * 1024ul * 1024 * 1024;
  int mode = atoi(argv[3]);
  u64 page_size = atoi(argv[4]);
  int maxTime = atoi(argv[5]);

  unsigned threads = sched::cpus.size();
  u64 batch = NVME_IO_QUEUE_SIZE / (page_size/4096);
  createCache(physSize, batch);
  VMA* vma = uCacheManager->mmap("/nvme/cache", virtSize, page_size);
  char* p = (char*)vma->start;
  assert_crash(virtSize <= vma->size);
  
  struct atomic_u64_padded{
    std::atomic<uint64_t> val;
    char padding[64 - sizeof(std::atomic<uint64_t>)];
  };

  std::vector<atomic_u64_padded> counts(threads);
  std::vector<atomic_u64_padded> sums(threads);
  atomic<uint64_t> seqScanPos(0);

  keepGoing.store(true);

  vector<thread> t;
  for (unsigned i = 0; i < threads; i++) {
    t.emplace_back([&, i]() {
      stick_this_thread_to_core(i);
      atomic<uint64_t> &count = counts[i].val;
      atomic<uint64_t> &sum = sums[i].val;
      count = 0;
      sum = 0;
      switch(mode){
        case 0: { // seqread
	        while (keepGoing.load()) {
	          uint64_t scanBlock = 512*1024*1024;
	          uint64_t pos = (seqScanPos += scanBlock) % (virtSize-page_size);
            if(pos >= physSize){
              reset_stats(i); 
            }

	          for (uint64_t j=0; j<scanBlock; j+=4096) {
	            sum += p[pos + j];
	            count++;
	          }
	        }
          break;
        }
        case 1: { // rndread
          {
          std::random_device rd;
          std::mt19937 gen(rd());
          std::uniform_int_distribution<uint64_t> rnd(0, (virtSize/page_size)-1);
            while (keepGoing.load()) {
              u64 pos = rnd(gen)*page_size;
              sum += p[pos];
              count++;
            }
          }
          break;
        }
        case 2: { // seqread
	        while (keepGoing.load()) {
	          uint64_t scanBlock = 512*1024*1024;
	          uint64_t pos = (seqScanPos += scanBlock) % (virtSize-page_size);
            if(pos >= physSize){
              reset_stats(i); 
            }

	          for (uint64_t j=0; j<scanBlock; j+=4096) {
	            p[pos + j] = count;
	            count++;
	          }
	        }
          break;
        }
        default:
          printf("mode not supported\n");
      }
    });
  }

  cout << "system,workload,pageSize,threads,time,throughput" << endl;
  double start = gettime();
  while (true) {
    sleep(1);
    uint64_t workCount = 0;
    for (auto &x : counts)
      workCount += x.val.exchange(0);
    double ti = gettime() - start;
    cout << 
      "ucache," << mode << "," << page_size << "," << threads << "," << ti  <<  "," << workCount << endl;
    if(ti >= maxTime){
      keepGoing.store(false);
      break;
    }
  }
  for(auto& t: t)
    t.join();

  ucache::print_stats();

  vma->file->close();

  return 0;
}
