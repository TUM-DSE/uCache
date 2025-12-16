src=../..
CXXFLAGS = -g -DNDEBUG -DCONF_debug_memory=0 -std=c++20 -mavx -mavx2 -shared -fPIC -O3
INCLUDES = -I$(src)/include -I$(src) -I$(src)/arch/common -I$(src)/arch/x64 -I$(src)/build/release/gen/include

mmapbench/mmapbench: mmapbench.cpp
	g++ $(CXXFLAGS) $(INCLUDES) mmapbench.cpp -o mmapbench

module: mmapbench/mmapbench

clean:
	rm -f mmapbench
