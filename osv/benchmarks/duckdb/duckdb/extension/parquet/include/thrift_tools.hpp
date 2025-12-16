//===----------------------------------------------------------------------===//
//                         DuckDB
//
// thrift_tools.hpp
//
//
//===----------------------------------------------------------------------===/

#pragma once

#include <list>
#include "thrift/protocol/TCompactProtocol.h"
#include "thrift/transport/TBufferTransports.h"
#include <osv/ucache.hh>

#include "duckdb.hpp"
#ifndef DUCKDB_AMALGAMATION
#include "duckdb/common/file_system.hpp"
#include "duckdb/common/allocator.hpp"
#endif

namespace duckdb {

// A ReadHead for prefetching data in a specific range
struct ReadHead {
	ReadHead(idx_t location, uint64_t size) : location(location), size(size){};
	// Hint info
	idx_t location;
	uint64_t size;

	bool already_used = false;

	idx_t GetEnd() const {
		return size + location;
	}
};

// Comparator for ReadHeads that are either overlapping, adjacent, or within ALLOW_GAP bytes from each other
struct ReadHeadComparator {
	static constexpr uint64_t ALLOW_GAP = 1 << 14; // 16 KiB
	bool operator()(const ReadHead *a, const ReadHead *b) const {
		auto a_start = a->location;
		auto a_end = a->location + a->size;
		auto b_start = b->location;

		if (a_end <= NumericLimits<idx_t>::Maximum() - ALLOW_GAP) {
			a_end += ALLOW_GAP;
		}

		return a_start < b_start && a_end < b_start;
	}
};

// Two-step read ahead buffer
// 1: register all ranges that will be read, merging ranges that are consecutive
// 2: prefetch all registered ranges
struct ReadAheadBuffer {
	ReadAheadBuffer(ucache::VMA* vma_p): vma(vma_p)
	{
		read_heads.reserve(128);
	}

	// The list of read heads
	std::vector<ReadHead> read_heads;
	// Set for merging consecutive ranges
	std::set<ReadHead *, ReadHeadComparator> merge_set;

	ucache::VMA* vma;

	idx_t total_size = 0;

	// Add a read head to the prefetching list
	void AddReadHead(idx_t pos, uint64_t len, bool merge_buffers = true) {
		// Attempt to merge with existing
		if (merge_buffers) {
			ReadHead new_read_head {pos, len};
			auto lookup_set = merge_set.find(&new_read_head);
			if (lookup_set != merge_set.end()) {
				auto existing_head = *lookup_set;
				auto new_start = MinValue<idx_t>(existing_head->location, new_read_head.location);
				auto new_length = MaxValue<idx_t>(existing_head->GetEnd(), new_read_head.GetEnd()) - new_start;
				existing_head->location = new_start;
				existing_head->size = new_length;
				return;
			}
		}

		read_heads.emplace(read_heads.begin(), ReadHead(pos, len));
		total_size += len;
		auto &read_head = read_heads.front();

		if (merge_buffers) {
			merge_set.insert(&read_head);
		}

		if (read_head.GetEnd() > vma->file->size) {
			throw std::runtime_error("Prefetch registered for bytes outside file: " + std::string(vma->file->name) +
			                         ", attempted range: [" + std::to_string(pos) + ", " +
			                         std::to_string(read_head.GetEnd()) +
			                         "), file size: " + std::to_string(vma->file->size));
		}
	}

	// Returns the relevant read head
	ReadHead *GetReadHead(idx_t pos) {
		for(int i=0; i<read_heads.size(); i++){
			ReadHead &read_head = read_heads[i];
			if (pos >= read_head.location && pos < read_head.GetEnd()) {
				return &read_head;
			}
		}
		return nullptr;
	} 
	void printHeads(){
		for(int i=0; i<read_heads.size(); i++){
			ReadHead &rh = read_heads[i];
			printf("%u (%lu - %lu) / ", i, rh.location, rh.GetEnd());
		}
		printf("\n");
	}
};

void parquet_prefetch(ucache::VMA* vma, void* addr, ucache::PrefetchList pl);

class ThriftFileTransport : public duckdb_apache::thrift::transport::TVirtualTransport<ThriftFileTransport> {
public:
	static constexpr uint64_t PREFETCH_FALLBACK_BUFFERSIZE = 1000000;
	ThriftFileTransport(ucache::VMA* vma_p, bool prefetch_mode_p)
	    : vma(vma_p), location(0), size(vma->file->size), prefetch_mode(prefetch_mode_p), ra_buffer(ReadAheadBuffer(vma_p))
			{
			}

	uint32_t read(uint8_t *buf, uint32_t len) {
		memcpy(buf, reinterpret_cast<void*>(reinterpret_cast<uintptr_t>(vma->start)+location), len);
		location += len;
		return len;
	}

	// Prefetch a single buffer
	void Prefetch(idx_t pos, uint64_t len) {
		RegisterPrefetch(pos, len, false);
		FinalizeRegistration();
		PrefetchRegistered();
	}

	// Register a buffer for prefixing
	void RegisterPrefetch(idx_t pos, uint64_t len, bool can_merge = true) {
		ra_buffer.AddReadHead(pos, len, can_merge);
	}

	// Prevents any further merges, should be called before PrefetchRegistered
	void FinalizeRegistration() {
		ra_buffer.merge_set.clear();
	}

	// Prefetch all previously registered ranges
	void PrefetchRegistered() {
	}

	void ClearPrefetch() {
		ra_buffer.read_heads.clear();
		ra_buffer.merge_set.clear();
	}

	void Skip(idx_t skip_count) {
		location += skip_count;
	}

	bool HasPrefetch() const {
		return !ra_buffer.read_heads.empty() || !ra_buffer.merge_set.empty();
	}

	void SetLocation(idx_t location_p) {
		location = location_p;
	}

	idx_t GetLocation() {
		return location;
	}
	idx_t GetSize() {
		return size;
	}

	// Multi-buffer prefetch
	ReadAheadBuffer ra_buffer;
private:
	ucache::VMA* vma;
	idx_t location;
	idx_t size;

	// Whether the prefetch mode is enabled. In this mode the DirectIO flag of the handle will be set and the parquet
	// reader will manage the read buffering.
	bool prefetch_mode;
};

extern ThriftFileTransport* cores_transports[64];
} // namespace duckdb
