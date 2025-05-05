#include <vector>
#include <cstdint>
#include <iostream>
#include <cmath>
#include "memory.h"

#ifdef ENABLE_DEBUG
#define DEBUG(x) x
#else
#define DEBUG(x) 
#endif

using namespace std;

// Check if hit in the cache
bool Cache::isHit(uint32_t address, uint32_t &loc) {
    int idx = getIndex(address);
    int tag = getTag(address);

    for (int w=0; w<assoc; w++) {
        if (line[idx*assoc+w].valid && line[idx*assoc+w].tag == tag) {
            loc = idx*assoc+w;
            updateReplacementBits(idx, w);
            return true;
        }
    }
    return false;
}

// Update replacement bits after access
void Cache::updateReplacementBits(int idx, int way) {
    uint8_t curRepl = line[idx*assoc+way].replBits;
    for (int w=0; w<assoc; w++) {
        if (line[idx*assoc+w].valid && line[idx*assoc+w].replBits > curRepl)
            line[idx*assoc+w].replBits--;
    }
    line[idx*assoc+way].replBits = assoc-1;
}

// Read a word from this cache
bool Cache::read(uint32_t address, uint32_t &read_data) {
    uint32_t loc = 0;
    if (missCountdown) {
        DEBUG(cout << name + " Cache (read miss) at address " << std::hex << address << std::dec << ": " << missCountdown << " cycles remaining to be serviced\n");
        missCountdown--;
        return false;
    }
    // Once miss penalty is completely paid, isHit should return true
    if (!isHit(address, loc)) {
        missCountdown = missPenalty-1;
        return false;
    }
    read_data = line[loc].data[getOffset(address)/4]; 
    DEBUG(cout << name + " Cache (read hit): " << read_data << "<-[" << std::hex << address << std::dec << "]\n");
    return true;
}

// Write a word to this cache
bool Cache::write(uint32_t address, uint32_t write_data) {
    uint32_t loc = 0;
    if (missCountdown) {
        DEBUG(cout << name + " Cache (write miss) at address " << std::hex << address << std::dec << ": " << missCountdown << " cycles remaining to be serviced\n");
        missCountdown--;
        return false;
    }
    // Once miss penalty is completely paid, isHit should return true
    if (!isHit(address, loc)) {
        missCountdown = missPenalty-1;
        return false;
    }
    line[loc].data[getOffset(address)/4] = write_data;
    line[loc].dirty = true; 
    DEBUG(cout << name + " Cache (write hit): [" << std::hex << address << std::dec << "]<-" << write_data << "\n");
    return true;
}

// Call this only if you know that a valid line with matching tag exists at that address 
CacheLine Cache::readLine(uint32_t address) {
    int idx = getIndex(address);
    int tag = getTag(address);

    for (int w=0; w<assoc; w++) {
        if (line[idx*assoc+w].valid && line[idx*assoc+w].tag == tag) {
            return line[idx*assoc+w];
        }
    }
    CacheLine c;
    c.valid = false;
    return c;
}

// Call this only if you know that a valid line with matching tag exists at that address 
void Cache::writeBackLine(CacheLine evictedLine) {
    int idx = getIndex(evictedLine.address);
    int tag = getTag(evictedLine.address);

    for (int w=0; w<assoc; w++) {
        if (line[idx*assoc+w].valid && line[idx*assoc+w].tag == tag) {
            for (int i = 0; i < CACHE_LINE_SIZE/4; i++) {
                line[idx*assoc+w].data[i] = evictedLine.data[i];
            }
            line[idx*assoc+w].dirty = true;
        }
    }
}

// Replace a line at the set corresponding this address
void Cache::replace(uint32_t address, CacheLine newLine, CacheLine &evictedLine) {
    int idx = getIndex(address);
    newLine.address = address;
    newLine.tag = getTag(address);
    newLine.valid = true;
   
    /* Return if replacement already completed. */ 
    for (int w=0; w<assoc; w++) {
        if (line[idx*assoc+w].valid && line[idx*assoc+w].tag == newLine.tag) {
            return;
        }
    }
    /* Replace. */ 
    for (int w=0; w<assoc; w++) {
        if (!line[idx*assoc+w].valid || line[idx*assoc+w].replBits == 0) {
            DEBUG(cout << name + " Cache: replacing line at idx:" << idx << " way:" << w << " due to conflicting address:" << std::hex << address << std::dec << "\n");
            evictedLine = line[idx*assoc+w];
            line[idx*assoc+w] = newLine;
            return;
        }
    }
}

// Invalidate a line
void Cache::invalidateLine(uint32_t address) {
    int idx = getIndex(address);
    int tag = getTag(address);

    for (int w=0; w<assoc; w++) {
        if (line[idx*assoc+w].valid && line[idx*assoc+w].tag == tag) {
            line[idx*assoc+w].valid = false;
        }
    }
}

bool Memory::access(uint32_t address, uint32_t &read_data, uint32_t write_data, bool mem_read, bool mem_write) {
    if (opt_level == 0) {
        if (mem_read) {
            read_data = mem[address/4];
        }
        if (mem_write) {
            mem[address/4] = write_data;
        }
        return true;
    }

    if (!mem_read && !mem_write) {
        return true;
    }

    if ((mem_read && L1.read(address, read_data)) || (mem_write && L1.write(address, write_data))) {
        return true;
    } else if ((mem_read && L2.read(address, read_data)) || (mem_write && L2.write(address, write_data))) {
        // Read from L2 but don't return a success status until miss penalty is paid off completely
        CacheLine evictedLine;
        L1.replace(address, L2.readLine(address), evictedLine);

        // writeback dirty line
        if (evictedLine.valid && evictedLine.dirty) {
            L2.writeBackLine(evictedLine);
        }
    } else {
        // Read from memory but don't return a success status until miss penalty is paid off completely
        int lineAddr = address & ~(CACHE_LINE_SIZE-1);
        CacheLine c;
        CacheLine evictedLine;
        evictedLine.valid = false;
        DEBUG(print(lineAddr, 8));
        for (int i = 0; i < CACHE_LINE_SIZE/4; i++) {
           c.data[i] = mem[lineAddr/4+i];
        }
        L2.replace(address, c, evictedLine); 

        // model an inclusive hierarchy
        if (evictedLine.valid) {
            L1.invalidateLine(evictedLine.address);
        }

        // writeback dirty line
        if (evictedLine.valid && evictedLine.dirty) {
            lineAddr = evictedLine.address & ~(CACHE_LINE_SIZE-1);
            for (int i = 0; i < CACHE_LINE_SIZE/4; i++) {
               mem[lineAddr/4+i] = evictedLine.data[i];
            }
        }
    }
    return false;
}

// --- NonBlockingCache Method Implementations ---

// Constructor: initialize members in declaration order (cache → sets → assoc → mshrCount → latency → mshr)
NonBlockingCache::NonBlockingCache(size_t numSets,
                                   size_t assoc,
                                   size_t mshrEntries,
                                   unsigned refillLatency)
  : cache("", numSets * assoc * CACHE_LINE_SIZE, assoc, refillLatency),
    sets(numSets),
    assoc(assoc),
    mshrCount(mshrEntries),
    latency(refillLatency),
    mshr(mshrEntries) {}

// Search for an existing MSHR entry to support miss coalescing
int NonBlockingCache::findMSHR(uint64_t blockAddr) {
    for (int i = 0; i < (int)mshrCount; ++i) {
        if (mshr[i].valid && mshr[i].blockAddr == blockAddr)
            return i;
    }
    return -1;
}

// Allocate a free MSHR entry; return -1 if none available (structural stall)
int NonBlockingCache::allocMSHR(uint64_t blockAddr,
                                bool isLoad,
                                uint32_t instrId) {
    for (int i = 0; i < (int)mshrCount; ++i) {
        if (!mshr[i].valid) {
            mshr[i].valid = true;
            mshr[i].blockAddr = blockAddr;
            mshr[i].isLoad = isLoad;
            mshr[i].waitingInstrIds.clear();
            mshr[i].waitingInstrIds.push_back(instrId);
            return i;
        }
    }
    return -1;
}

// Access the cache: true+data on hit; false on miss (enqueues MSHR/refill)
bool NonBlockingCache::access(uint32_t address,
                              bool isLoad,
                              uint32_t instrId,
                              uint32_t &outData) {
    // Compute block-aligned address for MSHR indexing
    uint64_t blockAddr = address / CACHE_LINE_SIZE;
    uint32_t loc;
    // Hit path: return data immediately from underlying cache
    if (cache.isHit(address, loc)) {
        CacheLine line = cache.readLine(address);
        uint32_t offset = cache.getOffset(address);
        outData = line.data[offset / 4];
        cache.updateReplacementBits(cache.getIndex(address), loc % assoc);
        return true;
    }
    // Miss: check if this block is already being fetched (coalesce miss)
    int idx = findMSHR(blockAddr);
    if (idx >= 0) {
        mshr[idx].waitingInstrIds.push_back(instrId);
        return false;
    }
    // Allocate a new MSHR entry for this miss if possible
    idx = allocMSHR(blockAddr, isLoad, instrId);
    if (idx < 0) {
        // Structural stall: MSHR table full
        return false;
    }
    // Schedule cache-line refill after specified latency
    refillQ.push({blockAddr, static_cast<int>(latency)});
    return false;
}

// Each cycle: process the next pending refill request if any
void NonBlockingCache::tick() {
    if (refillQ.empty()) return;
    auto &req = refillQ.front();
    // Decrement remaining cycles until this refill completes
    if (--req.second == 0) {
        // Refill complete: install line into cache and wake waiting instructions
        uint64_t blockAddr = req.first;
        refillQ.pop();
        uint32_t address = blockAddr * CACHE_LINE_SIZE;
        // Install the filled line into cache (evict if needed)
        CacheLine newLine;
        // TODO: copy data bytes from main memory 'mem' into newLine.data[]
        newLine.address = address;
        newLine.tag = cache.getTag(address);
        newLine.valid = true;
        newLine.dirty = false;
        // Evicted line goes into 'evicted' so we don't overwrite our newLine buffer
        CacheLine evicted;
        cache.replace(address, newLine, evicted);
        int idx = findMSHR(blockAddr);
        if (idx >= 0) {
            for (auto instId : mshr[idx].waitingInstrIds) {
                // TODO: once fill completes, enqueue 'instId' into the processor's replay/ready queue
            }
            mshr[idx].valid = false;
            mshr[idx].waitingInstrIds.clear();
        }
    }
}
