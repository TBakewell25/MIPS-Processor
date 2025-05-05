#ifndef MEMORY
#define MEMORY
#include <vector>
#include <queue>
#include <cstdint>
#include <iostream>
#include <string>
#include <cmath>

struct MSHREntry {
    uint64_t blockAddr;               // block-aligned address
    bool     valid        = false;    // entry in use?
    bool     isLoad = false;          // load vs store (default false)
    std::vector<uint32_t> waitingInstrIds; // instructions waiting on this block
};

#define CACHE_LINE_SIZE 64

struct CacheLine {
    uint32_t data[CACHE_LINE_SIZE/4];
    uint32_t address;
    int tag;
    bool valid;
    bool dirty;
    uint8_t replBits;
};

class Cache {
    private:
        std::vector<CacheLine> line;
        int size;
        int assoc;
        int missPenalty;
        int missCountdown;
        std::string name;
    public:
        Cache(std::string nm, int sz, int asc, int penalty) {
            name = nm;
            size = sz;
            assoc = asc;
            line.resize(size/CACHE_LINE_SIZE);

            for (int i = 0; i < (size/CACHE_LINE_SIZE); i++) {
                line[i].valid = false;
            }
            
            missCountdown = 0;
            missPenalty = penalty;
        }

        // offset, index, tag computation
        int getOffset(uint32_t address) {
            return address & (CACHE_LINE_SIZE-1);
        }
        int getIndex(uint32_t address) {
            return (address >> (int)log2(CACHE_LINE_SIZE)) & (size/CACHE_LINE_SIZE-1);
        }
        int getTag(uint32_t address) {
            return address >> (int)log2(size);
        }

        // Check if hit in the cache
        bool isHit(uint32_t address, uint32_t &loc);

        // Update replacement bits after access
        void updateReplacementBits(int idx, int way);

        // Read a word from this cache
        bool read(uint32_t address, uint32_t &read_data);

        // Write a word to this cache
        bool write(uint32_t address, uint32_t write_data);

        // Call this only if you know that a valid line with matching tag exists at that address 
        CacheLine readLine(uint32_t address);

        // Call this only if you know that a valid line with matching tag exists at that address 
        void writeBackLine(CacheLine evictedLine);

        // Replace a line at the set corresponding this address
        void replace(uint32_t address, CacheLine newLine, CacheLine &evictedLine);

        // Invalidate a line
        void invalidateLine(uint32_t address);

        // Print a cache line
        void printLine(uint32_t address) {
            int idx = getIndex(address);
            int tag = getTag(address);

            for (int w=0; w<assoc; w++) {
                if (line[idx*assoc+w].valid && line[idx*assoc+w].tag == tag) {
                    std::cout<< "Valid:" << line[idx*assoc+w].valid << "\n";
                    std::cout<< "Address:" << line[idx*assoc+w].address << "\n";
                    std::cout<< "Tag:" << line[idx*assoc+w].tag << "\n";
                    std::cout<< "Dirty:" << line[idx*assoc+w].dirty << "\n";
                    std::cout<< "Replacement Bits:" << line[idx*assoc+w].replBits << "\n";
                    for (int i = 0; i < CACHE_LINE_SIZE/4; i++) {
                        std::cout<< "DATA[" << i << "]: " << line[idx*assoc+w].data[i] << "\n";
                    }
                    return;
                }
            }
        }
};

class NonBlockingCache {
public:
    // Constructor: initialize members in declaration order 
    NonBlockingCache(size_t numSets,
                     size_t assoc,
                     size_t mshrEntries,
                     unsigned refillLatency)
      : cache("", numSets * assoc * CACHE_LINE_SIZE, assoc, refillLatency),
        sets(numSets),
        assoc(assoc),
        mshrCount(mshrEntries),
        latency(refillLatency),
        mshr(mshrEntries) {}

    // Access the cache: true+data on hit; false on miss
    bool access(uint32_t address,
                bool isLoad,
                uint32_t instrId,
                uint32_t &outData) {
        uint64_t blockAddr = address / CACHE_LINE_SIZE;
        uint32_t loc;
        if (cache.isHit(address, loc)) {
            CacheLine line = cache.readLine(address);
            uint32_t offset = cache.getOffset(address);
            outData = line.data[offset / 4];
            cache.updateReplacementBits(cache.getIndex(address), loc % assoc);
            return true;
        }
        int idx = findMSHR(blockAddr);
        if (idx >= 0) {
            mshr[idx].waitingInstrIds.push_back(instrId);
            return false;
        }
        idx = allocMSHR(blockAddr, isLoad, instrId);
        if (idx < 0) {
            return false;
        }
        refillQ.push({blockAddr, static_cast<int>(latency)});
        return false;
    }

    // Advance outstanding misses each cycle
    void tick() {
        if (refillQ.empty()) return;
        auto &req = refillQ.front();
        if (--req.second == 0) {
            uint64_t blockAddr = req.first;
            refillQ.pop();
            uint32_t address = blockAddr * CACHE_LINE_SIZE;
            // Install line into cache (evict if needed)
            CacheLine newLine;
            // TODO: copy data bytes from main memory 'mem' into newLine.data[]
            //       e.g. for each word i in 0..CACHE_LINE_SIZE/4: newLine.data[i] = mem[(address/4)+i];
            newLine.address = address;
            newLine.tag = cache.getTag(address);
            newLine.valid = true;
            newLine.dirty = false;
            // Install new line, capturing any evicted line
            CacheLine evicted;
            // Evicted line goes into 'evicted' so we don't overwrite our newLine buffer
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

private:
    // Find existing MSHR entry or -1
    int findMSHR(uint64_t blockAddr) {
        for (int i = 0; i < (int)mshrCount; ++i) {
            if (mshr[i].valid && mshr[i].blockAddr == blockAddr)
                return i;
        }
        return -1;
    }

    // Allocate a free MSHR entry or -1
    int allocMSHR(uint64_t blockAddr,
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

    Cache cache;  // underlying blocking cache for tag/data arrays
    size_t sets, assoc, mshrCount, latency;
    std::vector<MSHREntry> mshr;
    std::queue<std::pair<uint64_t,int>> refillQ;
};

class Memory {
    private:
        std::vector<uint32_t> mem;
        NonBlockingCache L1;
        NonBlockingCache L2;
        int opt_level;
    public:
        Memory()
          : L1( 32768 / CACHE_LINE_SIZE / 8, 8, 16, 12),
            L2( 262144 / CACHE_LINE_SIZE / 8, 8, 32, 59) {
            mem.resize(2097152, 0);
            opt_level = 0;
        }
        void setOptLevel(int level) {
            opt_level = level;
        }
        // address is the adress which needs to be read or written from
        // read_data the variable into which data is read, it is passed by reference
        // write_data is the data which is written into the memory address provided
        // mem_read specifies whether memory should be read or not
        // mem_write specifies whether memory whould be written to or not
        // returns false if there is a cache miss (O1 and above) 
        // -- currently follows stall-on-miss model, so call every cycle until you see a hit
        bool access(uint32_t address, uint32_t &read_data, uint32_t write_data, bool mem_read, bool mem_write);

        // given a starting address and number of words from that starting address
        // this function prints int values at the memory
        void print(uint32_t address, int num_words) {
            for (uint32_t i = address; i < address+num_words; ++i) {
                std::cout<< "MEM[" << std::hex << i << "]: " << mem[i] << std::dec << "\n";
            }
        }
};

#endif
