#ifndef MEMORY
#define MEMORY
#include <vector>
#include <queue>
#include <cstdint>
#include <iostream>
#include <string>
#include <cmath>
#include <cstddef>

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
        Cache(std::string nm, int sz, int asc, int penalty);

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
        void printLine(uint32_t address);
};

class NonBlockingCache {
public:
    NonBlockingCache(size_t numSets,
                     size_t assoc,
                     size_t mshrEntries,
                     unsigned refillLatency);

    // Access the cache: true+data on hit; false on miss
    bool access(uint32_t address,
                bool isLoad,
                uint32_t instrId,
                uint32_t &outData);

    // Advance outstanding misses each cycle
    void tick();

    // Underlying cache operations
    bool read(uint32_t address, uint32_t &outData);
    bool write(uint32_t address, uint32_t writeData);
    CacheLine readLine(uint32_t address);
    void replace(uint32_t address, CacheLine newLine, CacheLine &evictedLine);
    void writeBackLine(CacheLine evictedLine);
    void invalidateLine(uint32_t address);

private:
    // Find existing MSHR entry or -1
    int findMSHR(uint64_t blockAddr);

    // Allocate a free MSHR entry or -1
    int allocMSHR(uint64_t blockAddr,
                  bool isLoad,
                  uint32_t instrId);

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
        Memory();
        Memory(size_t size_bytes, int optLevel);

        void setOptLevel(int level);
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
        void print(uint32_t address, int num_words);
};

#endif