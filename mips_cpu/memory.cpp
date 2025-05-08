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
