#ifndef MEMORY
#define MEMORY
#include <vector>
#include <cstdint>
#include <iostream>
#include <cmath>

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

class Memory {
    private:
        std::vector<uint32_t> mem;
        Cache L1 = Cache("L1", 32768, 8, 12);
        Cache L2 = Cache("L2", 262144, 8, 59);
        int opt_level;
    public:
        Memory() {
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
