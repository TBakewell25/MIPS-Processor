#include <vector>

class StoreBuffer {
    public:
        struct entry {
            uint32_t address; // address for store
            uint32_t data;
            int size;
            bool ready;
            int rob_index;
        };

        std::vector<entry> StoreBuffer;

        void enqueue(uint32_t addr, uint32_t data, int size, int rob_idx);

        bool checkConflict(uint32_t addr, int size);

        bool forwardData(uint32_t addr, int size, uint32_t& result);

        void commitStore(Memory* memory);
};

class LoadQueue {
    public:
        struct entry {
            uint32_t address;
            int size;           // 1=byte, 2=halfword, 4=word
            int rob_index;      // position in ROB for ordering
            bool speculative;   // was executed speculatively
        };
      
        std::vector<entry> LoadQueue;

        void enqueueLoad(uint32_t addr, int size, int rob_idx);
    
        // verify a speculative load (check for ordering violations)
        bool verifyLoad(int load_idx, StoreBuffer* store_buffer);
    
        // remove a committed load
        void removeLoad(int rob_idx);
};
  
