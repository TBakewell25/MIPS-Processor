class MemoryOperationUnit {
public:
    // Store Buffer entry - holds stores until they can commit
    struct StoreQueueEntry {
        uint32_t instruction;
        uint32_t address;
        uint32_t data;
        int size;              // 1=byte, 2=halfword, 4=word
        int rob_index;
        bool executed;
        bool address_ready;    // Address calculation complete
        
        StoreQueueEntry() : address(0), data(0), size(0), rob_index(-1), address_ready(false) {}
        StoreQueueEntry(int rob, uint32_t instruction) : address(0), data(0), size(0), rob_index(rob), address_ready(false), instruction(instruction) {}
    };
    
    // Load Queue entry - for tracking loads and detecting ordering violations
    struct LoadQueueEntry {
        uint32_t instruction;
        uint32_t address;
        int size;              // 1=byte, 2=halfword, 4=word
        int rob_index;
        int dest_reg;          // Physical register destination
        bool address_ready;    // Address calculation complete
        bool executed;         // Load has been executed
        
        LoadQueueEntry() : address(0), size(0), rob_index(-1), dest_reg(-1), 
                          address_ready(false), executed(false) {}
        LoadQueueEntry(int rob, int dest, uint32_t inst) : address(0), size(0), rob_index(rob), 
                                           dest_reg(dest), address_ready(false), executed(false), instruction(inst) {}
    };
    
    std::vector<StoreQueueEntry> store_queue;
    std::vector<LoadQueueEntry> load_queue;

    MemoryOperationUnit& operator=(const MemoryOperationUnit& other) {
        if (this != &other) {
            // clear and resize vectors to match the source
            store_queue.clear();
            store_queue.resize(other.store_queue.size());
        
            load_queue.clear();
            load_queue.resize(other.load_queue.size());
        
            for (size_t j = 0; j < other.store_queue.size(); ++j)
                store_queue[j] = other.store_queue[j];

            for (size_t i = 0; i < other.load_queue.size(); ++i) 
                load_queue[i] = other.load_queue[i];
        }
        return *this;
    }

    int findByInstructionLoad(uint32_t instruction) {
        for (int j = 0; j < load_queue.size(); ++j) {
            if (load_queue[j].instruction == instruction)
                return j;
        }
        return -1;
    }

     int findByInstructionStore(uint32_t instruction) {
        for (int j = 0; j < store_queue.size(); ++j) {
            if (store_queue[j].instruction = instruction)
                return j;
        }
        return -1;
    }  
    // Add a store to the store queue
    int addStore(int rob_index, int size, uint32_t instruction) {
        store_queue.emplace_back(rob_index, instruction);
        store_queue.back().size = size;
        return store_queue.size() - 1;
    }
    
    // Add a load to the load queue
    int addLoad(int rob_index, int dest_reg, int size, uint32_t instruction) {
        load_queue.emplace_back(rob_index, dest_reg, instruction);
        load_queue.back().size = size;
        return load_queue.size() - 1;
    }
    
    // Update store address and data
    void updateStore(int index, uint32_t address, uint32_t data) {
        //int index = store_queue.size() - 1; // always tail
        if (index >= 0 && index < store_queue.size()) {
            store_queue[index].address = address;
            store_queue[index].data = data;
            store_queue[index].address_ready = true;
            store_queue[index].executed = true;
        }
    }
    
    // Update load address
    void updateLoad(int index, uint32_t address) {
        if (index >= 0 && index < load_queue.size()) {
            load_queue[index].address = address;
            load_queue[index].address_ready = true;
        }
    }

    bool executeStore(int index, Memory *memory) {
        if (index < 0 || index >= store_queue.size() || !store_queue[index].address_ready)
            return false;

        StoreQueueEntry& store = store_queue[index];

        uint32_t dummy;
        bool success = false;
        while(!success)
            success = memory->access(store.address, dummy, store.data, 0, 1);
        
        //if (!success) return false;

        store.executed = true;

        return true;
    }

         
    // Execute a load and return the result
    bool executeLoad(int index, Memory* memory, uint32_t& result) {
        if (index < 0 || index >= load_queue.size() || !load_queue[index].address_ready)
            return false;
        
        LoadQueueEntry& load = load_queue[index];
        
        // Check for memory ordering violations with older stores
        for (const auto& store : store_queue) {
            if (store.rob_index < load.rob_index && store.address_ready) {
                // If addresses overlap and the store is older, we should wait
                if (addressesOverlap(store.address, store.size, load.address, load.size)) {
                    result = store.data;
                    return true;
                }
            }
        }
        
        // No violations, execute the load
        uint32_t read_data;
        bool success = memory->access(load.address, read_data, 0, 1, 0);
        
        if (!success) return false; // Memory access failed
        
        // Apply masking based on size
        if (load.size == 1) { // byte
            result = read_data & 0xFF;
        } else if (load.size == 2) { // halfword
            result = read_data & 0xFFFF;
        } else { // word
            result = read_data;
        }
        
        load.executed = true;
        return true;
    }

    // Commit a store (perform the actual memory write)
    bool commitStore(int rob_index, Memory* memory) {
        for (auto it = store_queue.begin(); it != store_queue.end(); ++it) {
            if (it->executed) {
                store_queue.erase(it);
                return true;
            }
            /*if (it->rob_index == rob_index) {
                if (!it->address_ready) return false;
                
                // Perform the actual memory write
                uint32_t dummy;
                bool success = memory->access(it->address, dummy, it->data, 0, 1);
                
                if (!success) return false;
                
                // Remove the store from the queue
                store_queue.erase(it);
                return true;
            }*/
        }
        return false;
    }
    
    // Remove a load from the queue
    void removeLoad(int rob_index) {
        for (auto it = load_queue.begin(); it != load_queue.end(); ++it) {
            if (it->rob_index == rob_index) {
                load_queue.erase(it);
                break;
            }
        }
    }
 

    void update() {
        for (int j = 0; j < load_queue.size(); ++j) 
            load_queue[j].rob_index--;
        for (int j = 0; j < store_queue.size(); ++j) 
            store_queue[j].rob_index--;
     }


   
private:
    // Helper function to check if two address ranges overlap
    bool addressesOverlap(uint32_t addr1, int size1, uint32_t addr2, int size2) {
        return (addr1 <= addr2 && addr1 + size1 > addr2) ||
               (addr2 <= addr1 && addr2 + size2 > addr1);
    }
};
