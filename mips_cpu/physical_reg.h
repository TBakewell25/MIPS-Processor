#include <vector>
#include <queue>

#ifdef enable_debug
#define debug(x) x
#else
#define debug(x) 
#endif

#define ARCH_REG 32
#define REG_COUNT 128

// 32 architectural registers mapping to up to 128 physical ones

/*
* A class to handle register allocation
* for both physical and architectural
* registers.
*
* Implements renaming, translation, and ROB management
*/

class PhysicalRegisterUnit {
    public:
        struct ROBEntry {
            uint32_t instruction;
            int dest_reg;        // Architectural destination register
            int phys_reg;        // Physical destination register
            int old_phys_reg;    // Previous mapping for recovery
            bool completed;      // Has the instruction completed execution?
            uint32_t result;     // Result value
            bool ready_to_commit;
        };

        class RAT {
            private:
                uint32_t mappings[ARCH_REG]; // Direct mapping from arch to physical register
                bool valid[ARCH_REG];        // Is this mapping valid?

            public:
                RAT() {
                    for (int i = 0; i < ARCH_REG; i++) {
                        mappings[i] = i;     // Initial identity mapping
                        valid[i] = true;     // All mappings start valid
                    }
                }

                // Get the current physical register for an architectural register
                uint32_t getLiveMapping(uint32_t arch_reg) {
                    if (arch_reg == 0) return 0; // R0 is always 0
                    return valid[arch_reg] ? mappings[arch_reg] : 0;
                }

                // Update mapping for an architectural register
                void updateMapping(uint32_t arch_reg, uint32_t phys_reg) {
                    if (arch_reg == 0) return; // R0 can't be remapped
                    
                    mappings[arch_reg] = phys_reg;
                    valid[arch_reg] = true;
                }

                // Get the previous mapping (for ROB recovery)
                uint32_t getPreviousMapping(uint32_t arch_reg) {
                    return mappings[arch_reg];
                }
        };


    private: 
        uint32_t physTable[ARCH_REG * 4]; // a table holding n many physical registers (32-bit each) 

        std::vector<uint32_t>freePhysRegs;
        //uint32_t freePhysRegs[ARCH_REG * 4]; // free list of physical registers
        bool regReady[REG_COUNT];          // is the register value ready?

        // Reorder Buffer (ROB)
        std::vector<ROBEntry> reorderBuffer; // the instruction reorder buffer, a circular buffer implemented below
        
        ///values for circ buffer implementation
        int head, tail, capacity;

        bool is_full;

        // Stores the most recently committed ROB entry for processor to query
        ROBEntry lastCommittedEntry;

    public:

        bool checkReady(int phys_reg) { return regReady[phys_reg]; }

        // Register Alias Table implementation
        RAT RAT_Unit;

        int getMapping(uint32_t arch_reg) { return RAT_Unit.getLiveMapping(arch_reg); }

        bool checkFreePhys() { return freePhysRegs.empty(); }
        // Enqueue operation
        // Returns the ROB index on success, -1 on error
        int enqueue(ROBEntry value) {
            if (is_full)
                return -1;

            reorderBuffer[tail] = value;
            int rob_index = tail;
            tail = (tail + 1) % capacity;

            // Check if reorder buffer is now full
            if (head == tail) {
                is_full = true;
            }

            return rob_index;
        }

        // Dequeue operation
        ROBEntry dequeue() {
            ROBEntry value = reorderBuffer[head];
            head = (head + 1) % capacity;
            is_full = false;  // After dequeue, buffer cannot be full
        
            return value;
        }

        // Check if reorderbuffer is empty
        bool isEmpty() const {
            return (head == tail) && !is_full;
        }

        // Check if reorderbuffer is full
        bool isFull() const { return is_full; }

        // Get current size of reorder
        int size() const {
            if (is_full) 
                return capacity;
            
            return (tail >= head) ? (tail - head) : (capacity - head + tail);
        }
        
        // allocate a physical register
        int allocatePhysReg() {
            if (freePhysRegs.empty())
                return -1; // no free registers
                
            int reg = freePhysRegs.front();
            freePhysRegs.pop_back();
            regReady[reg] = false; // not ready until written
            return reg;
        }

        // free a physical register
        void freePhysReg(int reg) {
            if (reg >= ARCH_REG)
                freePhysRegs.push_back(reg);
        }

        // set a register as ready with its value
        void setRegReady(int reg, uint32_t value) {
            physTable[reg] = value;
            regReady[reg] = true;
        }

        // check if register value is ready
        bool isValueReady(int reg) {
            return regReady[reg];
        }

        // get register value (only if ready)
        uint32_t getValue(int reg) {
            return physTable[reg];
        }

        // write to the physical register
        void writeRegister(int phys_reg, uint32_t val) {
            physTable[phys_reg] = val;
            regReady[phys_reg] = true;
        }
        
        // get number of free physical registers
        int freeRegistersCount() {
            return freePhysRegs.size();
        }
        
        // mark a ROB entry as completed with result
        void completeROBEntry(int rob_index, uint32_t result) {
            if (rob_index < 0 || rob_index >= capacity) return;
            reorderBuffer[rob_index].completed = true;
            reorderBuffer[rob_index].result    = result;
        }
        
        // Check if head of ROB is ready to commit
        bool isHeadReadyToCommit() {
            if (isEmpty())
                return false;
                
            return reorderBuffer[head].completed;
        }
        
        // get the index of a ROB entry (for CDB broadcasts)
        int findROBEntryByPhysReg(int phys_reg) {
            for (int i = 0; i < capacity; i++) {
                int idx = (head + i) % capacity;
           
                if (reorderBuffer[idx].phys_reg == phys_reg)
                    return idx;
            }
            return -1;
        }
        
        // get access to the RAT for renaming operations
        RAT* getRAT() {
            return &RAT_Unit;
        }
        
        // Commit the head ROB entry:
        // - Processor should use the returned entry.result and entry.dest_reg
        //   to update the architectural register file before calling this.
        bool commitHead() {
            if (isEmpty() || !reorderBuffer[head].completed)
                return false;
                
            ROBEntry entry = dequeue();
            lastCommittedEntry = entry;
            
            // If this instruction writes to a register
            if (entry.dest_reg != -1 && entry.phys_reg != -1) {
                // The architectural commit would happen here in hardware
                // For simulation, we just update our tracking structures
                
                // Free the old physical register that was previously mapped
                // to this architectural register
                if (entry.old_phys_reg != -1) {
                    freePhysReg(entry.old_phys_reg);
                }
            }
            
            return true;
        }

        // After a successful commitHead(), retrieve the committed entry
        const ROBEntry& getLastCommittedEntry() const {
            return lastCommittedEntry;
        }

        // Update the capacity in constructor
        PhysicalRegisterUnit(int reg_count) {
            capacity = reg_count - ARCH_REG;
            head = 0;
            tail = 0;
            is_full = false;

            reorderBuffer.resize(capacity);

            for (int i = ARCH_REG; i < reg_count; i++) 
                freePhysRegs.push_back(i);

            for (int i = 0; i < reg_count; i++)
                regReady[i] = (i < ARCH_REG);
        }
        
        void recover(int rob_entry_index) {
            // Flush all ROB entries younger than the mispredicted one
            int idx = (rob_entry_index + 1) % capacity;
            while (idx != tail) {
                ROBEntry &entry = reorderBuffer[idx];
                // Restore RAT to the old mapping for dest_reg
                if (entry.dest_reg != -1 && entry.old_phys_reg != -1) {
                    RAT_Unit.updateMapping(entry.dest_reg, entry.old_phys_reg);
                }
                // Free the newly allocated physical register
                if (entry.phys_reg != -1) {
                    freePhysReg(entry.phys_reg);
                }
                idx = (idx + 1) % capacity;
            }
            // Move tail back to just after the recovered entry
            tail = (rob_entry_index + 1) % capacity;
            is_full = false;
        }
};
