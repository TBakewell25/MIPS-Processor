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
* Implements renaming, translation, and ROB management using a simple vector
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
            int rob_index;
            
            // Branch-related fields
            bool is_branch;
            bool predicted_taken;
            bool actual_taken;
            uint32_t branch_target;
            uint32_t recovery_pc;
            bool mispredicted;
            
            // Constructor to ensure proper initialization
            ROBEntry() : 
                instruction(0),
                dest_reg(-1),
                phys_reg(-1),
                old_phys_reg(-1),
                completed(false),
                result(0),
                ready_to_commit(false),
                rob_index(-1),
                is_branch(false),
                predicted_taken(false),
                actual_taken(false),
                branch_target(0),
                recovery_pc(0),
                mispredicted(false) {}
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
        std::vector<uint32_t> freePhysRegs;
        bool regReady[REG_COUNT];          // is the register value ready?

        // Simple vector-based reorder buffer
        int maxBufferSize;  // Maximum allowed size of the ROB
        
        // Stores the most recently committed ROB entry for processor to query
        ROBEntry lastCommittedEntry;

    public:

        std::vector<ROBEntry> reorderBuffer;

        // Register Alias Table implementation
        RAT RAT_Unit;

        // Default constructor
        PhysicalRegisterUnit() : maxBufferSize(0) {
            // Zero out the physical register table
            for (int i = 0; i < ARCH_REG * 4; i++) {
                physTable[i] = 0;
            }
            
            // Initialize register ready flags
            for (int i = 0; i < REG_COUNT; i++) {
                regReady[i] = false;
            }
        }

        void flushROB(int index) {
            std::vector<ROBEntry> newROB;
            for (int j = 0; j < index; ++j) {
                ROBEntry preserve = reorderBuffer[j];
                newROB.push_back(preserve);
            }

            reorderBuffer = newROB;
            return;
        }

        // Constructor with register count
        PhysicalRegisterUnit(int reg_count) {
            // Validate input
            if (reg_count <= ARCH_REG || reg_count > REG_COUNT) {
                maxBufferSize = REG_COUNT - ARCH_REG; // Default to safe value
            } else {
                maxBufferSize = reg_count - ARCH_REG;
            }
            
            // Zero out the physical register table
            for (int i = 0; i < ARCH_REG * 4; i++) {
                physTable[i] = 0;
            }
            
            // Clear the free physical registers list and add registers
            freePhysRegs.clear();
            for (int i = ARCH_REG; i < reg_count && i < REG_COUNT; i++) {
                freePhysRegs.push_back(i);
            }

            // Initialize register ready flags
            for (int i = 0; i < REG_COUNT; i++) {
                regReady[i] = (i < ARCH_REG); // Only architectural registers start ready
            }
        }

        // Assignment operator - simplified with vector
        PhysicalRegisterUnit& operator=(const PhysicalRegisterUnit& other) {
            if (this != &other) {
                // Copy physical register table
                for (int i = 0; i < ARCH_REG * 4; i++) {
                    physTable[i] = other.physTable[i];
                }

                // Copy free physical registers list
                freePhysRegs = other.freePhysRegs;

                // Copy register ready flags
                for (int i = 0; i < REG_COUNT; i++) {
                    regReady[i] = other.regReady[i];
                }

                // Copy ROB 
                maxBufferSize = other.maxBufferSize;
                if (other.reorderBuffer.size() > 0 &&  other.reorderBuffer.size() < 1000)
                    reorderBuffer = other.reorderBuffer;  // Simple vector copy
 
                else
                    std::vector<ROBEntry> reorderBuffer {};

                // Copy the last committed entry
                lastCommittedEntry = other.lastCommittedEntry;

                // Copy the Register Alias Table
                RAT_Unit = other.RAT_Unit;
            }
            return *this;
        }

        // Search ROB for an instruction
        int searchByInstruction(uint32_t instruction) {
            for (size_t i = 0; i < reorderBuffer.size(); i++) {
                if (reorderBuffer[i].instruction == instruction) {
                    return i;
                }
            }
            return -1;
        }

        // Check if a physical register is ready
        bool checkReady(int phys_reg) { 
            if (phys_reg < 0 || phys_reg >= REG_COUNT) return false;
            return regReady[phys_reg]; 
        }

        // Get physical register mapping for an architectural register
        int getMapping(uint32_t arch_reg) { 
            return RAT_Unit.getLiveMapping(arch_reg); 
        }

        // Check if there are any free physical registers
        bool checkFreePhys() { 
            return freePhysRegs.empty(); 
        }

        // Add an entry to the ROB (enqueue)
        int enqueue(ROBEntry value) {
            if (reorderBuffer.size() >= maxBufferSize) 
                return -1;  // ROB is full
            
            // Set the ROB index
            value.rob_index = reorderBuffer.size();
            
            // Simply add to the end of the vector
            reorderBuffer.push_back(value);
            
            return value.rob_index;
        }
        
        // Remove the oldest entry from ROB (dequeue)
        ROBEntry dequeue() {
            if (reorderBuffer.empty()) {
                return ROBEntry();  // Return empty entry if ROB is empty
            }
            
            // Save the oldest entry (at front of vector)
            ROBEntry oldestEntry = reorderBuffer.front();
            
            // Remove it from the buffer
            reorderBuffer.erase(reorderBuffer.begin());
            
            // Update ROB indices for remaining entries
            for (size_t i = 0; i < reorderBuffer.size(); i++) {
                reorderBuffer[i].rob_index = i;
            }
            
            return oldestEntry;
        }

        // Check if ROB is empty
        bool isEmpty() const {
            return reorderBuffer.empty();
        }

        // Check if ROB is full
        bool isFull() const { 
            return reorderBuffer.size() >= maxBufferSize; 
        }

        // Get current size of ROB
        int size() const {
            return reorderBuffer.size();
        }
        
        // Allocate a physical register
        int allocatePhysReg() {
            if (freePhysRegs.empty())
                return -1; // no free registers
                
            int reg = freePhysRegs.front();
            freePhysRegs.erase(freePhysRegs.begin());
            regReady[reg] = false; // not ready until written
            return reg;
        }

        // Free a physical register
        void freePhysReg(int reg) {
            if (reg >= ARCH_REG && reg < REG_COUNT)
                freePhysRegs.push_back(reg);
        }

        // Set a register as ready with its value
        void setRegReady(int reg, uint32_t value) {
            if (reg >= 0 && reg < REG_COUNT) {
                physTable[reg] = value;
                regReady[reg] = true;
            }
        }

        // Check if register value is ready
        bool isValueReady(int reg) {
            if (reg < 0 || reg >= REG_COUNT) return false;
            return regReady[reg];
        }

        // Get register value (only if ready)
        uint32_t getValue(int reg) {
            if (reg < 0 || reg >= REG_COUNT) return 0;
            return physTable[reg];
        }

        // Write to the physical register
        void writeRegister(int phys_reg, uint32_t val) {
            if (phys_reg >= 0 && phys_reg < REG_COUNT) {
                physTable[phys_reg] = val;
                regReady[phys_reg] = true;
            }
        }
        
        // Get number of free physical registers
        int freeRegistersCount() {
            return freePhysRegs.size();
        }
        
        // Mark a ROB entry as completed with result based on physical register
        void completeROBEntry(int phys_dest, uint32_t result) {
            for (size_t i = 0; i < reorderBuffer.size(); i++) {
                if (reorderBuffer[i].phys_reg == phys_dest) {
                    reorderBuffer[i].completed = true;
                    reorderBuffer[i].result = result;
                    return;
                }
            }
        }

        // Mark a specific ROB entry as completed (for load/store)
        void completeByIndex(int index, uint32_t result) {
            if (index < 0 || index >= reorderBuffer.size())
                return;
                
            reorderBuffer[index].completed = true;
            reorderBuffer[index].result = result;
        }
        
        // Check if head of ROB is ready to commit
        bool isHeadReadyToCommit() {
            if (reorderBuffer.empty())
                return false;
                
            return reorderBuffer[0].completed;
        }
        
        // Find ROB entry by physical register (for CDB broadcasts)
        int findROBEntryByPhysReg(int phys_reg) {
            for (size_t i = 0; i < reorderBuffer.size(); i++) {
                if (reorderBuffer[i].phys_reg == phys_reg)
                    return i;
            }
            return -1;
        }
        
        // Get access to the RAT for renaming operations
        RAT* getRAT() {
            return &RAT_Unit;
        }
        
        // Commit the head ROB entry
        bool commitHead() {
            if (isEmpty() || !reorderBuffer[0].completed)
                return false;
                
            ROBEntry entry = dequeue();
            lastCommittedEntry = entry;
            
            // If this instruction writes to a register
            if (entry.dest_reg != -1 && entry.phys_reg != -1) {
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

        // Peek at head without removing it
        ROBEntry peekHead() {
            if (reorderBuffer.size() <= 0) {
                return ROBEntry();  // Return empty entry if ROB is empty
            }
            return reorderBuffer[0];
        }

        // Mark a ROB entry as ready to commit
        void markReadyToCommit(int phys_reg) {
            for (size_t i = 0; i < reorderBuffer.size(); i++) {
                if (reorderBuffer[i].phys_reg == phys_reg && reorderBuffer[i].completed) {
                    reorderBuffer[i].ready_to_commit = true;
                    return;
                }
            }
        }
        
        // Handle branch misprediction recovery
        void recover(int rob_entry_index) {
            if (rob_entry_index < 0 || rob_entry_index >= reorderBuffer.size())
                return;  // Invalid index
                
            // Keep all entries up to and including the mispredicted one
            int entriesToKeep = rob_entry_index + 1;
            
            // Process entries that will be removed
            for (size_t i = entriesToKeep; i < reorderBuffer.size(); i++) {
                ROBEntry &entry = reorderBuffer[i];
                
                // Restore RAT to the old mapping for dest_reg
                if (entry.dest_reg != -1 && entry.old_phys_reg != -1) {
                    RAT_Unit.updateMapping(entry.dest_reg, entry.old_phys_reg);
                }
                
                // Free the newly allocated physical register
                if (entry.phys_reg != -1) {
                    freePhysReg(entry.phys_reg);
                }
            }
            
            // Remove all entries after the mispredicted one
            if (entriesToKeep < reorderBuffer.size()) {
                reorderBuffer.erase(reorderBuffer.begin() + entriesToKeep, reorderBuffer.end());
            }
        }

        // Get total size of reorder buffer
        int getSize() { 
            return reorderBuffer.size(); 
        }
        
        // Get instruction at specific index
        uint32_t getInstruction(int idx) { 
            if (idx < 0 || idx >= reorderBuffer.size())
                return 0;
                
            return reorderBuffer[idx].instruction;
        }
        
        // Debug method to print ROB state
        void printROBState() {
            std::cout << "ROB State: size=" << reorderBuffer.size() 
                      << ", maxBufferSize=" << maxBufferSize << std::endl;
        }
};
