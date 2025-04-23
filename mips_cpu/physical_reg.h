#include <vector>

#ifdef enable_debug
#define debug(x) x
#else
#define debug(x) 
#endif

#define ARCH_REG 32

/*
* A class to handle register allocation
* for bothe phsyical and architectural
* registers.
*
* Need to implement renaming, translation
*/

class PhysicalRegisterUnit {
    private: 
        // nested class to represent Register Alias Table
        class RAT {
            struct rat_line {
                uint32_t arch_reg; // corresponding architectural register
                std::vector<uint32_t>RAT; // mappings of arch registers to physical registers
                bool in_use; 
            };

            // define a rat with ARCH_REG (usually 32) entries
            rat_line rat[ARCH_REG];
            
            public:

                RAT(int reg_count) {
                    for (int i = 0; i < reg_count; i++) {
                        rat[i].arch_reg = i;
                        rat[i].in_use = false;
                    }
                }

                bool ratIsUsed(uint32_t register_number) { return rat[register_number].in_use; }

                void insertRAT(uint32_t register_number, uint32_t new_mapping) { rat[register_number].RAT.push_back(new_mapping); }
 
                uint32_t popRAT(uint32_t register_number) { return rat[register_number].RAT.back(); }
        };

        RAT RAT_Unit = RAT(32);

        std::vector<uint32_t> physTable; // a table holding n many phsyical registers (32-bit each) 
        
        std::vector<uint32_t> reorderBuffer; // the instruction reorderbuffer, a circular buffer implemented below
        
        ///values for circ buffer implementation
        int head, tail, 
            count, capacity;

        bool is_full;

    public:
        
        PhysicalRegisterUnit(int reg_count) : physTable(reg_count, 0) {}
 
        // Enqueue operation
        // 0 on success, -1 on error
        int enqueue(uint32_t value) {
            if (is_full) 
                return -1;

            reorderBuffer[tail] = value;
            tail = (tail + 1) % capacity;
        
            // Check if reorder buffer is now full
            if (head == tail) {
                is_full = true;
            }

            return 0;
        }

        // Dequeue operation
        uint32_t dequeue() {
            if (isEmpty())
                return 0xFFFFFFFF; 

            uint32_t value = reorderBuffer[head];
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
        size_t size() const {
            if (is_full) 
                return capacity;
            
            return (tail >= head) ? (tail - head) : (capacity - head + tail);
        }
}; 
