#include <vector>

#ifdef enable_debug
#define debug(x) x
#else
#define debug(x) 
#endif

/*
* A class to handle register allocation
* for bothe phsyical and architectural
* registers.
*
* Need to implement renaming, translation
*/

class PhysicalRegisterUnit {
    private: 

        std::vector<uint32_t> physTable; // a table holding n many phsyical registers (32-bit each) 
        
        std::vector<uint32_t> reorderBuffer; // the instruction reorderbuffer, a circular buffer implemented below
        
        ///values for circ buffer implementation
        int head, tail, 
            count, capacity;

        bool is_full;

    public:
        
        PhysicalRegisterUnit(int reg_count) : physTable(reg_count, 0) {}
 
        // Enqueue operation
        int enqueue(uint32_t value) {
            if (is_full) 
                return -1;

            reorderBuffer[tail] = value;
            tail = (tail + 1) % capacity;
        
            // Check if buffer is now full
            if (head == tail) {
                is_full = true;
            }
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

        // Check if buffer is empty
        bool isEmpty() const {
            return (head == tail) && !is_full;
        }

        // Check if buffer is full
        bool isFull() const { return is_full; }

        // Get current size
        size_t size() const {
            if (is_full) 
                return capacity;
            
            return (tail >= head) ? (tail - head) : (capacity - head + tail);
        }
}; 
