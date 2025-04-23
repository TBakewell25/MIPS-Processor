#ifndef REG_FILE
#define REG_FILE
#include <vector>
#include <cstdint>
#include <iostream>

struct PhysReg {
    int32_t value;
    bool ready;
};

class Registers {

    private:
        std::vector<PhysReg> R;
        std::vector<int> regmap;
        std::vector<int> rename_pool;
    public:
        uint32_t pc;
        Registers() {
            R.resize(32);
            for (int i = 0; i < 32; i++) {
                R[i].value = 0;
                R[i].ready = true;
            }
        }
        // read_reg_1, read_reg_2 are register numbers from which the data should be read
        // read_data_1, read_data_2 are variables into which the data is read. These are passed by reference
        // write_reg is the register number to which the data needs to be written
        // write is a flag which incidates whether it is a write access or not
        // write_data is the data which needs to be written to write_reg
        void access(int read_reg_1, int read_reg_2, uint32_t &read_data_1, uint32_t &read_data_2,
                int write_reg, bool write, uint32_t write_data) {
            read_data_1 = R[read_reg_1].value;
            read_data_2 = R[read_reg_2].value;
            if (write) {
                R[write_reg].value = write_data;
                R[write_reg].ready = true;
            }
        }

        bool ready(int reg) {
            return R[reg].ready;
        }

        // Prints the contents of all the registers
        void print() {
            for(int i = 0; i < 32; ++i) {
                std::cout << std::dec << "R[" << i << "]: " << R[i].value << "\n";
            }
        }
        // Prints the contents of the register specified by reg 
        // This function should help you debug your code
        void print(int reg) {
            std::cout << "R[" << reg << "]: " << R[reg].value << "\n";
        }
            
};
#endif
