#ifndef CONTROL_CLASS
#define CONTROL_CLASS
#include <vector>
#include <cstdint>
#include <iostream>
using namespace std;

// Control signals for the processor
struct control_t {
    bool reg_dest;           // 0 if rt, 1 if rd
    bool jump;               // 1 if jummp
    bool jump_reg;           // 1 if jr
    bool link;               // 1 if jal
    bool shift;              // 1 if sll or srl
    bool branch;             // 1 if branch
    bool bne;                // 1 if bne
    bool mem_read;           // 1 if memory needs to be read
    bool mem_to_reg;         // 1 if memory needs to written to reg
    unsigned ALU_op : 2;     // 10 for R-type, 00 for LW/SW, 01 for BEQ/BNE, 11 for others
    bool mem_write;          // 1 if needs to be written to memory
    bool halfword;           // 1 if loading/storing halfword from memory
    bool byte;               // 1 if loading/storing a byte from memory
    bool ALU_src;            // 0 if second operand is from reg_file, 1 if imm
    bool reg_write;          // 1 if need to write back to reg file
    bool zero_extend;        // 1 if immediate needs to be zero-extended
    
    void print() {      // Prints the generated contol signals
        cout << "REG_DEST: " << reg_dest << "\n";
        cout << "JUMP: " << jump << "\n";
        cout << "BRANCH: " << branch << "\n";
        cout << "MEM_READ: " << mem_read << "\n";
        cout << "MEM_TO_REG: " << mem_to_reg << "\n";
        cout << "ALU_OP: " << ALU_op << "\n";
        cout << "MEM_WRITE: " << mem_write << "\n";
        cout << "ALU_SRC: " << ALU_src << "\n";
        cout << "REG_WRITE: " << reg_write << "\n";
    }
    void reset() {
        reg_dest = 0;         
        jump = 0;             
        jump_reg = 0;         
        link = 0;             
        shift = 0;            
        branch = 0;           
        bne = 0;              
        mem_read = 0;         
        mem_to_reg = 0;       
        ALU_op = 0;           
        mem_write = 0;         
        halfword = 0;          
        byte = 0;              
        ALU_src = 0;           
        reg_write = 0;          
        zero_extend = 0;        

    }
    // Decode instructions into control signals
    void decode(uint32_t instruction) {
        reset();
        // Get OPCODE
        int opcode = (instruction >> 26) & 0x3f;
        
        if (!opcode) { // R-Type Instructions
            reg_dest = 1;
            reg_write = 1;
            ALU_op = 2;

            // Special Case: jr
            if ((instruction & 0x3f) == 0x08) {
                reg_dest = 0;
                reg_write = 0;
                ALU_op = 0;
                jump = 1;
                jump_reg = 1;
            }

            // Special Case: shift
            if ((instruction & 0x3f) == 0x0 || (instruction & 0x3f) == 0x2) {
                shift = 1;
            }
        } // end R-Type
        
        else if (opcode == 0x2 || opcode == 0x3) { // J-Type Instructions
            jump = 1;
            
            // Special Case: jal
            if (opcode == 0x3) {
                link = 1;
                reg_write = 1;
            }
        } // end J-Type

        else { // I-Type Instructions
            ALU_src = 1; // choose immediate for alu source
            
            if (opcode == 0x4 || opcode == 0x5) { // beq, bne
                branch = 1;
                ALU_op = 1;
                ALU_src = 0;
                
                // Special Case: bne
                if (opcode == 0x5) {
                    bne = 1;
                }
            } // end beq, bne
            
            else if (opcode == 0x2b || opcode == 0x28 || opcode == 0x29) { // Stores
                mem_write = 1;

                // Special Case: sb, sh
                if (opcode == 0x28) { // sb
                    byte = 1;
                }
                else if (opcode == 0x29) { // sh
                    halfword = 1;
                }
            } // end stores

            else if ((opcode >= 0x23 && opcode <= 0x25) || opcode == 0x30) { // Loads
                mem_read = 1;
                mem_to_reg = 1;
                reg_write = 1;
                
                // Special Case: lbu, lhu
                if (opcode == 0x24) { // lbu
                    byte = 1;
                }
                else if (opcode == 0x25) { // lhu
                    halfword = 1;
                }
            } // end loads

            else { // Catch all I-type Instrcutions
                reg_write = 1;
                ALU_op = 3; 
                // Special Case: ori, andi
                if (opcode == 0xc || opcode == 0xd) {
                    zero_extend = 1;
                }
            }

        }// end I-Type
    }
};




#endif
