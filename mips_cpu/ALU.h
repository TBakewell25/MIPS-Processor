#ifndef ALU_CLASS
#define ALU_CLASS
#include <vector>
#include <cstdint>
#include <iostream>
class ALU {
    private:
        int ALU_control_inputs;
    public:
        // Generate the control inputs for the ALU
        void generate_control_inputs(int ALU_op, int funct, int opcode) {
            if(!ALU_op) { // loads, stores
                ALU_control_inputs = 2; // set to add
            } 
            else if(ALU_op == 1) { // beq, bne
                ALU_control_inputs = 6; // set to subtract
            } 
            else if(ALU_op == 2) { // R-Type
                switch(funct) {
                    case 0x00: ALU_control_inputs = 3; break;               // sll
                    case 0x02: ALU_control_inputs = 4; break;               // srl
                    case 0x08: ALU_control_inputs = 2; break;               // don't care
                    case 0x20: case 0x21: ALU_control_inputs = 2; break;    // add
                    case 0x22: case 0x23: ALU_control_inputs = 6; break;    // sub
                    case 0x24: ALU_control_inputs = 0; break;               // and
                    case 0x25: ALU_control_inputs = 1; break;               // or
                    case 0x27: ALU_control_inputs = 12; break;              // nor
                    case 0x2a: case 0x2b: ALU_control_inputs = 7; break;    // slt
                    default: ALU_control_inputs = 2;
                }
            }
            else { // Other I-type
                switch(opcode) {
                    case 0x8: case 0x9: ALU_control_inputs = 2; break;      // add
                    case 0xa: case 0xb: ALU_control_inputs = 7; break;      // slt
                    case 0xc: ALU_control_inputs = 0; break;                // and
                    case 0xd: ALU_control_inputs = 1; break;                // or
                    case 0xf: ALU_control_inputs = 5; break;                // lui
                    default: ALU_control_inputs = 2;
                }
            }
        }
        
        // execute ALU operations, generate result, and set the zero control signal if necessary
        uint32_t execute(uint32_t operand_1, uint32_t operand_2, uint32_t &ALU_zero) {
            uint32_t result = 0;
            switch(ALU_control_inputs) {
                case 0: result = operand_1 & operand_2; break;
                case 1: result = operand_1 | operand_2; break;
                case 2: result = operand_1 + operand_2; break;
                case 3: result = operand_2 << operand_1; break;
                case 4: result = operand_2 >> operand_1; break;
                case 5: result = operand_2 << 16; break;
                case 6: result = operand_1 - operand_2; break;
                case 7: result = ((int)operand_1 < (int)operand_2) ? 1 : 0; break;
                case 12: result = ~(operand_1 | operand_2); break;
                default: result = operand_1 + operand_2; break;
            }
            if(!result) {
                ALU_zero = 1;
            }
            else {
                ALU_zero = 0;
            }
            return result;
        }
            
};
#endif
