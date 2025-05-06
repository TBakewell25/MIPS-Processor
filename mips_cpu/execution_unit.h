#include "ALU.h"

class ExecutionUnit {
    protected:
        bool in_use; 

        uint32_t instruction;
        uint32_t op1;
        uint32_t op2;
        uint32_t alu_zero;
        uint32_t ALU_op;
        uint32_t result; 

        int src_rs; // source rs

        ALU alu;
    public:
        ExecutionUnit() : in_use(false) {}

        bool checkBusy() { return in_use; }

        virtual void issueInstruction(uint32_t instr, uint32_t oper1, uint32_t oper2, int station_idx) {
            instruction = instr;
            op1 = oper1;
            op2 = oper2;
            src_rs = station_idx;
            in_use = true;
        }

        bool execute() {
            if (!in_use) return false;

            int opcode = (instruction >> 26) & 0x3f;
            int funct = instruction & 0x3f;
            
            alu.generate_control_inputs(ALU_op, funct, opcode);

            // TODO: fix alu_zero logic
            uint32_t alu_zero = 0;
            result = alu.execute(op1, op2, alu_zero);
          
            in_use = false;
            return true;
        } 

        uint32_t getResult() { return result; }
        int getSourceRS() { return src_rs; }

};


