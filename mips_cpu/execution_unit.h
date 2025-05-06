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

            control_t control;
            control.decode(instruction);

            int opcode = (instruction >> 26) & 0x3f;
            int shamt = (instruction >> 6) & 0x1f;
            int funct = instruction & 0x3f;
            uint32_t imm = (instruction & 0xffff);
            //int addr = instruction & 0x3ffffff;
    
            // Execution 
            alu.generate_control_inputs(control.ALU_op, funct, opcode);
   
            // Sign Extend Or Zero Extend the immediate
            // Using Arithmetic right shift in order to replicate 1 
            imm = control.zero_extend ? imm : (imm >> 15) ? 0xffff0000 | imm : imm;
    
            // Find operands for the ALU Execution
            // Operand 1 is always R[rs] -> read_data_1, except sll and srl
            // Operand 2 is immediate if ALU_src = 1, for I-type
            uint32_t operand_1 = control.shift ? shamt : op1;
            uint32_t operand_2 = control.ALU_src ? imm : op2;
            alu_zero = 0;

            result = alu.execute(operand_1, operand_2, alu_zero);
          
            in_use = false;
            return true;
        } 

        uint32_t getResult() { return result; }
        int getSourceRS() { return src_rs; }

};


