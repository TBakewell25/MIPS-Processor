#include <queue>
#include <cstdint>
#include <iostream>
#include "processor.h"
using namespace std;

#ifdef enable_debug
#define debug(x) x
#else
#define debug(x) 
#endif

       

void Processor::initialize(int level) {
    // Initialize Control
    control = {.reg_dest = 0, 
               .jump = 0,
               .jump_reg = 0,
               .link = 0,
               .shift = 0,
               .branch = 0,
               .bne = 0,
               .mem_read = 0,
               .mem_to_reg = 0,
               .ALU_op = 0,
               .mem_write = 0,
               .halfword = 0,
               .byte = 0,
               .ALU_src = 0,
               .reg_write = 0,
               .zero_extend = 0};
   
    opt_level = level;
    // Optimization level-specific initialization
}

void Processor::advance() {
    switch (opt_level) {
        case 0: single_cycle_processor_advance();
                break;
        case 2: ooo_advance();
                break;
        default: break;
    }
}

void Processor::single_cycle_processor_advance() {
    // fetch
    uint32_t instruction;
    memory->access(regfile.pc, instruction, 0, 1, 0);
    //DEBUG(cout << "\nPC: 0x" << std::hex << regfile.pc << std::dec << "\n");
    // increment pc
    regfile.pc += 4;
    
    // decode into contol signals
    control.decode(instruction);
//    DEBUG(control.print());

    // extract rs, rt, rd, imm, funct 
    int opcode = (instruction >> 26) & 0x3f;
    int rs = (instruction >> 21) & 0x1f;
    int rt = (instruction >> 16) & 0x1f;
    int rd = (instruction >> 11) & 0x1f;
    int shamt = (instruction >> 6) & 0x1f;
    int funct = instruction & 0x3f;
    uint32_t imm = (instruction & 0xffff);
    int addr = instruction & 0x3ffffff;
    // Variables to read data into
    uint32_t read_data_1 = 0;
    uint32_t read_data_2 = 0;
    
    // Read from reg file
    regfile.access(rs, rt, read_data_1, read_data_2, 0, 0, 0);
    
    // Execution 
    alu.generate_control_inputs(control.ALU_op, funct, opcode);
   
    // Sign Extend Or Zero Extend the immediate
    // Using Arithmetic right shift in order to replicate 1 
    imm = control.zero_extend ? imm : (imm >> 15) ? 0xffff0000 | imm : imm;
    
    // Find operands for the ALU Execution
    // Operand 1 is always R[rs] -> read_data_1, except sll and srl
    // Operand 2 is immediate if ALU_src = 1, for I-type
    uint32_t operand_1 = control.shift ? shamt : read_data_1;
    uint32_t operand_2 = control.ALU_src ? imm : read_data_2;
    uint32_t alu_zero = 0;

    uint32_t alu_result = alu.execute(operand_1, operand_2, alu_zero);
    
    
    uint32_t read_data_mem = 0;
    uint32_t write_data_mem = 0;

    // Memory
    // First read no matter whether it is a load or a store
    memory->access(alu_result, read_data_mem, 0, control.mem_read | control.mem_write, 0);
    // Stores: sb or sh mask and preserve original leftmost bits
    write_data_mem = control.halfword ? (read_data_mem & 0xffff0000) | (read_data_2 & 0xffff) : 
                    control.byte ? (read_data_mem & 0xffffff00) | (read_data_2 & 0xff): read_data_2;
    // Write to memory only if mem_write is 1, i.e store
    memory->access(alu_result, read_data_mem, write_data_mem, control.mem_read, control.mem_write);
    // Loads: lbu or lhu modify read data by masking
    read_data_mem &= control.halfword ? 0xffff : control.byte ? 0xff : 0xffffffff;

    int write_reg = control.link ? 31 : control.reg_dest ? rd : rt;

    uint32_t write_data = control.link ? regfile.pc+8 : control.mem_to_reg ? read_data_mem : alu_result;  

    // Write Back
    regfile.access(0, 0, read_data_2, read_data_2, write_reg, control.reg_write, write_data);
    
    // Update PC
    regfile.pc += (control.branch && !control.bne && alu_zero) || (control.bne && !alu_zero) ? imm << 2 : 0; 
    regfile.pc = control.jump_reg ? read_data_1 : control.jump ? (regfile.pc & 0xf0000000) & (addr << 2): regfile.pc;
}

// check for arithmetic or mem to dispatch to RS
// 0 if ar, 1 if mem
int checkInstructionType(uint32_t instruction) {
    int opcode = (instruction >> 26) & 0x3f;
 
    if (opcode == 0x2b || opcode == 0x28 || opcode == 0x29) // stores
        return 1;

    if ((opcode >= 0x23 && opcode <= 0x25) || opcode == 0x30) // Loads
        return 1;

    // base case, arithmetic
    return 0;
}
    
    

void Processor::fetch() {
    uint32_t instruction;
    bool instruction_read;

    // fetch instructions from the cache, exit and try again next cycle on miss
    // send to instruction queue otherwise
    instruction_read = memory->access(regfile.pc, instruction, 0, 1, 0);
    if (!instruction_read)
        return;
    else 
       instruction_queue.push(instruction); 
          
    // increment pc
    regfile.pc += 4;
}

/*
*   Steps for rename stage:
*       1. peek at instruction in IQ
*       2. decode the peeked value (do not fetch)
*       3. check resources (ROB, RS, RAT)
*       4. Rename
*       5. Create ROBEntry
*       6. Push to RS
*       7. remove instruction
*/

void Processor::rename(){
    // 1. peek at instruction
    // TODO: currenty pops, needs to peek
    uint32_t instruction = instruction_queue.pop_back();

    // 2. decode peeked value
    control_t control;
    control.decode(instruction);

    // extract rs, rt, rd, imm, funct 
    int opcode = (instruction >> 26) & 0x3f;
    int rs = (instruction >> 21) & 0x1f;
    int rt = (instruction >> 16) & 0x1f;
    int rd = (instruction >> 11) & 0x1f;
    int shamt = (instruction >> 6) & 0x1f;
    int funct = instruction & 0x3f;
    uint32_t imm = (instruction & 0xffff);
    int addr = instruction & 0x3ffffff;

    // 3. check resources
    // 0 arithmetic, 1 memory operation
    int instr_type = getInstructionType(instruction); 

    // exit if there is no reorder buffer spot available
    // logic probably needs work
    if (!nextState.checkReorderBuffer())
        return;  

    // check if we need to send this somewhere
    // then check mappings to see if we can 
    if (control.reg_dest) {
        bool rat_availability = checkRAT(rd);
        if (rat_availability)
            pushToRat(rd, instruction);
        else
            return;
    }

    switch(instr_type) {
        case(0):
            int available_station_a = nextState.checkStationArith();
            
            if (available_station_a >= 0)
                nextState.pushToArith(available_station_a, instruction);        
            else
                return;
            break;

        case(1):
            int available = nextState.checkStationMem();
            
            if (available >= 0)
                nextState.pushToMem(available_station_a, instruction);
            else
                return;
            break;
    }    

    // 4. Rename

    // Variables to read data into
    uint32_t read_data_1 = 0;
    uint32_t read_data_2 = 0;
    
    // Read from reg file
    regfile.access(rs, rt, read_data_1, read_data_2, 0, 0, 0);

    // 5. Create ROBEntry
    ROBEntry toBeSent = populateROBEntry(instruction,
                     rd,
                     phys_reg, // not implemented
                     old_phys_reg) // not implemented
    pushToROB(toBeSent);  
 
    // 6. done above, needs to be moved down here

    // 7.
    instruction_queue.pop();
}
void Processor::dispatch(){}
void Processor::execute(){}
void Processor::write_back(){}
void Processor::commit(){}
 
void Processor::ooo_advance() {
    fetch();
    rename();
    dispatch();
    execute();
    write_back();
    commit();

    currentState = nextState;
}
