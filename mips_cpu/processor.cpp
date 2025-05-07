#include <queue>
#include <cstdint>
#include <iostream>
#include <cstring>
#include "processor.h"

using namespace std;

#ifdef enable_debug
#define debug(x) x
#else
#define debug(x) 
#endif

#define EXEC_UNITS 6      
#define ARITHM_STATIONS 4
#define MEM_STATIONS 2 
#define COMMIT_WIDTH 2

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
        case 1: test_advance();
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
    

void Processor::testFetch(uint32_t instruction) {
    //nextState.instruction_queue.push(instruction); 
          
    // increment pc
    regfile.pc += 4;
}

   

void Processor::fetch() {
    uint32_t instruction;
    bool instruction_read;

    // fetch instructions from the cache, exit and try again next cycle on miss
    // send to instruction queue otherwise
    instruction_read = memory->access(regfile.pc, instruction, 0, 1, 0);
    if (!instruction_read) {
        stall = true;
        return;
    } else {
        nextState.instruction_queue = instruction;
        //nextState.instruction_queue.push(instruction); 
        stall = false;
    }
          
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
*       6. Dispatch
*       7. remove instruction
*/

void Processor::rename(){
    if (cold_start > 4)
        return;

    // new control for each cycle, I don't think signals need to persist
    control_t control;
    uint32_t instruction = 0;

    // 1. peek at instruction
    //if (currentState.instruction_queue.size())
     //   instruction = currentState.instruction_queue.front();
    
    if (currentState.instruction_queue)
        instruction = currentState.instruction_queue;
    else
        return;

    // 2. decode peeked value
 //TODO: move control to state class
    control.decode(instruction);

    // extract rs, rt, rd
    int rs = (instruction >> 21) & 0x1f;
    int rt = (instruction >> 16) & 0x1f;
    int rd = (instruction >> 11) & 0x1f;

    int write_reg = control.reg_dest ? rd : rt;

    // 3. check resources

    // exit if there is no reorder buffer spot available
    // logic probably needs work
    if (nextState.check_reorderBuffer())
        return; 
// need to preserve IQ 
    // check for availability of physical registers
    if (control.reg_write && nextState.physRegFile.checkFreePhys())
        return;

    // 0 arithmetic, 1 memory operation
    int instr_type = checkInstructionType(instruction); 

    // check availability of reservation stations
    int available_station_a = -1;
    int available_station_m = -1;
    switch(instr_type) {
        case 0:
            available_station_a = nextState.checkStationsArith();
            
            if (available_station_a < 0)
                return;
            break;

        case 1:
            available_station_m = nextState.checkStationsMem();
            
            if (available_station_m < 0)
                return;
            break;

        default:
            break;
    }    

    // 4. Rename

    // these represent a previous mapping and a new mapping
    // initialize as illegal values (<0)
    int new_phys_reg = -1;
    int old_phys_reg = -1; 

    // allocate new regs
    // should init these to usable vals
    if (control.reg_write) {
        new_phys_reg = nextState.physRegFile.allocatePhysReg();
        old_phys_reg = nextState.physRegFile.getMapping(write_reg);
    }
        
    // get physical registers for source operands
    int phys_rs = rs != 0 ? nextState.physRegFile.getMapping(rs) : 0;
    int phys_rt = rt != 0 ? nextState.physRegFile.getMapping(rt) : 0;

    ReservationStation *station = nullptr;
    if (instr_type == 0)
        station = &nextState.ArithmeticStations[available_station_a];
    else 
        station = &nextState.MemoryStations[available_station_m];

    // set up the reservation station
    station->instruction = instruction;    
    station->phys_rs = phys_rs;
    station->phys_rt = phys_rt;
    station->phys_rd = new_phys_reg;

    station->ready_rs = (rs == 0) || nextState.physRegFile.checkReady(phys_rs);
    station->ready_rt = (rt == 0) || nextState.physRegFile.checkReady(phys_rt);
    station->in_use = true;

    // if ready set the values
    if (station->ready_rs) 
        station->rs_val = (rs == 0) ? 0 : nextState.physRegFile.getValue(phys_rs);
    
    if (station->ready_rt)
        station->rt_val = (rt == 0) ? 0 : nextState.physRegFile.getValue(phys_rt);
    
    // map the arch write_reg to the allocated "new_phys_reg"
    if (control.reg_write && write_reg != 0)
        nextState.physRegFile.RAT_Unit.updateMapping(write_reg, new_phys_reg);


    // now that we have actually read data from the reg file we can send along instructions
    // TODO: need to be able to read from phys regs, not arch regs
    // TODO: add actual way to push instruction data, not instruction itself.
 
    // 5. Create ROBEntry
    PhysicalRegisterUnit::ROBEntry toBeSent = populateROBEntry(instruction,
                     write_reg,
                     new_phys_reg, // not implemented
                     old_phys_reg);// not implemented

    
    // 6. Dispatch HANDLED ABOVE
    /*
    switch(instr_type) {
        case 0:
            nextState.pushToArith(instruction);
            break;
        case 1:
            nextState.pushToMem(instruction);
            break;
        default:
            break;
    }
*/
    nextState.pushToROB(toBeSent);  
 

    // 7. Remove Instruction
    //currentState.instruction_queue.pop();
    //std::swap(currentState.instruction_queue, nextState.instruction_queue);    
}

/*
*   Steps for issue stage:
*       1. Monitor results
*       2. Check stations
*       3. Select instruction
*       4. Dispatch to execution unit
*/

void Processor::issue(){
    if (cold_start > 3)
        return;

    // 1. Monitor results (on CDB)
    for (int i = 0; i < EXEC_UNITS; ++i) {
        if (currentState.CDB[i].valid) {
            // wake up whatever station is waiting
            currentState.wakeUpRS(currentState.CDB[i].phys_reg, currentState.CDB[i].value);
            //currentState.CDB[i].valid = false; TODO: note, check back here
         }
    }

    // 2. Check stations
    std::vector<int> ready_arith_rs;
    std::vector<int> ready_mem_rs;

    // populate all available arithm stations
    for (int j = 0; j < ARITHM_STATIONS; ++j) {
        if (currentState.ArithmeticStations[j].in_use && 
            !currentState.ArithmeticStations[j].executing &&
            currentState.ArithmeticStations[j].ready_rs && 
            currentState.ArithmeticStations[j].ready_rt) {
            ready_arith_rs.push_back(j);
        }
    }

    // populate all available mem stations
    for (int n = 0; n < MEM_STATIONS; ++n) {
        if (currentState.MemoryStations[n].in_use && 
            !currentState.MemoryStations[n].executing &&
            currentState.MemoryStations[n].ready_rs && 
            currentState.MemoryStations[n].ready_rt) {
            ready_mem_rs.push_back(n);
        }
    }

    if (!ready_arith_rs.size() && !ready_mem_rs.size())
        return;

    // 4. Dispatch to execution unit
    nextState.issueToExecutionUnits(ready_arith_rs, ready_mem_rs);

    // push stations to next cycle
/*    for (int i= 0; i < EXEC_UNITS; ++i) {
        if (i > 3) {
            int k = i % 4;
            nextState.MemUnits[k] = ExecutionUnit(currentState.MemUnits[k]); 
        } else {
            nextState.ArithUnits[i] = ExecutionUnit(currentState.ArithUnits[i]); 
        }
    }*/

}

/*
*   Steps for execute stage:
*       1. Check for available units
*       2. Do execution on available units
*/

void Processor::execute(){
    if (cold_start > 2)
        return;

    for (int j = 0; j < 4; ++j) {
        ExecutionUnit currentUnit = currentState.ArithUnits[j];
        if (!currentUnit.checkBusy())
            continue;

        // execute the instruction
        currentUnit.execute();

        // get the result and the original RS
        uint32_t result = currentUnit.getResult();
        int source_station = currentUnit.getSourceRS(); 

        // queue of for writeback (make CDB entry)
        // TODO: handle no open cdb entry
        int free_cdb_entry = currentState.findOpenCDB();

        CDBEntry *newCDBEntry = &nextState.CDB[free_cdb_entry];

        // we need to transfer metadata from rs to new cdb entry to broadcast
        int phys_dest = currentState.ArithmeticStations[source_station].phys_rd;
        newCDBEntry->valid = true;
        newCDBEntry->phys_reg = phys_dest;
        newCDBEntry->result = result;
        //nextState.CDB[free_cdb_entry] = CDBEntry(phys_dest, result);

        nextState.physRegFile.completeROBEntry(phys_dest, result);
        

        nextState.ArithmeticStations[source_station].in_use = false;
        nextState.ArithmeticStations[source_station].executing = false;
    }

    for (int j = 0; j < 2; ++j) {
        ExecutionUnit currentUnit = currentState.MemUnits[j];
        if (!currentUnit.checkBusy())
            continue;

        // execute the instruction
        currentUnit.execute();

        // get the result and the original RS
        uint32_t result = currentUnit.getResult();
        int source_station = currentUnit.getSourceRS(); 

        // queue of for writeback (make CDB entry)
        // TODO: handle no open cdb entry
        int free_cdb_entry = currentState.findOpenCDB();

        // we need to transfer metadata from rs to new cdb entry to broadcast
        int phys_dest = currentState.MemoryStations[source_station].phys_rd;
        nextState.CDB[free_cdb_entry] = CDBEntry(phys_dest, result);

        nextState.physRegFile.completeROBEntry(phys_dest, result);

        nextState.MemoryStations[source_station].in_use = false;
        nextState.MemoryStations[source_station].executing = false;

//        markROBEntryCompleted(phys_dest, result);
    }
}

void Processor::write_back(){
    if (cold_start > 1)
        return;

    // iterate through CDB entries, update phys reg where necessary
    for (int i = 0; i < EXEC_UNITS; ++i) {
        if (currentState.CDB[i].valid) {
            int phys_reg = currentState.CDB[i].phys_reg;
            uint32_t result = currentState.CDB[i].result;
     
            // write back to physical register 
            nextState.physRegFile.writeRegister(phys_reg, result);

            // ROB entry is ready to commit, mark it
            nextState.physRegFile.markReadyToCommit(phys_reg);

            // entry is no longer valid
            nextState.CDB[i].valid = false;
        }
    }
}

void Processor::commit(){
    if (cold_start > 0)
        return;

    // bookeeping for committing
    int commit_count = 0; 
    
    while (commit_count < COMMIT_WIDTH) {
        // prep new ROB entry
        PhysicalRegisterUnit::ROBEntry head = currentState.physRegFile.peekHead();

        if (!head.completed)
            break;

        // we peeked, now we can actually fetch it
        PhysicalRegisterUnit::ROBEntry entry = currentState.physRegFile.dequeue();        

        // good ol' decode logic, nothing new
//        uint32_t instruction = entry.instruction;
//        int opcode = (instruction >> 26) & 0x3f;
//        int rs = (instruction >> 21) & 0x1f;
//        int rt = (instruction >> 16) & 0x1f;
//        int rd = (instruction >> 11) & 0x1f;
//        int funct = instruction & 0x3f;

        int dest_reg = entry.dest_reg;
    
        if (dest_reg > 0) {
            // write to arch reg
            uint32_t dummy;
            regfile.access(0, 0, dummy, dummy, dest_reg, true, entry.result);
           
            // free up phys reg in RAT and elsewhere as necessary 
            if (entry.old_phys_reg != -1) 
                nextState.physRegFile.freePhysReg(entry.old_phys_reg);
        }

        // TODO: NEED BRANCH AND LOAD/STORE
        commit_count++;
    }
}

void Processor::test_advance() {
    uint32_t instruction = 19546144; //add r8, r9, r10
    testFetch(instruction);
    rename();
    issue();
    execute();
    write_back();
    commit();

    currentState = nextState;
    cold_start--;
}           
void Processor::ooo_advance() {
    updateState(0); // push current state forward

    if (!stall) {
        commit();
        write_back();
        execute();
        issue();
        rename();
    }
    fetch();

/*    fetch();

    if (!stall) {
        rename();
        issue();
        execute();
        write_back();
        commit();
    }

*/

    updateState(1);
    cold_start--;
}
