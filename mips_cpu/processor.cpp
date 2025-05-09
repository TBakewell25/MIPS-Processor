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
    
    // Initialize branch predictor
    for (int i = 0; i < branch_predictor.TABLE_SIZE; i++) {
        branch_predictor.prediction_table[i] = false;
    }
    branch_predictor.history_register = 0;
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
        return 2;

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
        if (instruction) {
            if (instruction == 2888040456) {
               cout << "found instruction at " << regfile.pc << "\n";
            }
            //nextState.instruction_queue = instruction;
            nextState.instruction_queue.push(instruction); 
        }
        stall = false;
    }

    if (instruction)
        cout << "Just fetched instruction " << instruction << " at PC: " << regfile.pc << "\n" << endl;
          
    // increment pc
    regfile.pc += 4;
    //processor_pc += 4;
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
    
    if (currentState.instruction_queue.size())
        instruction = currentState.instruction_queue.front();
    else
        return;

    // 2. decode peeked value
 //TODO: move control to state class
    control.decode(instruction);

    // extract rs, rt, rd
    int rs = (instruction >> 21) & 0x1f;
    int rt = (instruction >> 16) & 0x1f;
    int rd = (instruction >> 11) & 0x1f;
    int shamt = (instruction >> 6) & 0x1f;
    int funct = instruction & 0x3f;
    uint32_t imm = (instruction & 0xffff);
    int addr = instruction & 0x3ffffff;

    int write_reg = control.reg_dest ? rd : rt;

    // 3. check resources

    // exit if there is no reorder buffer spot available
    // logic probably needs work
    if (nextState.check_reorderBuffer()) {
        //nextState.instruction_queue = instruction;
        return; 
    }

    // need to preserve IQ 
    // check for availability of physical registers
    if (control.reg_write && nextState.physRegFile.checkFreePhys()){
        //nextState.instruction_queue = instruction;
        return;
    }

    // 0 arithmetic, 1 memory operation
    int instr_type = checkInstructionType(instruction); 
    int size = control.byte ? 1 : control.halfword ? 2 : 4; // for memory ops

    // check availability of reservation stations
    int available_station_a = -1;
    int available_station_m = -1;
    switch(instr_type) {
        case 0:
            available_station_a = nextState.checkStationsArith();
            
            if (available_station_a < 0) {
           //     nextState.instruction_queue = instruction;
                return;
            }
            break;

        case 1:
            available_station_m = nextState.checkStationsMem();
            
            if (available_station_m < 0) {
         //       nextState.instruction_queue = instruction;
                return;
            }
            break;
        case 2:
            available_station_m = nextState.checkStationsMem();
            
            if (available_station_m < 0) {
          //      nextState.instruction_queue = instruction;
                return;
            }
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

    // Branch handling
    if (isBranchInstruction(instruction)) {
        // This is a branch instruction
        int opcode = (instruction >> 26) & 0x3f;
        station->is_branch = true;
        
        // Calculate the branch target for immediate branches
        int16_t offset = static_cast<int16_t>(imm);
        uint32_t branch_target = regfile.pc + (offset << 2);
        station->branch_target = branch_target;
        station->next_pc = regfile.pc;  // Store current PC for recovery
        
        // Get prediction
        bool prediction = branch_predictor.getPrediction(regfile.pc - 4);
        station->predicted_taken = prediction;
        
        // Create ROB entry with branch info
        PhysicalRegisterUnit::ROBEntry toBeSent = populateROBEntry(instruction,
                        write_reg,
                        new_phys_reg,
                        old_phys_reg);
        
        toBeSent.is_branch = true;
        toBeSent.predicted_taken = prediction;
        toBeSent.branch_target = branch_target;
        toBeSent.recovery_pc = regfile.pc;
        toBeSent.mispredicted = false;
        
        // Push to ROB and get index
        int ROB_index = nextState.pushToROB(toBeSent);
        
        // If we predict taken, update PC to branch target
        if (prediction) {
            regfile.pc = branch_target;
            cout << "Branch at PC: 0x" << hex << (regfile.pc - 4) << dec 
                 << " predicted taken to target: 0x" << hex << branch_target << dec << endl;
        } else {
            cout << "Branch at PC: 0x" << hex << (regfile.pc - 4) << dec 
                 << " predicted not taken" << endl;
        }
    } else {
        // now that we have actually read data from the reg file we can send along instructions
        // TODO: need to be able to read from phys regs, not arch regs
        // TODO: add actual way to push instruction data, not instruction itself.
     
        // 5. Create ROBEntry
        PhysicalRegisterUnit::ROBEntry toBeSent = populateROBEntry(instruction,
                         write_reg,
                         new_phys_reg, // not implemented
                         old_phys_reg);// not implemented
    
        // 6. Dispatch HANDLED ABOVE
        int ROB_index = nextState.pushToROB(toBeSent);  
     
        if (instr_type == 1) { // store
            int store_idx = nextState.memUnit.addStore(ROB_index, size, instruction);        
            station->mem_op_index = store_idx;
            station->is_store = true;
            station->is_load = false;
        } else if (instr_type == 2) { // load
            int load_idx = nextState.memUnit.addLoad(ROB_index, new_phys_reg, size, instruction);
            station->mem_op_index = load_idx;
            station->is_load = true;
            station->is_store = false;
        }
    }
    
    // 7. Remove Instruction
    currentState.instruction_queue.pop();
    nextState.instruction_queue.pop();
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
            nextState.wakeUpRS(currentState.CDB[i].phys_reg, currentState.CDB[i].result);
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

        // Check if this is a branch instruction
        if (currentState.ArithmeticStations[source_station].is_branch) {
            ReservationStation &station = currentState.ArithmeticStations[source_station];
            
            // Calculate branch outcome
            bool actual_taken = false;
            int opcode = (station.instruction >> 26) & 0x3f;
            
            if (opcode == 0x4) {  // beq
                actual_taken = (station.rs_val == station.rt_val);
            } else if (opcode == 0x5) {  // bne
                actual_taken = (station.rs_val != station.rt_val);
            } else if (opcode == 0x2 || opcode == 0x3) {  // j, jal are always taken
                actual_taken = true;
            }
            
            // Update the ROB entry
            int rob_index = nextState.physRegFile.searchByInstruction(station.instruction);
            if (rob_index >= 0) {
                nextState.physRegFile.reorderBuffer[rob_index].actual_taken = actual_taken;
                
                // Check for misprediction
                bool misprediction = (actual_taken != station.predicted_taken);
                nextState.physRegFile.reorderBuffer[rob_index].mispredicted = misprediction;
                
                // If mispredicted, set flag for recovery during commit
                if (misprediction) {
                    nextState.handling_misprediction = true;
                    nextState.mispredicted_rob_index = rob_index;
                    
                    // Set recovery PC
                    if (actual_taken) {
                        nextState.recovery_pc = station.branch_target;
                    } else {
                        nextState.recovery_pc = station.next_pc;
                    }
                    
                    // Update branch predictor
                    branch_predictor.update(station.next_pc - 4, actual_taken);
                    
                    std::cout << "Branch misprediction detected at PC: 0x" << std::hex 
                              << station.next_pc - 4 << std::dec << std::endl;
                    std::cout << "  Predicted: " << (station.predicted_taken ? "taken" : "not taken") 
                              << ", Actual: " << (actual_taken ? "taken" : "not taken") << std::endl;
                    std::cout << "  Recovery PC: 0x" << std::hex << nextState.recovery_pc 
                              << std::dec << std::endl;
                }
            }
        }

        // queue of for writeback (make CDB entry)
        // TODO: handle no open cdb entry
        int free_cdb_entry = nextState.findOpenCDB();
        if (free_cdb_entry >= 0) {
            int phys_dest = currentState.ArithmeticStations[source_station].phys_rd;
            nextState.CDB[free_cdb_entry].valid = true;
            nextState.CDB[free_cdb_entry].phys_reg = phys_dest;
            nextState.CDB[free_cdb_entry].result = result;
    
            nextState.physRegFile.completeROBEntry(phys_dest, result);
    
            nextState.ArithmeticStations[source_station].in_use = false;
            nextState.ArithmeticStations[source_station].executing = false;
            nextState.ArithUnits[j].setOpen();
        }
/*
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
        nextState.ArithUnits[j].setOpen(); */
    }


    for (int j = 0; j < 2; ++j) {
        ExecutionUnit currentUnit = currentState.MemUnits[j];
        if (!currentUnit.checkBusy())
            continue;

        // execute the instruction
        currentUnit.execute();

        // get the result and the original RS
        uint32_t address = currentUnit.getResult();
        int source_station = currentUnit.getSourceRS(); 

          
        //int mem_op_idx = currentState.MemoryStations[source_station].mem_op_index; // get the index in l/s queue
        //nextState.memOpQueue.markOpReady(mem_op_idx, calculated_address); // mark ready
        if (currentState.MemoryStations[source_station].is_store) { // store
            int mem_op_idx_store = nextState.memUnit.findByInstructionStore(currentUnit.instruction);
            int rob_idx = nextState.physRegFile.searchByInstruction(currentUnit.instruction);
            //int rob_idx = nextState.memUnit.store_queue[mem_op_idx_store].rob_index;

            nextState.memUnit.updateStore(mem_op_idx_store, address, currentUnit.op2);
        
            // Mark the ROB entry as completed (it will be committed later)
            nextState.physRegFile.completeByIndex(rob_idx, address);
        
            // Free the reservation station and execution unit
            nextState.MemoryStations[source_station].in_use = false;
            nextState.MemoryStations[source_station].executing = false;
            nextState.MemUnits[j].setOpen();

        } else if (currentState.MemoryStations[source_station].is_load) {
            int mem_op_idx_load = nextState.memUnit.findByInstructionLoad(currentUnit.instruction);
            int rob_idx = nextState.physRegFile.searchByInstruction(currentUnit.instruction);
            //int rob_idx = nextState.memUnit.load_queue[mem_op_idx_load].rob_index;

            nextState.memUnit.updateLoad(mem_op_idx_load, address);
            uint32_t result;

            bool success = nextState.memUnit.executeLoad(mem_op_idx_load, memory, result);
            if (success) {
                // Send the result to the CDB
                int free_cdb_entry = nextState.findOpenCDB();
                if (free_cdb_entry >= 0) {
                    int phys_dest = currentState.MemoryStations[source_station].phys_rd;
                
                    nextState.CDB[free_cdb_entry].valid = true;
                    nextState.CDB[free_cdb_entry].phys_reg = phys_dest;
                    nextState.CDB[free_cdb_entry].result = result;
                
                    // Mark the ROB entry as completed
                    nextState.physRegFile.completeROBEntry(phys_dest, result);
                
                    // Free the reservation station and execution unit
                    nextState.MemoryStations[source_station].in_use = false;
                    nextState.MemoryStations[source_station].executing = false;
                    nextState.MemUnits[j].setOpen();
                } 
            } else {
                nextState.MemUnits[j] = currentUnit; // miss, push to next cycle
                nextState.MemUnits[j].in_use = true;
                //nextState.MemUnits[j].executing = true;
            }
        }
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

    // Handle branch misprediction recovery
    if (currentState.handling_misprediction) {
        // Recover from misprediction
        std::cout << "Recovering from branch misprediction" << std::endl;
        std::cout << "  Setting PC to: 0x" << std::hex << currentState.recovery_pc 
                << std::dec << std::endl;
        std::cout << "  Flushing instructions after ROB index: " 
                << currentState.mispredicted_rob_index << std::endl;
        
        // Reset PC to correct path
        regfile.pc = currentState.recovery_pc - 4;
        
        // Clear instruction queue
        nextState.instruction_queue = std::queue<uint32_t>();
        
        // Use the existing recover method to restore the RAT and free registers
        nextState.physRegFile.recover(currentState.mispredicted_rob_index);
        
        // Clear any executing instructions in execution units
        for (int i = 0; i < 4; i++) {
            nextState.ArithUnits[i].setOpen();
        }
        for (int i = 0; i < 2; i++) {
            nextState.MemUnits[i].setOpen();
        }
        
        // Clear reservation stations that are younger than mispredicted instruction
        for (int i = 0; i < ARITHM_STATIONS; i++) {
            int rob_idx = findInstructionInROB(nextState.ArithmeticStations[i].instruction);
            if (rob_idx > currentState.mispredicted_rob_index || rob_idx == -1) {
                nextState.ArithmeticStations[i].in_use = false;
                nextState.ArithmeticStations[i].executing = false;
            }
        }
        for (int i = 0; i < MEM_STATIONS; i++) {
            int rob_idx = findInstructionInROB(nextState.MemoryStations[i].instruction);
            if (rob_idx > currentState.mispredicted_rob_index || rob_idx == -1) {
                nextState.MemoryStations[i].in_use = false;
                nextState.MemoryStations[i].executing = false;
            }
        }
        
        // Remove memory operations from load/store queues that are younger than mispredicted instruction
        for (auto it = nextState.memUnit.load_queue.begin(); it != nextState.memUnit.load_queue.end();) {
            if (it->rob_index > currentState.mispredicted_rob_index) {
                it = nextState.memUnit.load_queue.erase(it);
            } else {
                ++it;
            }
        }
        for (auto it = nextState.memUnit.store_queue.begin(); it != nextState.memUnit.store_queue.end();) {
            if (it->rob_index > currentState.mispredicted_rob_index) {
                it = nextState.memUnit.store_queue.erase(it);
            } else {
                ++it;
            }
        }
        
        // Clear CDB entries
        for (int i = 0; i < MEM_STATIONS + ARITHM_STATIONS; i++) {
            nextState.CDB[i].valid = false;
        }
        
        // Reset misprediction handling flag
        nextState.handling_misprediction = false;
        
        return; // Skip normal commit for this cycle
    }

    // bookeeping for committing
    int commit_count = 0; 
    
    while (commit_count < COMMIT_WIDTH) {
        // prep new ROB entry
        PhysicalRegisterUnit::ROBEntry head = currentState.physRegFile.peekHead();

        if (!head.completed)
            break;
        
        int instr_type = checkInstructionType(head.instruction); 

        if (instr_type == 1) { //store
            nextState.memUnit.executeStore(0, memory);
            bool committed = nextState.memUnit.commitStore(head.rob_index, memory); // do the store
            if (!committed) { // fail
                return;     
            }
        }

        // we peeked, now we can actually fetch it
        // need to dump both next state and current state since its in both
        // both are the same, but we'll use currentState
        PhysicalRegisterUnit::ROBEntry entry = currentState.physRegFile.dequeue();        
        nextState.physRegFile.dequeue();      // ignore value  

        int dest_reg = entry.dest_reg;
        uint32_t instruction = entry.instruction;
                
        if (instr_type == 2) {
            // For loads, just remove from the queue
            nextState.memUnit.removeLoad(entry.rob_index);
        } 

        if (dest_reg > 0 && instr_type != 1) {
            // write to arch reg
            uint32_t dummy;
            if (!isBranchInstruction(entry.instruction))
                regfile.access(0, 0, dummy, dummy, dest_reg, true, entry.result);
          
             
            // free up phys reg in RAT and elsewhere as necessary 
            if (entry.old_phys_reg != -1) 
                nextState.physRegFile.freePhysReg(entry.old_phys_reg);
        }

        // Handle committing of branches
        if (entry.is_branch) {
            cout << "Committed branch at ROB index " << entry.rob_index 
                 << ", mispredicted: " << (entry.mispredicted ? "yes" : "no") << endl;

            // If this was a correctly predicted branch, we can update the predictor
            // (Mispredicted branches already updated the predictor in the execute stage)
            if (!entry.mispredicted) {
                branch_predictor.update(entry.recovery_pc - 4, entry.actual_taken);
            }
        }

        commit_count++;
        cout << "Committed instruction, processor_pc is " << processor_pc << "\n" << endl;
        processor_pc+=4;
//        regfile.pc += 4;
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

    updateState(1);
    cold_start--;
}
