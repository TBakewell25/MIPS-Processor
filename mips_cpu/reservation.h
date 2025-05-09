#include <iostream>
#include "control.h"

#ifdef enable_debug
#define debug(x) x
#else
#define debug(x) 
#endif

class ReservationStation {

    public:
        uint32_t instruction;
        uint32_t opcode;

        int phys_rt, phys_rs; // mappings for physical rt and rd
        bool ready_rt, ready_rs; // ready bits for above
        uint32_t rt_val, rs_val; // actual data goes here

        uint32_t phys_rd; // mapping for destination
        uint32_t ROB_index; // index of entry in ROB
           
        int shamt;
        int funct;
        uint32_t imm;
        int addr;

        control_t control;
        bool in_use;
        bool executing;

        int mem_op_index; // location in memory queue, only for load/store
        bool is_load;
        bool is_store;
        
        // Branch prediction fields
        bool is_branch;
        bool predicted_taken;
        uint32_t branch_target;
        uint32_t next_pc;     // PC + 4, needed for recovery

        ReservationStation() {
            in_use = false;
            executing = false;
            is_branch = false;
            predicted_taken = false;
            branch_target = 0;
            next_pc = 0;
        };

        // Check if CandidateStation is occupied
        bool checkStation() { return in_use; }

        void setInUse() { in_use = true; }

        void setInstruction(uint32_t instr) { instruction = instr; }
       
        // check if RS is present in station 
        //bool peekRS() { return (ResStation == RS1 || ResStation == RS2); }

        // write values to station
        //void writeValues(uint32_t Value_1, uint32_t Value_2) { Val1 = Value_1; Val2 = Value_2; } 
 
        // write RS values
        //void writeRS(uint32_t Stat1, uint32_t Stat2) { RS1 = Stat1; RS2 = Stat2; }

};
