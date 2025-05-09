#include <queue>
#include "memory.h"
#include "regfile.h"
#include "ALU.h"
#include "control.h"
#include "reservation.h"
#include "physical_reg.h"
#include "execution_unit.h"
#include "load_store.h"

#ifdef enable_debug
#define debug(x) x
#else
#define debug(x) 
#endif

// define REG_COUNT 64 , defined in physical_reg.h
#define ARITHM_STATIONS 4
#define MEM_STATIONS 2

class Processor {
    private:

        int opt_level;
        ALU alu;
        control_t control;
        Memory *memory;
        Registers regfile;
    
        uint32_t processor_pc; // an additional pc to track 
        //// OOO things

        // Simple branch predictor
        struct BranchPredictor {
            static const int TABLE_SIZE = 1024;
            bool prediction_table[TABLE_SIZE];  // True = taken, False = not taken
            uint32_t history_register;          // Global history register
            
            BranchPredictor() : history_register(0) {
                // Initialize predictor - default to not taken
                for (int i = 0; i < TABLE_SIZE; i++) {
                    prediction_table[i] = false;
                }
            }
            
            // Get prediction for a branch instruction
            bool getPrediction(uint32_t pc) {
                int index = (pc ^ history_register) & (TABLE_SIZE - 1);
                return prediction_table[index];
            }
            
            // Update predictor after branch resolution
            void update(uint32_t pc, bool taken) {
                int index = (pc ^ history_register) & (TABLE_SIZE - 1);
                prediction_table[index] = taken;
                
                // Update history register
                history_register = ((history_register << 1) | (taken ? 1 : 0)) & 0xF;
            }
        };

        BranchPredictor branch_predictor;

        /* States like with pipeline
        * will theoretically be needed for:
        *    -reservation stations
        *    -ROB
        *    -RAT
        *    -pipeline queues
        *    -functional units
        */

        struct CDBEntry {
            bool valid;
            int phys_reg; // physical reg being written to, check for dependencies
            uint32_t value;
            uint32_t result;

            // invalid entry constructor
            CDBEntry() : valid(false), phys_reg(0), value(0) {}
    
            // valid entry constructor
            CDBEntry(int reg, uint32_t val) : 
                valid(true), phys_reg(reg), value(val)  {}
        };


        class State {
            public:

                // an instruction queue to get instructions from fetch to rename stages  
                std::queue<uint32_t>instruction_queue;
                //uint32_t instruction_queue; // 1 instruction at a time for now
                // Reservation Stations
                ReservationStation ArithmeticStations[ARITHM_STATIONS];
                ReservationStation MemoryStations[MEM_STATIONS];

                ExecutionUnit ArithUnits[4];
                ExecutionUnit MemUnits[2];

                CDBEntry CDB[MEM_STATIONS + ARITHM_STATIONS];
   
                MemoryOperationUnit memUnit; 

                // Branch misprediction tracking
                bool handling_misprediction;
                uint32_t recovery_pc;
                int mispredicted_rob_index;

                // copy assignment
                State& operator=(const State& other) {
                    if (this != &other) {
                        // Copy instruction queue
                        instruction_queue = other.instruction_queue;
 
                        // Copy reservation stations
                        for (int i = 0; i < ARITHM_STATIONS; i++)
                            ArithmeticStations[i] = other.ArithmeticStations[i];
                        for (int i = 0; i < MEM_STATIONS; i++)
                            MemoryStations[i] = other.MemoryStations[i];
            
                        // Copy execution units
                        for (int i = 0; i < 4; i++)
                            ArithUnits[i] = other.ArithUnits[i];
                        for (int i = 0; i < 2; i++)
                            MemUnits[i] = other.MemUnits[i];
            
                        // Copy CDB entries
                        for (int i = 0; i < MEM_STATIONS + ARITHM_STATIONS; i++)
                            CDB[i] = other.CDB[i];
            
                        // Deep copy the physical register file
                        physRegFile = other.physRegFile;
                        memUnit = other.memUnit;
                        
                        // Copy branch misprediction state
                        handling_misprediction = other.handling_misprediction;
                        recovery_pc = other.recovery_pc;
                        mispredicted_rob_index = other.mispredicted_rob_index;
                    }
                    return *this;
                }

                int findOpenCDB() {
                    for (int i = 0; i < MEM_STATIONS + ARITHM_STATIONS; ++i)
                       // potential logic issues
                       if (!CDB[i].valid) return i;
                    return -1;
                }

                void issueToExecutionUnits(std::vector<int>& ready_arith_rs, std::vector<int>& ready_mem_rs) {
                    // issue to arithmetic unit
                    if (!ready_arith_rs.empty()) {
                        int num_free_stations = ready_arith_rs.size();

                        for (int j = 0; j < num_free_stations; ++j) {
                            int selected_station = ready_arith_rs[j];
                            if (ArithmeticStations[selected_station].executing == true)
                                continue;

                            ArithmeticStations[selected_station].executing = true;

                            for (int i = 0; i < 4; ++i) {
                                if (!ArithUnits[i].checkBusy()) {
                                    uint32_t instruction = ArithmeticStations[selected_station].instruction;
                                    uint32_t rs_val = ArithmeticStations[selected_station].rs_val;
                                    uint32_t rt_val = ArithmeticStations[selected_station].rt_val;
            
                                    ArithUnits[i].issueInstruction(instruction, rs_val, rt_val, selected_station);
                                    break;
                                }
                            }
                        }
                    }

                    // issue to memory station
                    if (!ready_mem_rs.empty()) {
                        int rs_idx = ready_mem_rs[0];
                        MemoryStations[rs_idx].executing = true;
                        
                        for (int i = 0; i < 4; ++i) {
                            if (!MemUnits[i].checkBusy()) {
                                uint32_t instruction = MemoryStations[rs_idx].instruction;
                                uint32_t rs_val = MemoryStations[rs_idx].rs_val;
                                uint32_t rt_val = MemoryStations[rs_idx].rt_val;
            
                                MemUnits[i].issueInstruction(instruction, rs_val, rt_val, rs_idx);
                                break;
                            }

                        }
                    }
                }


                // create physical registers and reorder buffer
                PhysicalRegisterUnit physRegFile = PhysicalRegisterUnit(REG_COUNT);       

                // check for opening in state's instance of the ROB
                bool check_reorderBuffer() { return physRegFile.isFull(); }

                // push to state's instance of ROB
                int pushToROB(PhysicalRegisterUnit::ROBEntry item) { return physRegFile.enqueue(item); }

                int checkStationsArith() {
                    for (int i = 0; i < ARITHM_STATIONS; ++i) {
                        if (!ArithmeticStations[i].checkStation())
                            return i;
                    }
                    return -1;
                }

                int checkStationsMem() {
                    for (int i = 0; i < MEM_STATIONS; ++i) {
                        if (!MemoryStations[i].checkStation())
                            return i;
                    }
                    return -1;
                }

                // obsolete (?)
                void pushToArith(uint32_t instruction) {
                    for (int i = 0; i < ARITHM_STATIONS; ++i) {
                        if (!ArithmeticStations[i].checkStation()) {
                            ArithmeticStations[i].setInstruction(instruction);
                            ArithmeticStations[i].setInUse();
                            break; // might need another break
                        }
                    }
                }

                void pushToMem(uint32_t instruction) {
                    for (int i = 0; i < MEM_STATIONS; ++i) {
                        if (!MemoryStations[i].checkStation()) {
                            MemoryStations[i].setInstruction(instruction);
                            MemoryStations[i].setInUse();
                            break;
                        }
                    }
                }
               
                // wakeup every station
                void wakeUpRS(int phys_reg, uint32_t value) {
                    for (int i = 0; i < ARITHM_STATIONS; ++i) {
                        int index = i;
                        //ReservationStation &rs;
                        ReservationStation &rs = ArithmeticStations[index];

                        if (rs.checkStation()) {
                            // update source if waiting on this tag

                            // if the source isn't ready, and its the same as our physical register
                            if (!rs.ready_rs && rs.phys_rs == phys_reg) {
                                rs.ready_rs = true;
                                rs.rs_val = value;
                            }
                 
                            // if the rt isn't ready, and its the same as our physical
                            if (!rs.ready_rt && rs.phys_rt == phys_reg) {
                                rs.ready_rt = true;
                                rs.rt_val = value;
                            }
                        }
                    }
                    for (int i = 0; i < MEM_STATIONS; ++i) {
                        int index = i;
                        //ReservationStation &rs;
                        ReservationStation &rs = MemoryStations[index];

                        if (rs.checkStation()) {
                            // update source if waiting on this tag

                            // if the source isn't ready, and its the same as our physical register
                            if (!rs.ready_rs && rs.phys_rs == phys_reg) {
                                rs.ready_rs = true;
                                rs.rs_val = value;
                            }
                 
                            // if the rt isn't ready, and its the same as our physical
                            if (!rs.ready_rt && rs.phys_rt == phys_reg) {
                                rs.ready_rt = true;
                                rs.rt_val = value;
                            }
                        }
                    }
                }
       };
   
        State currentState = State();
        State nextState = State();

        void safeStateCopy(State& dest, const State& src) {
            // Copy instruction queue
            dest.instruction_queue = src.instruction_queue;
    
            // Copy reservation stations
            for (int i = 0; i < ARITHM_STATIONS; i++)
                dest.ArithmeticStations[i] = src.ArithmeticStations[i];
            for (int i = 0; i < MEM_STATIONS; i++)
                dest.MemoryStations[i] = src.MemoryStations[i];
    
            // Copy execution units
            for (int i = 0; i < 4; i++)
                dest.ArithUnits[i] = src.ArithUnits[i];
            for (int i = 0; i < 2; i++)
                dest.MemUnits[i] = src.MemUnits[i];
    
            // Copy CDB entries
            for (int i = 0; i < MEM_STATIONS + ARITHM_STATIONS; i++)
                dest.CDB[i] = src.CDB[i];
    
            // Deep copy the physical register file
            dest.physRegFile = src.physRegFile;
            
            // Copy branch misprediction state
            dest.handling_misprediction = src.handling_misprediction;
            dest.recovery_pc = src.recovery_pc;
            dest.mispredicted_rob_index = src.mispredicted_rob_index;
        }


        // common data bus is just a vector for simplicity, might need to do more 
        std::vector<uint32_t> CommonDataBus;

        void single_cycle_processor_advance();
        //void pipelined_processor_advance();
        void ooo_advance();

        void test_advance();
        /* OOO stages
        *  Fetch - fetch instructions into reorder
        *  Rename - register renaming through RAT
        *  Dispatch - dispatch instructions to stationSet (reservation stations)
        *  Execute - execution unit 
        *  write_back - push to the CBD to broadcast
        *  commit - commit completed, pipelined one
        */
        void testFetch(uint32_t instruction);
        void fetch();
        void rename();
        void issue();
        void execute();
        void write_back();
        void commit();
        
        // Check if an instruction is a branch
        bool isBranchInstruction(uint32_t instruction) {
            int opcode = (instruction >> 26) & 0x3f;
            return (opcode == 0x4 || opcode == 0x5 || // beq, bne
                    opcode == 0x2 || opcode == 0x3);  // j, jal
        }
        
        // Find an instruction in the ROB
        int findInstructionInROB(uint32_t instruction) {
            for (int i = 0; i < currentState.physRegFile.getSize(); i++) {
                if (currentState.physRegFile.getInstruction(i) == instruction) {
                    return i;
                }
            }
            return -1;
        }

    public:

        Processor(Memory *mem) { 
            regfile.pc = processor_pc = 0; 
            memory = mem;
            
            // Initialize branch predictor and misprediction flags
            currentState.handling_misprediction = false;
            currentState.recovery_pc = 0;
            currentState.mispredicted_rob_index = -1;
            
            nextState.handling_misprediction = false;
            nextState.recovery_pc = 0;
            nextState.mispredicted_rob_index = -1;
        }
        
        // Get PC
        uint32_t getPC() { return regfile.pc; }

        // Prints the Register File
        void printRegFile() { regfile.print(); }
        
        // Initializes the processor appropriately based on the optimization level
        void initialize(int opt_level);

        // Advances the processor to an appropriate state every cycle
        void advance(); 

        void updateState(int switchVal) {
            if (switchVal)
                safeStateCopy(currentState, nextState);
            else
                safeStateCopy(nextState, currentState);
        }

        PhysicalRegisterUnit::ROBEntry populateROBEntry(uint32_t instruction,
                                  int dest_reg,  
                                  int phys_reg,   
                                  int old_phys_reg) {

            PhysicalRegisterUnit::ROBEntry new_entry;
            new_entry.instruction = instruction;
            new_entry.dest_reg = dest_reg;
            new_entry.phys_reg = phys_reg;
            new_entry.old_phys_reg = old_phys_reg;
            new_entry.completed = false;
            new_entry.ready_to_commit = false;
            new_entry.result = 0;
            // Initialize branch fields
            new_entry.is_branch = false;
            new_entry.predicted_taken = false;
            new_entry.actual_taken = false;
            new_entry.branch_target = 0;
            new_entry.recovery_pc = 0;
            new_entry.mispredicted = false;

            return new_entry;
         }    

         //STUBS
         //void push_to_rs() { return; }
         int cold_start = 5;
         bool stall = false;
};

