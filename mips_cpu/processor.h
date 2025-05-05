#include <queue>
#include "memory.h"
#include "regfile.h"
#include "ALU.h"
#include "control.h"
#include "reservation.h"
#include "physical_reg.h"
#include "execution_unit.h"

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

                // Reservation Stations
                ReservationStation ArithmeticStations[ARITHM_STATIONS];
                ReservationStation MemoryStations[MEM_STATIONS];

                ExecutionUnit ArithUnits[4];
                ExecutionUnit MemUnits[2];

                CDBEntry CDB[MEM_STATIONS + ARITHM_STATIONS];
    
                /* Common Data Bus signals
                bool CDB_valid;
                int CDB_phys_reg;
                uint32_t CDB_value; */

                int findOpenCDB() {
                    for (int i = 0; i < MEM_STATIONS + ARITHM_STATIONS; ++i)
                       // potential logic issues
                       if (!CDB[i].valid) return i;
                    return -1;
                }

                void issueToExecutionUnits(std::vector<int>& ready_arith_rs, std::vector<int>& ready_mem_rs) {
                    // issue to arithmetic unit
                    if (!ready_arith_rs.empty()) {
                        // select first available station
                        int selected_station = ready_arith_rs[0];
                        ArithmeticStations[selected_station].executing = true;


                        uint32_t instruction = ArithmeticStations[selected_station].instruction;
                        uint32_t rs_val = ArithmeticStations[selected_station].rs_val;
                        uint32_t rt_val = ArithmeticStations[selected_station].rt_val;
        
                       // send to execution unit (will be processed in execute stage)
                       // store which reservation station this came from for writeback
                       // TODO: currently only issues to first 
                       ArithUnits[0].issueInstruction(instruction, rs_val, rt_val, selected_station);
                          
                    }
                    // issue to memory station
                    if (!ready_mem_rs.empty()) {
                        int rs_idx = ready_arith_rs[0];
                        ArithmeticStations[rs_idx].executing = true;


                        uint32_t instruction = ArithmeticStations[rs_idx].instruction;
                        uint32_t rs_val = ArithmeticStations[rs_idx].rs_val;
                        uint32_t rt_val = ArithmeticStations[rs_idx].rt_val;
        
                       // send to execution unit (will be processed in execute stage)
                       // store which reservation station this came from for writeback

                      // TODO: currently only issues to first
                       MemUnits[0].issueInstruction(instruction, rs_val, rt_val, rs_idx);
                    }
                }

    

                // create physical registers and reorder buffer
                PhysicalRegisterUnit physRegFile = PhysicalRegisterUnit(REG_COUNT);       

                // check for opening in state's instance of the ROB
                bool check_reorderBuffer() { return physRegFile.isFull(); }

                // push to state's instance of ROB
                void pushToROB(PhysicalRegisterUnit::ROBEntry item) { physRegFile.enqueue(item); }

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
                        }
                    }
                }
               
                // wakeup every station
                void wakeUpRS(int phys_reg, uint32_t value) {
                    int stations = MEM_STATIONS + ARITHM_STATIONS;
                    for (int i = 0; i < stations; ++i) {
                        bool arith = i < ARITHM_STATIONS;
                        int index = i < ARITHM_STATIONS ? i : i % ARITHM_STATIONS;

                        //ReservationStation &rs;
                        ReservationStation &rs = arith ? ArithmeticStations[index] : MemoryStations[index];

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

    public:
        Processor(Memory *mem) { regfile.pc = 0; memory = mem;}
        
        // Get PC
        uint32_t getPC() { return regfile.pc; }

        // Prints the Register File
        void printRegFile() { regfile.print(); }
        
        // Initializes the processor appropriately based on the optimization level
        void initialize(int opt_level);

        // Advances the processor to an appropriate state every cycle
        void advance(); 

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

            return new_entry;
         }    

         //STUBS
         void push_to_rs() { return; }
         int cold_start = 5;
};
