#include <queue>
#include "memory.h"
#include "regfile.h"
#include "ALU.h"
#include "control.h"
#include "reservation.h"
#include "physical_reg.h"

#ifdef enable_debug
#define debug(x) x
#else
#define debug(x) 
#endif

#define REG_COUNT 64
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

        class State {
            // Reservation Stations
            ReservationStation ArithmeticStations[ARITHM_STATIONS];
            ReservationStation MemoryStations[MEM_STATIONS];
    
    
            // Free list for physical registers
            std::queue<int> freePhysRegs;

            // create physical registers and reorder buffer
            PhysicalRegisterUnit physRegFile = PhysicalRegisterUnit(REG_COUNT);       
    
            // Common Data Bus signals
            bool CDB_valid;
            int CDB_phys_reg;
            uint32_t CDB_value;

            public:

                // check for opening in state's instance of the ROB
                bool check_reorderBuffer() { return physRegFile.isFull(); }

                // push to state's instance of ROB
                void pushToROB(ROBEntry item) { reorderBuffer.enqueue(item); }

                // push to any available arithmetic reservation stations
                void pushToArith(int index, uint32_t instruction) { ArithmeticStations[index] = instruction; }
              
                // push to any available memory stations 
                void pushToMem(int index, uint32_t instruction) { MemoryStations[index] = instruction; }

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
        };
   
        State currentState;
        State nextState;

        // an instruction queue to get instructions from fetch to rename stages  
        std::queue<uint32_t>instruction_queue;
 
        // common data bus is just a vector for simplicity, might need to do more 
        std::vector<uint32_t> CommonDataBus;

        void single_cycle_processor_advance();
        //void pipelined_processor_advance();
        void ooo_advance();

        /* OOO stages
        *  Fetch - fetch instructions into reorder
        *  Rename - register renaming through RAT
        *  Dispatch - dispatch instructions to stationSet (reservation stations)
        *  Execute - execution unit 
        *  write_back - push to the CBD to broadcast
        *  commit - commit completed, pipelined one
        */
        void fetch();
        void rename();
        void dispatch();
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

        ROBEntry populateROBEntry(uint32_t instruction,
                                  int dest_reg,  
                                  int phys_reg,   
                                  int old_phys_reg) {

            ROBEntry new_entry;
            new_entry.instruction = instruction;
            new_entry.dest_reg = dest_reg;
            new_entry.phys_reg = phys_reg;
            new_entry.old_phys_reg = old_phys_reg;
            new_entry.completed = false;
            new_entry.read_to_commit = false;
            new_entry.result = 0;

            return new_entry;
         }    
 
};
