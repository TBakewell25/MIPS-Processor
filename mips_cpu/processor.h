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

        struct State {
            // Reservation Stations
            ReservationStation ArithmeticStations[ARITHM_STATIONS];
            ReservationStation MemoryStations[MEM_STATIONS];
    
            // Register Alias Table (RAT) - maps architectural to physical registers
            int RAT[32];  // For 32 MIPS registers
    
            // Free list for physical registers
            std::queue<int> freePhysRegs;
    
            // Reorder Buffer (ROB)
            struct ROBEntry {
                uint32_t instruction;
                int dest_reg;        // Architectural destination register
                int phys_reg;        // Physical destination register
                int old_phys_reg;    // Previous mapping for recovery
                bool completed;      // Has the instruction completed execution?
                bool ready_to_commit; // Is it ready to commit?
                uint32_t result;     // Result value
            };
            std::queue<ROBEntry> reorderBuffer;
    
            // Common Data Bus signals
            bool CDB_valid;
            int CDB_phys_reg;
            uint32_t CDB_value;
        };
   
        State currentState;
        State nextState;

        // an instruction queue to get instructions from fetch to rename stages  
        std::queue<uint32_t>instruction_queue;
 
        // reservation stations, see reservation.h
        // need to be initialized (maybe?)
        ReservationStation ArithmeticStations[ARITHM_STATIONS]; 
        ReservationStation MemoryStations[MEM_STATIONS]; 

        // create physical registers and reorder buffer
        PhysicalRegisterUnit physRegFile = PhysicalRegisterUnit(REG_COUNT);       
    
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
          
        
        // Get PC
        uint32_t getPC() { return regfile.pc; }

        // Prints the Register File
        void printRegFile() { regfile.print(); }
        
        // Initializes the processor appropriately based on the optimization level
        void initialize(int opt_level);

        // Advances the processor to an appropriate state every cycle
        void advance(); 

	

};
