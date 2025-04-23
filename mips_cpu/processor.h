#include "memory.h"
#include "regfile.h"
#include "ALU.h"
#include "control.h"
#include "reservation.h"

#ifdef enable_debug
#define debug(x) x
#else
#define debug(x) 
#endif

class Processor {
    private:

        int opt_level;
        ALU alu;
        control_t control;
        Memory *memory;
        Registers regfile;
     
        //// OOO things
   
        // reservation stations, see reservation.h
        // need to be initialized
        ReservationStation stationSet[4]; 

        void single_cycle_processor_advance();
        void pipelined_processor_advance();
 
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
};
