#include <iostream>

#ifdef enable_debug
#define debug(x) x
#else
#define debug(x) 
#endif

class ReservationStation {

    private:

        uint32_t operation; // operation to perform
   
        uint32_t RS1; // station that will produce first operand, 0 indicates operand available
        uint32_t RS2; // station that will produce second operand 
  
        uint32_t Val1; // value of first operand, empty if RS1 has something
        uint32_t Val2; // value of second operand              

        uint32_t immediate; // immediate value (if necessary)
        bool occupied; // is this station occupied?

    public:

        ReservationStation() {
            occupied = false;
        };

        // Check if CandidateStation is occupied
        bool checkStation() { return occupied; }
       
        // check if RS is present in station 
        //bool peekRS() { return (ResStation == RS1 || ResStation == RS2); }

        // write values to station
        void writeValues(uint32_t Value_1, uint32_t Value_2) { Val1 = Value_1; Val2 = Value_2; } 
 
        // write RS values
        void writeRS(uint32_t Stat1, uint32_t Stat2) { RS1 = Stat1; RS2 = Stat2; }

};
