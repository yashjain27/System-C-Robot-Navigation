#include "systemc.h"

SC_MODULE(server){
  //Inputs
  sc_in<sc_uint<2> > statusIn1;
  sc_in<bool> incomingSig1;
  sc_in<sc_uint<2> > statusIn2;
  sc_in<bool> incomingSig2;

  //Outputs
  sc_out<bool> statusOut;
  sc_out<sc_uint<2> > outgoingSig;

  //Signals
  sc_signal<bool> flagUpdate;
  sc_signal<bool> flagTransmit;

  //Variables
  sc_uint<6> robot1Path[10]; //Robot 1 Path
  sc_uint<6> robot2Path[10]; //Robot 2 Path
  int currentPath1;
  int nextPath1;
  int currentPath2;
  int nextPath2;
  sc_uint<2> status[2]; //Stauses for each robot, 0 - OK, 1 - STOP, 2 - RESUME, 3 = CROSSED

  //Update
  void prc_update(){
    //Update Robot 1
    if(incoming1){
      status[0] = statusIn1;
      //Respond to Signal 0 - CROSSING

      //Respond to Signal 1 - STOPPED

      //Respond to Signal 2 - RESUME

      //Respon to Signal 3 - CROSSED by updating the grid indexes
      if(statusIn1 == 3){
        if(currentPath1 < sizeof(robot1Path)/sizeof(int) - 1){
          currentPath1++;
        }
        if(nextPath1 < sizeof(robot1Path)/sizeof(int) - 1){
          nextPath1++;
        }
      }
    }
    //Update Robot 2
    if(incoming2){
      status[1] = statusIn2;
      //Update Grid
      if(statusIn2 == 3){
        if(currentPath2 < sizeof(robot2Path)/sizeof(int) - 1){
          currentPath2++;
        }
        if(nextPath2 < sizeof(robot2Path)/sizeof(int) - 1){
          nextPath2++;
        }
      }
    }

    flagUpdate = 0;
    flagTransmit = 1;
  }

  //Receive
  void prc_receive(){
    flagUpdate = 1;
  }

  //Transmit
  void prc_transmit(){

    flagTransmit = 0;
  }

  SC_CTOR(server){
    cout << "Executing server.cpp" << endl;
    SC_METHOD(prc_receive);
    sensitive << incomingSig1;
    sensitive << incomingSig2;
    SC_METHOD(prc_update);
    sensitive << flagUpdate.pos();
    SC_METHOD(prc_receive);
    sensitive << flagTransmit.pos();
  }
};
