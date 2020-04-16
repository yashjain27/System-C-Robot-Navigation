#include "systemc.h"

SC_MODULE(server){
  //Inputs
  sc_in<sc_uint<2> > statusIn1;
  sc_in<bool> incomingSig1;
  sc_in<sc_uint<2> > statusIn2;
  sc_in<bool> incomingSig2;

  //Outputs
  sc_out<bool> statusOut1;
  sc_out<sc_uint<2> > outgoingSig1;
  sc_out<bool> statusOut2;
  sc_out<sc_uint<2> > outgoingSig2;

  //Variables
  sc_uint<6> robot1Path[11] = {10,9,8,7,6,5,4,3,2,1,11}; //Robot 1 Path
  sc_uint<6> robot2Path[10] = {51,49,39,36,26,23,13,11,1,2}; //Robot 2 Path
  int currentPath1;
  int nextPath1;
  int currentPath2;
  int nextPath2;
  sc_uint<2> status[2]; //Stauses for each robot, 0 - OK, 1 - STOP, 2 - RESUME, 3 = CROSSED

  //Update
  void prc_update(){
    //Update Robot 1
    if(incomingSig1.read()){
      cout << "Incoming Robot 1 Signal " << endl;

      //Respond to Signal 0 - CROSSING
      if(statusIn1.read() == 0){
        cout << "Robot 1 crossing " << endl;
        if(robot1Path[currentPath1] == robot2Path[nextPath2 + 1]){
          cout << "Robot 1 should stop, robot 2 in the way " << endl;
          status[0] = 1; // STOP
        }else{
          status[0] = 0; //OK
        }
      }else if(statusIn1.read() == 1 || statusIn1.read() == 2){
        //Respond to Signal 1 (STOP) and Signal 2 (RESUME)
        cout << "Robot 1 STOP OR RESUME " << endl;
        status[0] = statusIn1.read();
      }else{
        //Respond to Signal 3 - CROSSED by updating the grid indexes
        if(currentPath1 < sizeof(robot1Path)/24 - 1){
          currentPath1++;
          status[0] = 3;
          cout << "Robot 1 CROSSED " << currentPath1 << endl;
        }
        if(nextPath1 < sizeof(robot1Path)/24 - 1){
          nextPath1++;
        }
      }
    }

    //Update Robot 2
    if(incomingSig2.read()){
      cout << "Incoming Robot 2 Signal " << endl;

      //Respond to Signal 0 - CROSSING
      if(statusIn1.read() == 0){
        cout << "Robot 2 crossing " << endl;
        if(robot2Path[currentPath2] == robot1Path[nextPath1 + 1]){
          cout << "Robot 2 should stop, robot 1 in the way " << endl;
          status[1] = 1; // STOP
        }else{
          status[1] = 0; //OK
        }
      }else if(statusIn2.read() == 1 || statusIn2.read() == 2){
        //Respond to Signal 1 (STOP) and Signal 2 (RESUME)
        cout << "Robot 2 STOP OR RESUME " << endl;
        status[1] = statusIn2.read();
      }else{
        //Respond to Signal 3 - CROSSED by updating the grid indexes
        if(currentPath2 < sizeof(robot2Path)/24 - 1){
          currentPath2++;
          status[1] = 3;
          cout << "Robot 2 CROSSED " << currentPath2 << endl;
        }
        if(nextPath2 < sizeof(robot2Path)/24 - 1){
          nextPath2++;
        }
      }
    }
    prc_transmit();
  }

  //Receive
  void prc_receive(){
    prc_update();
  }

  //Transmit
  void prc_transmit(){
    if(incomingSig1.read()){
      statusOut1 = 1;
      outgoingSig1 = status[0];
      cout << "Robot 1: " << outgoingSig1 << " position: " << robot1Path[currentPath1] << endl;
    }
    if(incomingSig2.read()){
      statusOut2 = 1;
      outgoingSig2 = status[1];
      cout << "Robot 2: " << outgoingSig2 << " position: " << robot2Path[currentPath2] << endl;
    }
  }


  SC_CTOR(server){
    cout << "Executing server.cpp" << endl;
    SC_METHOD(prc_receive);
    sensitive << incomingSig1.pos();
    sensitive << incomingSig2.pos();
    currentPath1 = 0;
    currentPath2 = 0;
    nextPath1 = 0;
    nextPath2 = 0;
  }
};
