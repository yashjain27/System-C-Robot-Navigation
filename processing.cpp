#include "systemc.h"

SC_MODULE(process){
  //Inputs
  sc_in<bool> clk;
  sc_in<sc_uint<2> > statusIn1;
  sc_in<bool> incoming1;
  sc_in<sc_uint<2> > statusIn2;
  sc_in<bool> incoming2;

  //Outputs
  sc_out<bool> outgoing1;
  sc_out<sc_uint<2> > statusOut1;
  sc_out<bool> outgoing2;
  sc_out<sc_uint<2> > statusOut2;

  //Signals
  sc_signal<bool> flag; //Used to tell processing to transmit, direction is from update process to transmit

  //Variables
  float x[2]; //Both robots' X position
  float y[2]; //Both robots' Y position
  float obsX[2]; //Both obstacles' X position
  float obsY[2]; //Both obstacles' Y position
  sc_uint<2> status[2]; //Stauses for each robot, 0 - OK, 1 - STOP, 2 - RESUME,
  sc_uint<2> direction[2]; //Direction of each robot//0 - north, 1 - east, 2 - south, 3 - west
  sc_uint<2> directionObs[2]; //Direction of each obstacle//0 - north, 1 - east, 2 - south, 3 - west
  bool boundary[2]; //Stop at boundary
  bool robotTransmit[2]; //Choose which robot transmits
  sc_uint<2> signalTransmit[2]; //Specify which signal to transmit: 0,1,2

  //Update
  void prc_update(){
    //Clear outgoing1 and outgoing2 signal
    outgoing1 = 0;
    outgoing2 = 0;

    //Loop 1 - Update position
    for(int i = 0; i < 2; i++){
      if(status[i] != 1){
        if(direction[i] == 0){ //North
          y[i] = y[i] + .02;
        }else if(direction[i] == 1){ //East
          x[i] = x[i] + .02;
        }else if(direction[i] == 2){ //South
          y[i] = y[i] - .02;
        }else{                       //West
          x[i] = x[i] - .02;
        }

        if(boundary[i]){
          //update the current and next grids
          robotTransmit[i] = 1; //Robot is to transmit  a signal
          signalTransmit[i] = 3; //Signal 3 tells the robot has CROSSED
          flag = 1;
        }else{
          robotTransmit[i] = 0; //Robot isn't to transmit  a signal
          signalTransmit[i] = 2; //Signal 2 tells the robot to RESUME
        }
        
        //Signal 3 received from server acknowleding the border CROSS of the robot
        if(status[i] == 3){
          robotTransmit[i] = 1; //Robot is to transmit  a signal
          signalTransmit[i] = 2; //Signal 0 tells the robot to RESUME regularly
        }
      }else if(status[i] == 1){
        //Stop at boundary
        if(!boundary[i]){
          if(direction[i] == 0){ //North
            y[i] = y[i] + .02;
          }else if(direction[i] == 1){ //East
            x[i] = x[i] + .02;
          }else if(direction[i] == 2){ //South
            y[i] = y[i] - .02;
          }else{                      //West
            x[i] = x[i] - .02;
          }
        }
      }
    }

    //Loop 2 - Obstacle updates position
    for(int i = 0; i < 2; i++){
      //UPDATE GRID POSITIONS AND BOUNDARIES


      if(directionObs[i] == 0){ //North
        obsY[i] = obsY[i] + .04;
      }else if(directionObs[i] == 1){ //East
        obsX[i] = obsX[i] + .04;
      }else if(directionObs[i] == 2){ //South
        obsY[i] = obsY[i] - .04;
      }else{                       //West
        obsX[i] = obsX[i] - .04;
      }
    }

    //Loop 3 - Compare distance to boundary
    for(int i = 0; i < 2; i++){
      // Send signal CROSSING to server
      if(direction[i] == 0){                  //North
        if(fmod(y[i], 2.0) <= 0.5){
          robotTransmit[i] = 1; //Robot is to transmit  a signal
          signalTransmit[i] = 0; //Signal 0 tells the robot is CROSSING
          flag = 1;
        }else{
          robotTransmit[i] = 0; //Robot shouldn't transmit a signal
          signalTransmit[i] = 2; //Signal 2 - Robot should resume
        }
      }else if(direction[i] == 1){            //East
        if(fmod(x[i], 2.0) >= 1.5){
          robotTransmit[i] = 1; //Robot is to transmit  a signal
          signalTransmit[i] = 0; //Signal 0 tells the robot is CROSSING
          flag = 1;
        }else{
          robotTransmit[i] = 0; //Robot shouldn't transmit a signal
          signalTransmit[i] = 2; //Signal 2 - Robot should resume
        }
      }else if(direction[i] == 2){            //South
        if(fmod(y[i], 2.0) >= 1.5){
          robotTransmit[i] = 1; //Robot is to transmit  a signal
          signalTransmit[i] = 0; //Signal 0 tells the robot is CROSSING
          flag = 1;
        }else{
          robotTransmit[i] = 0; //Robot shouldn't transmit a signal
          signalTransmit[i] = 2; //Signal 2 - Robot should resume
        }
      }else{                                  //West
        if(fmod(x[i], 2.0) <= 0.5){
          robotTransmit[i] = 1; //Robot is to transmit  a signal
          signalTransmit[i] = 0; //Signal 0 tells the robot is CROSSING
          flag = 1;
        }else{
          robotTransmit[i] = 0; //Robot shouldn't transmit a signal
          signalTransmit[i] = 2; //Signal 2 - Robot should resume
        }
      }

      //Boundary internal flag
      if(direction[i] == 1 || direction[i] == 3){ //East / West
        boundary[i] = (fmod(x[i], 2.0) == 0);
      }else{
        boundary[i] = (fmod(y[i], 2.0) == 0);
      }
    }

    //Loop 4 - Compare distance to other obstacles
    for(int i = 0; i < 2; i++){
      //Loop each obstacle
      for(int j = 0; j < 2; j++){
        if((abs(x[i] - obsX[j] <= 3) && y[i] == obsY[i]) || ((abs(y[i] - obsY[j]) <= 3) && x[i] == obsX[i])){ //If robot detects an obstacle within 3m, send a stop signal
          robotTransmit[i] = 1; //Robot is to transmit  a signal
          signalTransmit[i] = 1; //Signal 1 tells the robot to tell the server to STOP
          flag = 1;
        }else{
          robotTransmit[i] = 0; //Robot shouldn't transmit a signal
          signalTransmit[i] = 2; //Signal 2 - Robot should resume
        }
      }
    }

  }

  //Receive
  void prc_receive(){
    if(incoming1){
      status[0] = statusIn1;
    }
    if(incoming2){
      status[1] = statusIn2;
    }
  }

  //Transmit
  void prc_transmit(){
    //Loop through to see which robot wants to transmit
    for(int i = 0; i < 2; i++){
      if(robotTransmit[i] == 1){
        switch (i) {
          case 0: outgoing1 = 1;
                  statusOut1 = signalTransmit[i];
                  break;
          case 1: outgoing2 = 1;
                  statusOut2 = signalTransmit[i];
                  break;
        }
      }
    }
    flag = 0;
  }

  SC_CTOR(process){
    cout << "Executing processing.cpp" << endl;
    SC_METHOD(prc_update);
    sensitive << clk.pos();
    SC_METHOD(prc_transmit);
    sensitive << flag.pos();
    SC_METHOD(prc_receive);
    sensitive << incoming1;
    sensitive << incoming2;
  }
};
