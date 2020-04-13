#include "systemc.h"

SC_MODULE(robot){
  //Inputs
  sc_in<sc_uint<2> > statusInProcess;
  sc_in<bool> incomingProcess;
  sc_in<sc_uint<2> > statusInServer;
  sc_in<bool> incomingServer;

  //Outputs
  sc_out<bool> outgoingProcess;
  sc_out<sc_uint<2> > statusOutServer;
  sc_out<bool> outgoingServer;
  sc_in<sc_uint<2> > statusOutProcess;

  //Variables
  sc_signal<bool> flagPtoR;
  sc_signal<bool> flagStoR;

  //Receive Process
  void prc_receive_process(){
    flagPtoR = 1;
  }

  //Transmit Process
  void prc_transmit_process(){
    statusOutProcess = statusInServer.read();
    flagStoR = 0;
  }

  //Receive Server
  void prc_receive_server(){
    flagStoR = 1;
  }

  //Transmit Server
  void prc_transmit_server(){
    statusOutServer = statusInProcess.read();
    flagPtoR = 0;
  }

  SC_CTOR(robot){
    cout << "Executing robot.cpp" << endl;
    SC_METHOD(prc_receive_process);
    sensitive << incomingProcess;
    SC_METHOD(prc_transmit_process);
    sensitive << flagStoR.pos();
    SC_METHOD(prc_receive_server);
    sensitive << incomingServer;
    SC_METHOD(prc_transmit_server);
    sensitive << flagPtoR.pos();

  }
};
