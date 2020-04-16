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
  sc_out<sc_uint<2> > statusOutProcess;

  //Receive Process
  void prc_receive_process(){
    cout << "Received from process: " << statusInProcess.read() << endl;
    prc_transmit_server();
  }

  //Transmit Process
  void prc_transmit_process(){
    outgoingServer = incomingProcess;
    statusOutProcess = statusInServer.read();
  }

  //Receive Server
  void prc_receive_server(){
    cout << "Received from server: " << statusInServer.read() << endl;
    prc_transmit_process();
  }

  //Transmit Server
  void prc_transmit_server(){
    outgoingProcess = incomingServer; 
    statusOutServer = statusInProcess.read();
  }

  SC_CTOR(robot){
    cout << "Executing robot.cpp" << endl;
    SC_METHOD(prc_receive_process);
    sensitive << incomingProcess.pos();
    SC_METHOD(prc_receive_server);
    sensitive << incomingServer.pos();
  }
};
