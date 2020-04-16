#include "systemc.h"
#include "robot.cpp"

int sc_main(int argc, char* argv[]){
  //Inputs
  sc_signal<bool> incomingProcess;
  sc_signal<sc_uint<2> > statusInProcess;
  sc_signal<bool> incomingServer;
  sc_signal<sc_uint<2> > statusInServer;
  sc_signal<bool> outgoingProcess;
  sc_signal<sc_uint<2> > statusOutServer;
  sc_signal<bool> outgoingServer;
  sc_signal<sc_uint<2> > statusOutProcess;

  robot r1("robot");
  r1.statusInProcess(statusInProcess);
  r1.incomingProcess(incomingProcess);
  r1.statusInServer(statusInServer);
  r1.incomingServer(incomingServer);
  r1.outgoingProcess(outgoingProcess);
  r1.statusOutServer(statusOutServer);
  r1.outgoingServer(outgoingServer);
  r1.statusOutProcess(statusOutProcess);

  sc_start(1, SC_NS);
  incomingProcess = 0;
  incomingServer = 0;

  sc_start(5, SC_NS);
  statusInProcess = 2;
  incomingProcess = 1;
  //cout << "Outgoing to server: " << outgoingProcess << "  Process #: " << outgoingServer << endl;
  //cout << "Outgoing to process: " << outgoingServer << "  Process #: " << outgoingProcess << endl;

  sc_start(1, SC_NS);
  incomingProcess = 0;

  sc_start(1, SC_NS);
  incomingProcess = 1;
  statusInProcess = 3;

  sc_start(1, SC_NS);
  incomingProcess = 0;

  sc_start(1, SC_NS);
  incomingServer = 1;
  statusInServer = 1;

  sc_start(1, SC_NS);
  incomingServer = 0;

  sc_start(1, SC_NS);
  incomingServer = 1;
  statusInServer = 3;

  sc_start(1, SC_NS);

}
