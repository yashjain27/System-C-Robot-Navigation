#include "systemc.h"
#include "server.cpp"

int sc_main(int argc, char* argv[]){
  //Inputs server
  sc_signal<bool> incomingSig1;
  sc_signal<sc_uint<2> > statusIn1;
  sc_signal<bool> incomingSig2;
  sc_signal<sc_uint<2> > statusIn2;

  //Outputs server
  sc_signal<bool> statusOut1;
  sc_signal<sc_uint<2> > outgoingSig1;
  sc_signal<bool> statusOut2;
  sc_signal<sc_uint<2> > outgoingSig2;

  server srvr("server");
  srvr.statusIn1(statusIn1);
  srvr.incomingSig1(incomingSig1);
  srvr.statusIn2(statusIn2);
  srvr.incomingSig2(incomingSig2);
  srvr.statusOut1(statusOut1);
  srvr.outgoingSig1(outgoingSig1);
  srvr.statusOut2(statusOut2);
  srvr.outgoingSig2(outgoingSig2);

  sc_start(1, SC_NS);
  incomingSig1 = 0;
  incomingSig2 = 0;

  sc_start(1, SC_NS);
  incomingSig1 = 1;
  incomingSig2 = 1;
  statusIn1 = 0;
  statusIn2 = 0;

  sc_start(1, SC_NS);
  incomingSig1 = 0;
  incomingSig2 = 0;

  sc_start(1, SC_NS);
  incomingSig1 = 1;
  incomingSig2 = 1;
  statusIn1 = 1;
  statusIn2 = 1;

  for(int i = 0; i < 7; i++){
    sc_start(1, SC_NS);
    incomingSig1 = 0;
    incomingSig2 = 0;

    sc_start(1, SC_NS);
    incomingSig1 = 1;
    incomingSig2 = 1;
    statusIn1 = 3;
    statusIn2 = 3;
  }

  cout << endl;
  cout << "8th" << endl << endl;

  sc_start(1, SC_NS);
  incomingSig1 = 0;
  incomingSig2 = 0;

  sc_start(1, SC_NS);
  incomingSig1 = 1;
  incomingSig2 = 1;
  statusIn1 = 0;
  statusIn2 = 0;

  sc_start(1, SC_NS);
  incomingSig1 = 0;
  incomingSig2 = 0;

  sc_start(1, SC_NS);
  incomingSig1 = 1;
  incomingSig2 = 1;
  statusIn1 = 0;
  statusIn2 = 0;
}
