#include "systemc.h"
#include "robot.cpp"
#include "processing.cpp"
#include "server.cpp"

int sc_main(int argc, char* argv[]){
  //Inputs processing
  sc_signal<bool> clk;
  sc_signal<sc_uint<2> > statusRP1;
  sc_signal<bool> incomingRP1;
  sc_signal<sc_uint<2> > statusRP2;
  sc_signal<bool> incomingRP2;

  //Outputs processing
  sc_signal<bool> outgoingPR1;
  sc_signal<sc_uint<2> > statusPR1;
  sc_signal<bool> outgoingPR2;
  sc_signal<sc_uint<2> > statusPR2;

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

  process processing("process");
  processing.clk(clk);
  processing.statusIn1(statusRP1);
  processing.incoming1(incomingRP1);
  processing.statusIn2(statusRP2);
  processing.incoming2(incomingRP2);
  processing.outgoing1(outgoingPR1);
  processing.statusOut1(statusPR1);
  processing.outgoing2(outgoingPR2);
  processing.statusOut2(statusPR2);

  robot r1("robot");
  r1.statusInProcess(statusRP1);
  r1.incomingProcess(outgoingPR1);
  r1.statusInServer(outgoingSig1);
  r1.incomingServer(statusOut1);
  r1.outgoingProcess(incomingRP1);
  r1.statusOutServer(statusIn1);
  r1.outgoingServer(incomingSig1);
  r1.statusOutProcess(statusRP1);

  robot r2("robot2");
  r2.statusInProcess(statusRP2);
  r2.incomingProcess(outgoingPR2);
  r2.statusInServer(outgoingSig2);
  r2.incomingServer(statusOut2);
  r2.outgoingProcess(incomingRP2);
  r2.statusOutServer(statusIn2);
  r2.outgoingServer(incomingSig2);
  r2.statusOutProcess(statusRP2);

  server srvr("server");
  srvr.statusIn1(statusIn1);
  srvr.incomingSig1(incomingSig1);
  srvr.statusIn2(statusIn2);
  srvr.incomingSig2(incomingSig2);
  srvr.statusOut1(statusOut1);
  srvr.outgoingSig1(outgoingSig1);
  srvr.statusOut2(statusOut2);
  srvr.outgoingSig2(outgoingSig2);

  //Run for 100 cycles
  for(int i = 0; i < 400; i++){
    clk = 1;
    sc_start(1, SC_NS);
    clk = 0;
    sc_start(1, SC_NS);
  }

}
