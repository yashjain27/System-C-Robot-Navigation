#include "systemc.h"
#include "processing.cpp";

int sc_main(int argc, char* argv[]){
  //Inputs
  sc_signal<bool> clk;
  sc_signal<sc_uint<2> > statusIn1;
  sc_signal<bool> incoming1;
  sc_signal<sc_uint<2> > statusIn2;
  sc_signal<bool> incoming2;

  //Outputs
  sc_signal<bool> outgoing1;
  sc_signal<sc_uint<2> > statusOut1;
  sc_signal<bool> outgoing2;
  sc_signal<sc_uint<2> > statusOut2;

  process processing("process");
  processing.clk(clk);
  processing.statusIn1(statusIn1);
  processing.incoming1(incoming1);
  processing.statusIn2(statusIn2);
  processing.incoming2(incoming2);
  processing.outgoing1(outgoing1);
  processing.statusOut1(statusOut1);
  processing.outgoing2(outgoing2);
  processing.statusOut2(statusOut2);

  for(int i = 0; i < 150; i++){
    clk = 0;
    sc_start(1, SC_NS);
    clk = 1;
    sc_start(1,SC_NS);
  }
}
