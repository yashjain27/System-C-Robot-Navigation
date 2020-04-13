#include "systemc.h"
#include "robot.cpp"
#include "processing.cpp"
#include "server.cpp"

int sc_main(int argc, char* argv[]){
  //Inputs
  sc_signal<bool> clk;
  sc_signal<sc_uint<6> > nextPath1;
  sc_signal<sc_uint<6> > nextPath2;
  sc_signal<bool> safeToCross1;
  sc_signal<bool> safeToCross2;
  sc_signal<bool> server1;
  sc_signal<bool> server2;
  sc_signal<sc_uint<1> > distance1;
  sc_signal<sc_uint<1> > distance2;
  sc_signal<bool> loop1;
  sc_signal<bool> loop2;
  sc_signal<bool> loop3;
  sc_signal<bool> loop4;
  sc_signal<bool> crossing1;
  sc_signal<bool> crossing2;
  sc_signal<bool> stopped1;
  sc_signal<bool> stopped2;
  sc_signal<bool> moving1;
  sc_signal<bool> moving2;

  //Obstacle 1
  sc_uint<6> path[10] = {13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
  sc_uint<6> index1;
  sc_uint<1> stepper1;

  //Obstacle 2
  sc_uint<6> path[10] = {26, 27, 28, 29, 30, 31, 32, 33, 34, 35};
  sc_uint<6> index2;
  sc_uint<1> stepper2;

  robot r1("robot1");
  r1.nextPath(nextPath1);
  r1.safeToCross(safeToCross1);
  r1.serverIn(server1);
  r1.distance(distance1);
  r1.loopIn1(loop1);
  r1.loopIn3(loop3);
  r1.loopIn4(loop4);
  r1.crossing(crossing1);
  r1.stopped(stopped1);
  r1.moving(moving1);

  robot r2("robot2");
  r2.nextPath(nextPath2);
  r2.safeToCross(safeToCross2);
  r2.serverIn(server2);
  r2.distance(distance2);
  r2.loopIn1(loop1);
  r2.loopIn3(loop3);
  r2.loopIn4(loop4);
  r2.crossing(crossing2);
  r2.stopped(stopped2);
  r2.moving(moving2);

  process p1("process");
  p1.clk(clk);
  p1.loop1(loop1);
  p1.loop2(loop2);
  p1.loop3(loop3);
  p1.loop4(loop4);

  server s1("server");
  s1.crossing1(crossing1);
  s1.stopped1();
  s1.moving1(stopped1);
  s1.crossing2(crossing2);
  s1.stopped2(stopped2);
  s1.moving2(moving2);
  s1.serverOut1(server1);
  s1.serverOut2(server2);

  //Run for 100 cycles
  for(int i = 0; i < 100; i++){
    clk = 1;
    sc_start(1, SC_NS);
    clk = 0;
    sc_start(1, SC_NS);
  }

}
