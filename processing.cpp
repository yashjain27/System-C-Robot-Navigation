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
  sc_uint<6> robot1Path[11] = {10,9,8,7,6,5,4,3,2,1,11}; //Robot 1 Path
  sc_uint<6> robot2Path[10] = {51,49,39,36,26,23,13,11,1,2}; //Robot 2 Path
  sc_uint<6> obs1Path[10] = {34,33,32,31,30,29,28,27,26,23}; //Obstacle 1 Path
  sc_uint<6> obs2Path[10] = {1,2,3,4,5,6,7,8,9,10}; //Obstacle 2 Path
  int currentPath1;
  int nextPath1;
  int currentPath2;
  int nextPath2;
  int currentObsPath1;
  int nextObsPath1;
  int currentObsPath2;
  int nextObsPath2;
  int map3d[60][8] = {
    //Row 0
    {0,0,2,2,0,2,11,0}, //1
    {2,0,4,2,0,3,0,1},  //2
    {4,0,6,2,0,4,0,2},  //3
    {6,0,8,2,0,5,0,3},  //4
    {8,0,10,2,0,6,0,4}, //5
    {10,0,12,2,0,7,0,5},  //6
    {12,0,14,2,0,8,0,6},  //7
    {14,0,16,2,0,9,0,7},  //8
    {16,0,18,2,0,10,0,8}, //9
    {18,0,20,2,0,0,12,9}, //10

    //Row 1
    {0,2,2,4,1,0,13,0}, //11
    {18,2,20,4,10,0,22,0},  //12

    //Row 2
    {0,4,2,6,11,14,23,0}, //13
    {2,4,4,6,0,15,0,13},  //14
    {4,4,6,6,0,16,0,14},  //15
    {6,4,8,6,0,17,0,15},  //16
    {8,4,10,6,0,18,0,16}, //17
    {10,4,12,6,0,19,24,17}, //18
    {12,4,14,6,0,20,0,18},  //19
    {14,4,16,6,0,21,0,19},  //20
    {16,4,18,6,0,22,0,20},  //21
    {18,4,20,6,12,0,25,21}, //22

    //Row 3
    {0,6,2,8,13,0,26,0},  //23
    {10,6,12,8,18,0,31,0},  //24
    {18,6,20,8,22,0,35,0},  //25

    //Row 4
    {0,8,2,10,23,27,36,0},  //26
    {2,8,4,10,0,28,0,26}, //27
    {4,8,6,10,0,29,0,27}, //28
    {6,8,8,10,0,30,0,28}, //29
    {8,8,10,10,0,31,0,29},  //30
    {10,8,12,10,0,32,24,30},  //31
    {12,8,14,10,0,33,0,31}, //32
    {14,8,16,10,0,34,0,32}, //33
    {16,8,18,10,0,35,0,33}, //34
    {18,8,20,10,25,0,38,34},  //35

    //Row 5
    {0,10,2,12,26,0,39,0},  //36
    {12,10,14,12,32,0,45,0},  //37
    {18,10,20,12,35,0,48,0},  //38

    //Row 6
    {0,12,2,14,36,40,49,0}, //39
    {2,12,4,14,0,41,0,39},  //40
    {4,12,6,14,0,42,0,40},  //41
    {6,12,8,14,0,43,0,41},  //42
    {8,12,10,14,0,44,0,42}, //43
    {10,12,12,14,0,45,0,43},  //44
    {12,12,14,14,37,46,0,44}, //45
    {14,12,16,14,0,47,0,45},  //46
    {16,12,18,14,0,48,0,46},  //47
    {18,12,20,14,38,0,48,47}, //48

    //Row 7
    {0,14,2,16,39,0,51,0},  //49
    {18,14,20,16,48,0,60,0},  //50

    //Row 8
    {0,16,2,18,49,52,0,0},  //51
    {2,16,4,18,0,53,0,51},  //52
    {4,16,6,18,0,54,0,52},  //53
    {6,16,8,18,0,55,0,53},  //54
    {8,16,10,18,0,56,0,54}, //55
    {10,16,12,18,0,57,0,55},  //56
    {12,16,14,18,0,58,0,56},  //57
    {14,16,16,18,0,59,0,57},  //58
    {16,16,18,18,0,60,0,58},  //59
    {18,16,20,18,50,0,0,59} //60
  };

  //Update
  void prc_update(){
    //Clear outgoing1 and outgoing2 signal
    outgoing1 = 0;
    outgoing2 = 0;

    //Loop 1 - Update position
    for(int i = 0; i < 2; i++){
      if(status[i] != 1){
        if(direction[i] == 0){ //North
          y[i] = y[i] - .02;
        }else if(direction[i] == 1){ //East
          x[i] = x[i] + .02;
        }else if(direction[i] == 2){ //South
          y[i] = y[i] + .02;
        }else{                       //West
          x[i] = x[i] - .02;
        }

        if(boundary[i]){
          //Update the current and next grids and direction
          //cout << "Boundary flag triggered " << i << endl;
          switch (i) {
            case 0: if(currentPath1 < sizeof(robot1Path)/24 - 1){
                      currentPath1++;
                      cout << "Robot 1 moved to the next grid "<< endl;
                    }
                    if(nextPath1 < sizeof(robot1Path)/24 - 1){
                      nextPath1++;
                      //cout << "Robot 1 next path updated " << nextPath1 << endl;
                    }
                    break;
            case 1: if(currentPath2 < sizeof(robot2Path)/24 - 1){
                      currentPath2++;
                      cout << "Robot 2 moved to the next grid " << endl;
                    }
                    if(nextPath2 < sizeof(robot2Path)/24 - 1){
                      nextPath2++;
                      //cout << "Robot 2 next path updated " << nextPath2 << endl;
                    }
                    break;
          }
          robotTransmit[i] = 1; //Robot is to transmit  a signal
          signalTransmit[i] = 3; //Signal 3 tells the robot has CROSSED
        }else{
          robotTransmit[i] = 0; //Robot isn't to transmit  a signal
          signalTransmit[i] = 2; //Signal 2 tells the robot to RESUME
        }

        //Signal 3 received from server acknowleding the border CROSS of the robot
        if(status[i] == 3){
          robotTransmit[i] = 1; //Robot is to transmit  a signal
          signalTransmit[i] = 2; //Signal 0 tells the robot to RESUME regularly
          cout << "Robot: " << i << " has crossed." << endl;
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
      //Round off float to 2 places
      float valueX = (int)(x[i] * 100 + .5);
      x[i] = (float)(valueX / 100);
      float valueY = (int)(y[i] * 100 + .5);
      y[i] = (float)(valueY / 100);
    }
    cout << "Robot 1 Grid#: " << robot1Path[currentPath1] << " | (x,y): " << "(" << x[0] << "," << y[0] << ")" << " ||| Robot 2 Grid#: " << robot2Path[currentPath2] << " | (x,y): " << "(" << x[1] << "," << y[1] << ")" << endl;
    //Update direction change for robot
    direction_change();


    //Loop 2 - Obstacle updates position
    for(int i = 0; i < 2; i++){
      if(directionObs[i] == 0){ //North
        obsY[i] = obsY[i] - .04;
      }else if(directionObs[i] == 1){ //East
        obsX[i] = obsX[i] + .04;
      }else if(directionObs[i] == 2){ //South
        obsY[i] = obsY[i] + .04;
      }else{                       //West
        obsX[i] = obsX[i] - .04;
      }

      if(directionObs[i] == 1 || directionObs[i] == 3){ //East / West
        if(fmod(obsX[i], 2.0) == 0){
          if(i == 0){
            if(currentObsPath1 < sizeof(obs1Path)/24 - 1){
              currentObsPath1++;
              cout << "Obstacle " << i << " moved to the next grid " << currentObsPath1 << endl;
            }
            if(nextObsPath1 < sizeof(obs1Path)/24 - 1){
              nextObsPath1++;
              //cout << "Obstacle " << i << " updated next path " << nextObsPath1 << endl;
            }
          }else{
            if(currentObsPath2 < sizeof(obs2Path)/24 - 1){
              currentObsPath2++;
              cout << "Obstacle  " << i << " moved to the next grid " << currentObsPath1 << endl;
            }
            if(nextObsPath2 < sizeof(obs2Path)/24 - 1){
              nextObsPath2++;
              //cout << "Obstacle  " << i << " updated next path " << nextObsPath1 << endl;
            }
          }
        }
      }else{
        if(fmod(obsY[i], 2.0) == 0){
          if(i == 0){
            if(currentObsPath1 < sizeof(obs1Path)/24 - 1){
              currentObsPath1++;
              cout << "Obstacle " << i << " moved to the next grid " << currentObsPath1 << endl;
            }
            if(nextObsPath1 < sizeof(obs1Path)/24 - 1){
              nextObsPath1++;
              //cout << "Obstacle " << i << " updated next path " << nextObsPath1 << endl;
            }
          }else{
            if(currentObsPath2 < sizeof(obs2Path)/24 - 1){
              currentObsPath2++;
              cout << "Obstacle  " << i << " moved to the next grid " << currentObsPath1 << endl;
            }
            if(nextObsPath2 < sizeof(obs2Path)/24 - 1){
              nextObsPath2++;
              //cout << "Obstacle  " << i << " updated next path " << nextObsPath1 << endl;
            }
          }
        }
      }
      //Round off float to 2 places
      float valueObsX = (int)(obsX[i] * 100 + .5);
      obsX[i] = (float)(valueObsX / 100);
      float valueObsY = (int)(obsY[i] * 100 + .5);
      obsY[i] = (float)(valueObsY / 100);
    }
    cout << "Obstacle 1 Grid#: " << obs1Path[currentObsPath1] << " | (x,y): " << "(" << obsX[0] << "," << obsY[0] << ")" << " ||| Obstacle 2 Grid#: " << obs2Path[currentObsPath2] << " | (x,y): " << "(" << obsX[1] << "," << obsY[1] << ")" << endl;
    //Update direction change for obstacles
    direction_changeObs();


    //Loop 3 - Compare distance to boundary
    for(int i = 0; i < 2; i++){
      // Send signal CROSSING to server
      if(direction[i] == 0){                  //North
        if(fmod(y[i], 2.0) <= 0.5){
          robotTransmit[i] = 1; //Robot is to transmit  a signal
          signalTransmit[i] = 0; //Signal 0 tells the robot is CROSSING
        }else{
          robotTransmit[i] = 0; //Robot shouldn't transmit a signal
          signalTransmit[i] = 2; //Signal 2 - Robot should resume
        }
      }else if(direction[i] == 1){            //East
        if(fmod(x[i], 2.0) >= 1.5){
          robotTransmit[i] = 1; //Robot is to transmit  a signal
          signalTransmit[i] = 0; //Signal 0 tells the robot is CROSSING
        }else{
          robotTransmit[i] = 0; //Robot shouldn't transmit a signal
          signalTransmit[i] = 2; //Signal 2 - Robot should resume
        }
      }else if(direction[i] == 2){            //South
        if(fmod(y[i], 2.0) >= 1.5){
          robotTransmit[i] = 1; //Robot is to transmit  a signal
          signalTransmit[i] = 0; //Signal 0 tells the robot is CROSSING
        }else{
          robotTransmit[i] = 0; //Robot shouldn't transmit a signal
          signalTransmit[i] = 2; //Signal 2 - Robot should resume
        }
      }else{                                  //West
        if(fmod(x[i], 2.0) <= 0.5){
          robotTransmit[i] = 1; //Robot is to transmit  a signal
          signalTransmit[i] = 0; //Signal 0 tells the robot is CROSSING
        }else{
          robotTransmit[i] = 0; //Robot shouldn't transmit a signal
          signalTransmit[i] = 2; //Signal 2 - Robot should resume
        }
      }

      //Boundary internal flag
      if(direction[i] == 1 || direction[i] == 3){ //East / West
        if(fmod(x[i], 2.0) == 0){
          boundary[i] = 1;
          //cout << "boundary 1 activated" << endl;
        }else{
          boundary[i] = 0;
        }
      }else{
        if(fmod(y[i], 2.0) == 0){
          boundary[i] = 1;
          //cout << "boundary 2 activated" << endl;
        }else{
          boundary[i] = 0;
        }
      }
    }

    //Loop 4 - Compare distance to other obstacles
    for(int i = 0; i < 2; i++){
      //Loop each obstacle
      for(int j = 0; j < 2; j++){
        //cout << "Abs: " << (abs(x[i] - obsX[j]) <= 3) << "(i,j)" << i << "," << j << endl;
        //cout << "True? " << (y[i] == obsY[j]) << " x[i]: " << y[i] << " obsX[j]: " << obsY[j] << endl;
        //cout << "Third: " << ((abs(x[i] - obsX[j]) <= 3) && y[i] == obsY[j])  << endl;
        if(((abs(x[i] - obsX[j]) <= 3) && y[i] == obsY[j]) || ((abs(y[i] - obsY[j]) <= 3) && x[i] == obsX[j])){ //If robot detects an obstacle within 3m, send a stop signal
          robotTransmit[i] = 1; //Robot is to transmit  a signal
          signalTransmit[i] = 1; //Signal 1 tells the robot to tell the server to STOP
          cout << "STOP, robot " << (i+1) << " is in the way of obstacle " << (j+1) << endl;
        }else{
          robotTransmit[i] = 0; //Robot shouldn't transmit a signal
          signalTransmit[i] = 2; //Signal 2 - Robot should resume
        }
      }
    }
    //Transmit the signals;
    prc_transmit();
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
    cout << endl << endl;
  }

  //Update direction change for robots
  void direction_change(){
    //Robot 1
    if(fmod(x[0],2.0) == 1.0 && fmod(y[0],2.0) == 1.0){
      for(int i = 4; i < 8; i++){
        if(robot1Path[nextPath1] == map3d[robot1Path[currentPath1]-1][i]){
          //cout << "1. Next Grid is: " << robot1Path[nextPath1] << endl;
          //cout << "map3d[robot1Path[currentPath1]][i]: " << map3d[robot1Path[currentPath1]-1][i] << endl;
          direction[0] = i - 4;
          break;
        }
      }
    }
    //cout << "Direction of Robot 1: " << direction[0] << endl;

    //Robot 2
    if(fmod(x[1],2.0) == 1.0 && fmod(y[1],2.0) == 1.0){
      //cout << "2. Current grid is " << robot2Path[currentPath2] << " next  grid is: " << robot2Path[nextPath2] << endl;
      for(int i = 4; i < 8; i++){
        if(robot2Path[nextPath2] == map3d[robot2Path[currentPath2]-1][i]){
          //cout << "2. Next Path is " << nextPath2 << " next  grid is: " << robot2Path[nextPath2] << endl;
          //cout << "map3d[robot2Path[currentPath2]][i]: " << map3d[robot2Path[currentPath2]-1][i] << endl;
          direction[1] = i - 4;
          break;
        }
      }
    }
    //cout << "Direction of Robot 2: " << direction[1] << endl;
  }

  //Update direction change for obstacles
  void direction_changeObs(){
    //Obstacle 1
    if(fmod(obsX[0],2.0) == 1.0 && fmod(obsY[0],2.0) == 1.0){
      for(int i = 4; i < 8; i++){
        if(obs1Path[nextObsPath1] == map3d[obs1Path[currentObsPath1]-1][i]){
          directionObs[0] = i - 4;
          //cout << i << " Next path: " << obs1Path[nextObsPath1] << " Current path: " << obs1Path[currentObsPath1] << endl;
          break;
        }
      }
    }
    //cout << "Direction of Obstacle 1: " << directionObs[0] << endl;

    //Obstacle 2
    if(fmod(obsX[1],2.0) == 1.0 && fmod(obsY[1],2.0) == 1.0){
      for(int i = 4; i < 8; i++){
        if(obs2Path[nextObsPath2] == map3d[obs2Path[currentObsPath2]-1][i]){
          directionObs[1] = i - 4;
          break;
        }
      }
    }
    //cout << "Direction of Obstacle 2: " << directionObs[1] << endl;
  }

  SC_CTOR(process){
    cout << "Executing processing.cpp" << endl;
    SC_METHOD(prc_update);
    sensitive << clk.pos();
    SC_METHOD(prc_receive);
    sensitive << incoming1;
    sensitive << incoming2;

    //Initializers
    currentPath1 = 0;
    nextPath1 = 1;
    currentPath2 = 0;
    nextPath2 = 1;
    currentObsPath1 = 0;
    nextObsPath1 = 1;
    currentObsPath2 = 0;
    nextObsPath2 = 1;
    direction[0] = 3;
    direction[1] = 0;
    directionObs[0] = 3;
    directionObs[1] = 1;
    x[0] = 19;
    y[0] = 1;
    x[1] = 1;
    y[1] = 17;
    obsX[0] = 17;
    obsY[0] = 9;
    obsX[1] = 1;
    obsY[1] = 1;

  }
};
