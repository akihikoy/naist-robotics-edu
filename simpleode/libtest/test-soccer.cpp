#include "simpleode/lib/soccer.h"
#include <iostream>
using namespace std;

double Target0[3]={0.5,0.0,0.0};
double Target1[3]={-0.5,0.0,0.0};

double get_theta(xode::TDynRobot &robot, int idx)
{
  const dReal *R= robot.Body(idx).getRotation();
  // for(int r(0);r<3;++r) {for(int c(0);c<4;++c)  cerr<<"\t"<<R[r*4+c]; cerr<<endl;}
  return std::atan2(R[1*4+0],R[0*4+0]);
}

void move_to(xode::TDynRobot &robot, int idx, const double &x, const double &y, const double &theta)
{
  const double kp(0.2),kd(0.1);
  const double krp(0.01),krd(0.01);
  robot.Body(idx).addForce(
      kp*(x-robot.Body(idx).getPosition()[0])+kd*(-robot.Body(idx).getLinearVel()[0]),
      kp*(y-robot.Body(idx).getPosition()[1])+kd*(-robot.Body(idx).getLinearVel()[1]),
      0.0);
  robot.Body(idx).addTorque(0.0,0.0,
      krp*(theta-get_theta(robot,idx))+krd*(-robot.Body(idx).getAngularVel()[2]) );
  // cerr<<robot.Body(idx).getPosition()[0]<<", "<<robot.Body(idx).getPosition()[1]<<endl;
  // cerr<<get_theta(robot,idx)<<endl;
}

void test_ctrl(xode::TEnvironment &env, xode::TDynRobot &robot)
{
  static int count(0);
  if(count%20==0)
  {
    Target0[0]= (dRandReal()-0.5)*2.0;
    Target0[1]= (dRandReal()-0.5)*1.5;
    Target0[2]= (dRandReal()-0.5)*2.0*M_PI;
  }
  move_to(robot,0,Target0[0],Target0[1],Target0[2]);
  move_to(robot,1,Target1[0],Target1[1],Target1[2]);
  ++count;
}

void test_draw(xode::TEnvironment &env, xode::TDynRobot &robot)
{
}

void test_keyevent(xode::TEnvironment &env, xode::TDynRobot &robot, int command)
{
  const double d(0.05);
  if(command=='r')  env.Reset();

  else if(command=='z')  Target1[0]+=d;
  else if(command=='c')  Target1[0]-=d;
  else if(command=='x')  Target1[1]+=d;
  else if(command=='s')  Target1[1]-=d;
  else if(command=='a')  Target1[2]+=d;
  else if(command=='d')  Target1[2]-=d;
}

int main (int argc, char **argv)
{
  xode::TEnvironment env;
  dsFunctions fn= xode::SimInit("textures",env);
  xode::ControlCallback= &test_ctrl;
  xode::DrawCallback= &test_draw;
  xode::KeyEventCallback= &test_keyevent;

  cerr<<"Push \'R\' to reset the simpulator"<<endl;

  env.SurfaceParams().mu= 0.00;

  dsSimulationLoop (argc,argv,400,400,&fn);

  dCloseODE();
  return 0;
}
