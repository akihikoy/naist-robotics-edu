#include "simpleode/lib/soccer.h"
#include <iostream>
using namespace std;

template<typename T>
inline T Square(const T &x)  {return x*x;}

double get_theta(xode::TDynRobot &robot, int idx)
{
  const dReal *R= robot.Body(idx).getRotation();
  // for(int r(0);r<3;++r) {for(int c(0);c<4;++c)  cerr<<"\t"<<R[r*4+c]; cerr<<endl;}
  return std::atan2(R[1*4+0],R[0*4+0]);
}

void move_to(xode::TDynRobot &robot, int idx, const double &x, const double &y, const double &theta)
{
  const double kp(1.0),kd(0.2);
  const double krp(0.1),krd(0.01);
  robot.Body(idx).addForce(
      kp*(x-robot.Body(idx).getPosition()[0])+kd*(-robot.Body(idx).getLinearVel()[0]),
      kp*(y-robot.Body(idx).getPosition()[1])+kd*(-robot.Body(idx).getLinearVel()[1]),
      0.0);
  robot.Body(idx).addTorque(0.0,0.0,
      krp*(theta-get_theta(robot,idx))+krd*(-robot.Body(idx).getAngularVel()[2]) );
  // cerr<<robot.Body(idx).getPosition()[0]<<", "<<robot.Body(idx).getPosition()[1]<<endl;
  // cerr<<get_theta(robot,idx)<<endl;
}

class TInterpolator
{
public:
  void Init(const double &q0, const double &qt, const double &v, const double &a)
    {
      t_= 0.0;
      q0_= q0;
      qt_= qt;
      v_= v;
      a_= a;
      d_= qt_-q0_;
      s_= (qt_>=q0_?1.0:-1.0);
      if(s_*d_ > v_*v_/a_)
      {
        t1_= v_/a_;
        tf_= t1_ + d_/(s_*v_);
        t2_= tf_ - t1_;
        x1_= 0.5*s_*v_*v_/a_;
        x2_= d_ - x1_;
        v2_= v_;
      }
      else
      {
        tf_= 2.0*std::sqrt(s_*d_/a_);
        t1_= 0.5*tf_;
        t2_= t1_;
        x1_= 0.5*d_;
        x2_= x1_;
        v2_= a_*t1_;
      }
    }
  double Step(const double &time_step)
    {
      double x;
      t_+= time_step;
      if(t_<t1_)  x= 0.5*s_*a_*Square(t_);
      else if(t_<t2_)  x= x1_ + s_*v_*(t_-t1_);
      else if(t_<tf_) x= x2_ + s_*v2_*(t_-t2_) - 0.5*s_*a_*Square(t_-t2_);
      else x= d_;
      x+= q0_;
      return x;
    }
  const double& Tf() const {return tf_;}
private:
  double t_;
  double q0_;
  double qt_;
  double v_;
  double a_;
  double d_;
  double s_;
  double t1_;
  double tf_;
  double t2_;
  double x1_;
  double x2_;
  double v2_;
};


// double Target0[3]={0.5,0.0,0.0};
double Target1[3]={-0.5,0.0,0.0};
int Count(0);

TInterpolator Intpl0[3];
// TInterpolator Intpl1[3];
const double V(0.5), A(0.2);

void test_ctrl(xode::TEnvironment &env, xode::TDynRobot &robot)
{
  if(Count%100==0)
  {
    Intpl0[0].Init(robot.Body(0).getPosition()[0], (dRandReal()-0.5)*2.0, V, A);
    Intpl0[1].Init(robot.Body(0).getPosition()[1], (dRandReal()-0.5)*1.5, V, A);
    Intpl0[2].Init(get_theta(robot,0), (dRandReal()-0.5)*2.0*M_PI, V, A);
  }
  move_to(robot,0,Intpl0[0].Step(env.TimeStep()),Intpl0[1].Step(env.TimeStep()),Intpl0[2].Step(env.TimeStep()));
  move_to(robot,1,Target1[0],Target1[1],Target1[2]);
  ++Count;
}

void test_draw(xode::TEnvironment &env, xode::TDynRobot &robot)
{
  dReal sides[3]={0.1,0.1,0.1};
  dVector3 p;
  std::copy(robot.Body(0).getPosition(),robot.Body(0).getPosition()+3,p);
  p[2]+=0.2;
  dsSetColorAlpha (1.0, 0.0, 0.0, 0.8);
  dsDrawBox (p, robot.Body(0).getRotation(), sides);

  std::copy(robot.Body(1).getPosition(),robot.Body(1).getPosition()+3,p);
  p[2]+=0.2;
  dsSetColorAlpha (0.0, 0.0, 1.0, 0.8);
  dsDrawBox (p, robot.Body(1).getRotation(), sides);
}

void test_keyevent(xode::TEnvironment &env, xode::TDynRobot &robot, int command)
{
  const double d(0.05);
  const int idx(1);
  if(command=='r')
  {
    Target1[0]= -0.5;
    Target1[1]= 0.0;
    Target1[2]= 0.0;
    Count= 0;
    env.Reset();
  }

  else if(command=='z')  Target1[0]= robot.Body(idx).getPosition()[0] + d;
  else if(command=='c')  Target1[0]= robot.Body(idx).getPosition()[0] - d;
  else if(command=='x')  Target1[1]= robot.Body(idx).getPosition()[1] + d;
  else if(command=='s')  Target1[1]= robot.Body(idx).getPosition()[1] - d;
  else if(command=='a')  Target1[2]= get_theta(robot,idx) + d;
  else if(command=='d')  Target1[2]= get_theta(robot,idx) - d;
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
