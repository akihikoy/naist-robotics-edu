//-------------------------------------------------------------------------------------------
/*! \file    bioloid-test.cpp
    \brief   bioloid test program
    \author  Akihiko Yamaguchi
    \version 0.1
    \date    Mar.08, 2010
*/
//-------------------------------------------------------------------------------------------
#include <bioloid.h>
#include <fstream>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{

// inline double RadToDeg(const double &rad)
// {
  // return rad/M_PI*180.0;
// }

template <typename t_angle_fwditr>
inline void ConstrainAngles (t_angle_fwditr begin, t_angle_fwditr end)
{
  // const double angle_max[18]={ 0.0, 1.6, 1.6, 1.6, 1.6, 1.5,   1.57, 1.57, 1.6, 1.6, 1.6, 1.5,   1.6, 0.0, 1.6, 1.6, 1.6, 1.5};
  // const double angle_min[18]={-1.6,-0.0,-1.6,-1.6,-1.5,-1.6,  -1.57,-1.57,-1.6,-1.6,-1.5,-1.6,  -0.0,-1.6,-1.6,-1.6,-1.5,-1.6};
  const double angle_max[18]={  0,90, 90, 90, 90, 80,   90, 90, 90, 90, 90, 80,   90,  0, 90, 90, 90, 80};
  const double angle_min[18]={-90,-0,-90,-90,-80,-90,  -90,-90,-90,-90,-80,-90,   -0,-90,-90,-90,-80,-90};
  const double *amax(angle_max), *amin(angle_min);
  for (; begin!=end; ++begin,++amax,++amin)
  {
    if(*begin>*amax)  *begin= *amax;
    else if (*begin<*amin)  *begin= *amin;
  }
}

}
//-------------------------------------------------------------------------------------------
using namespace std;
using namespace loco_rabbits;
using namespace serial;
using namespace bioloid;
//-------------------------------------------------------------------------------------------
// #define print(var) print_container((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  TBioloidController  bioloid;
  bioloid.Connect("/dev/ttyUSB0");

  bioloid.TossMode();
  // bioloid.TossTest();

  bioloid.SetLightDetectCompare (100,128);

  // int ids[]= {1,2};
  int ids[]= {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
  double angles[SIZE_OF_ARRAY(ids)], angles_observed[SIZE_OF_ARRAY(ids)];

  ofstream ofs_goal("goal.dat");
  ofstream ofs_angle("angle.dat");
  int skip(0);
  double omega(2.0*M_PI*0.05), goal, distance_c;
  double time_offset(GetCurrentTime()), time(GetCurrentTime()), t;

  #if 0
  while (time-time_offset<2.0)
  {
    bioloid.GetAllAngles (ids,ids+SIZE_OF_ARRAY(ids), angles_observed);
    bioloid.GoTo (ids,ids+SIZE_OF_ARRAY(ids), angles);
    usleep(10000);
    time= GetCurrentTime();
  }
  #endif
  LMESSAGE("----");

  while (time-time_offset<60.0)
  {
    t= time-time_offset;
    // goal= 150.0*sin(omega*t);
    goal= 30.0*sin(omega*t);
    LMESSAGE(t<<"[s]: ("<<goal<<") ");

    #if 1
    // bioloid.GetAngle(1);
    if(skip==0)
    {
      bioloid.GetAllAngles (ids,ids+SIZE_OF_ARRAY(ids), angles_observed);
      ofs_angle<<t<<" "<<PrintVector(angles_observed,angles_observed+SIZE_OF_ARRAY(angles_observed))<<endl;
      // bioloid.GetAllAngles (ids,ids+5, angles_observed);
      // ofs_angle<<t<<" "<<PrintVector(angles_observed,angles_observed+5)<<endl;
      skip=0;
    } else{--skip;}
    std::fill (angles,angles+SIZE_OF_ARRAY(angles),goal);
    ConstrainAngles (angles,angles+SIZE_OF_ARRAY(angles));
    bioloid.GoTo (ids,ids+SIZE_OF_ARRAY(ids), angles);
    ofs_goal<<t<<" "<<PrintVector(angles,angles+SIZE_OF_ARRAY(angles))<<endl;
    #endif

    #if 1
    bioloid.GetDistance(100,0,distance_c);
    ofs_angle<<t<<" "<<distance_c<<endl;
    #endif

    // usleep(5000);
    // usleep(10000);
    // usleep(15000);
    // usleep(20000);
    // usleep(30000);
    // usleep(50000);
    // usleep(100000);
    time= GetCurrentTime();
  }

  ofs_goal.close();
  ofs_angle.close();

  bioloid.Disconnect();

  return 0;
}
//-------------------------------------------------------------------------------------------
