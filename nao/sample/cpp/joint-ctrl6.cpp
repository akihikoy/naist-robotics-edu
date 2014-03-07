#include "naoconfig.h"
#include <alproxies/almotionproxy.h>
#include <cmath>
using namespace std;
struct TRArmSequence
{
  double Time;
  double Angles[6];
};
int main()
{
  AL::ALMotionProxy proxy_motion(ROBOT_IP,ROBOT_PORT);
  proxy_motion.stiffnessInterpolation("Body", 1.0, 1.0);

  // Prepare a sequence of target joint angles and time
  const int n_trg(100);
  TRArmSequence data[n_trg];
  data[0].Time= 1.0;
  for(int t(0);t<n_trg;++t)
  {
    if(t>0)  data[t].Time= data[t-1].Time+0.5;
    for(int j(0);j<6;++j)
      if(j<=2||j==5)  data[t].Angles[j]= sin(data[t].Time*2.0);
      else            data[t].Angles[j]= -sin(data[t].Time*2.0);
  }

  // Copy the sequence to Qi's data structure
  AL::ALValue angle_lists;
  angle_lists.arraySetSize(6);
  AL::ALValue time_lists;
  time_lists.arraySetSize(6);
  for(int j(0);j<6;++j)
  {
    angle_lists[j].arraySetSize(n_trg);
    time_lists[j].arraySetSize(n_trg);
    for(int t(0);t<n_trg;++t)
    {
      angle_lists[j][t]= data[t].Angles[j];
      time_lists[j][t]= data[t].Time;
    }
  }

  // Play
  proxy_motion.angleInterpolation("RArm", angle_lists, time_lists, true);
  // proxy_motion.post.angleInterpolation("RArm", angle_lists, time_lists, true);

  return 0;
}
