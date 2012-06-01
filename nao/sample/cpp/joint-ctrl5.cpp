#include "naoconfig.h"
#include <alproxies/almotionproxy.h>
using namespace std;
int main()
{
  AL::ALMotionProxy proxy_motion(ROBOT_IP,ROBOT_PORT);
  proxy_motion.stiffnessInterpolation("Body", 1.0, 1.0);

  AL::ALValue angle_lists;
  angle_lists.arraySetSize(6);
  for(int j(0);j<6;++j)
    if(j<=2||j==5)  angle_lists[j]= AL::ALValue::array(0.0, 1.0, 0.0);
    else            angle_lists[j]= AL::ALValue::array(0.0, -1.0, 0.0);
  AL::ALValue time_lists;
  time_lists.arraySetSize(6);
  for(int j(0);j<6;++j)
    time_lists[j]= AL::ALValue::array(1.0, 2.0, 3.0);
  proxy_motion.post.angleInterpolation("LArm", angle_lists, time_lists, true);

  for(int j(0);j<6;++j)
    if(j<=1)  angle_lists[j]= AL::ALValue::array(0.0, -1.0, 0.0);
    else      angle_lists[j]= AL::ALValue::array(0.0, 1.0, 0.0);
  proxy_motion.angleInterpolation("RArm", angle_lists, time_lists, true);

  return 0;
}
