#include "naoconfig.h"
#include <alproxies/almotionproxy.h>
using namespace std;
int main()
{
  AL::ALMotionProxy proxy_motion(ROBOT_IP,ROBOT_PORT);
  proxy_motion.stiffnessInterpolation("Body", 1.0, 1.0);
  AL::ALValue names= AL::ALValue::array("HeadYaw","HeadPitch");
  AL::ALValue angle_lists;
  angle_lists.arraySetSize(2);
  angle_lists[0]= AL::ALValue::array(0.0, 1.0);
  angle_lists[1]= AL::ALValue::array(0.0, -0.5);
  AL::ALValue time_lists;
  time_lists.arraySetSize(2);
  time_lists[0]= AL::ALValue::array(1.0, 2.0);
  time_lists[1]= AL::ALValue::array(1.0, 2.0);
  proxy_motion.angleInterpolation(names, angle_lists, time_lists, true);
  return 0;
}
