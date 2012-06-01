#include "naoconfig.h"
#include <alproxies/almotionproxy.h>
using namespace std;
int main()
{
  AL::ALMotionProxy proxy_motion(ROBOT_IP,ROBOT_PORT);
  proxy_motion.stiffnessInterpolation("Body", 1.0, 1.0);
  proxy_motion.angleInterpolation(AL::ALValue::array("HeadYaw","HeadPitch"), AL::ALValue::array(0.0, 0.0), AL::ALValue::array(1.0, 1.0), true);
  AL::ALValue names= AL::ALValue::array("HeadYaw","HeadPitch");
  AL::ALValue angle_lists= AL::ALValue::array(1.0, -0.5);
  AL::ALValue time_lists= AL::ALValue::array(1.0, 1.0);
  proxy_motion.angleInterpolation(names, angle_lists, time_lists, true);
  return 0;
}
