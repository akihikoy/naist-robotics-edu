#include "naoconfig.h"
#include <alproxies/almotionproxy.h>
using namespace std;
int main()
{
  AL::ALMotionProxy proxy_motion(ROBOT_IP,ROBOT_PORT);
  proxy_motion.stiffnessInterpolation("Body", 1.0, 1.0);
  proxy_motion.angleInterpolation("HeadYaw", -1.2, 1.0, true);
  proxy_motion.angleInterpolation("HeadYaw", 1.2, 1.0, true);
  return 0;
}
