#include "naoconfig.h"
#include <alproxies/almotionproxy.h>
int main()
{
  AL::ALMotionProxy proxy_motion(ROBOT_IP,ROBOT_PORT);
  proxy_motion.stiffnessInterpolation("Body", 1.0, 1.0);
  proxy_motion.walkTo(0.1, 0.0, 0.0);
  return 0;
}
