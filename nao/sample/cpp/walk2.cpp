#include "naoconfig.h"
#include <alproxies/almotionproxy.h>
int main()
{
  AL::ALMotionProxy proxy_motion(ROBOT_IP,ROBOT_PORT);
  proxy_motion.stiffnessInterpolation("Body", 1.0, 1.0);
  double frequency= 1.0;
  proxy_motion.setWalkTargetVelocity(0.1, 0.0, 0.0, frequency);
  usleep(1000*1000);
  proxy_motion.setWalkTargetVelocity(0.0, 0.0, 0.0, frequency);
  return 0;
}
