#include "naoconfig.h"
#include <alproxies/almotionproxy.h>
using namespace std;
int main(int argc, char**argv)
{
  AL::ALMotionProxy proxy_motion(ROBOT_IP,ROBOT_PORT);
  string names  = "Body";
  double stiffness_lists  = 1.0;  // NOTE: it seems not working in Choregraphe
  double time_lists  = 1.0;
  proxy_motion.stiffnessInterpolation(names, stiffness_lists, time_lists);

  proxy_motion.stiffnessInterpolation(names, 0.0, time_lists);
  return 0;
}
