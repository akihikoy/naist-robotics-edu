#include "naoconfig.h"
#include <alproxies/almotionproxy.h>
using namespace std;
int main()
{
  AL::ALMotionProxy proxy_motion(ROBOT_IP,ROBOT_PORT);
  proxy_motion.stiffnessInterpolation("Body", 1.0, 1.0);
  std::vector<std::string> names= proxy_motion.getJointNames("Body");

  AL::ALValue angle_lists, time_lists;
  angle_lists.arraySetSize(names.size());
  time_lists.arraySetSize(names.size());
  for(size_t j(0);j<names.size();++j)
  {
    angle_lists[j] = 0.0f;
    time_lists[j] = 2.0f;
  }

  proxy_motion.angleInterpolation("Body", angle_lists, time_lists, true);
  return 0;
}
