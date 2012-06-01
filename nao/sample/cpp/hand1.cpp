#include "naoconfig.h"
#include <alproxies/almotionproxy.h>
using namespace std;
int main()
{
  AL::ALMotionProxy proxy_motion(ROBOT_IP,ROBOT_PORT);

  proxy_motion.post.openHand("LHand");
  proxy_motion.openHand("RHand");

  proxy_motion.post.closeHand("LHand");
  proxy_motion.closeHand("RHand");

  return 0;
}
