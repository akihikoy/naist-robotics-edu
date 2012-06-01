#include "naoconfig.h"
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/almotionproxy.h>
using namespace std;

int main()
{
  AL::ALTextToSpeechProxy proxy_audio(ROBOT_IP,ROBOT_PORT);
  AL::ALMotionProxy proxy_motion(ROBOT_IP,ROBOT_PORT);
  proxy_motion.stiffnessInterpolation("Body", 1.0, 1.0);

  proxy_motion.angleInterpolation("HeadYaw", 0.0, 0.5, true);
  proxy_audio.post.say("Hello Garchia, my name is NAO. How are you?");
  proxy_motion.angleInterpolation("HeadYaw", 1.2, 1.0, true);
  proxy_motion.angleInterpolation("HeadYaw", -1.2, 1.0, true);
  proxy_motion.angleInterpolation("HeadYaw", 1.2, 1.0, true);
  proxy_motion.angleInterpolation("HeadYaw", -1.2, 1.0, true);
  return 0;
}
