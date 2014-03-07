#include "naoconfig.h"
#include <alproxies/almotionproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <iostream>
#include <cstdio>
using namespace std;
int main(int argc,const char**argv)
{
  AL::ALMotionProxy proxy_motion(ROBOT_IP,ROBOT_PORT);
  AL::ALTextToSpeechProxy proxy_audio(ROBOT_IP,ROBOT_PORT);
  proxy_motion.stiffnessInterpolation("Body", 1.0, 1.0);

  std::vector<std::string> names_body= proxy_motion.getJointNames("Body");
  AL::ALValue angle_lists, time_lists;
  angle_lists.arraySetSize(names_body.size());
  time_lists.arraySetSize(names_body.size());

  // Sitting pose:
  proxy_audio.post.say("I will sit down.");
  float angles_sit[]= {-0.00924587, 0.0429101, 1.4726, 0.144154, 0.00609398, -0.0475121, -0.0291879, 0.0268, -0.19631, 0.098218, -0.782298, 2.11255, -1.18944, -0.055182, -0.19631, 0.251618, -0.750168, 2.11255, -1.1863, -0.0889301, 1.39598, -0.142704, 0.400332, 0.299172, 0.0919981, 0.9148};
  for(size_t j(0);j<names_body.size();++j)
  {
    angle_lists[j]= angles_sit[j];
    time_lists[j] = 2.0f;
  }
  proxy_motion.angleInterpolation("Body", angle_lists, time_lists, true);

  // Stiff off:
  proxy_audio.post.say("Stiffness removed.");
  proxy_motion.stiffnessInterpolation("Body", 0.0, 1.0);

  proxy_audio.post.say("Bye bye.");
  return 0;
}
