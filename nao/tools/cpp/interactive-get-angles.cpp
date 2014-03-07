#include "naoconfig.h"
#include <alproxies/almotionproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <iostream>
using namespace std;
int main()
{
  AL::ALMotionProxy proxy_motion(ROBOT_IP,ROBOT_PORT);
  proxy_motion.stiffnessInterpolation("Body", 1.0, 1.0);
  AL::ALTextToSpeechProxy proxy_audio(ROBOT_IP,ROBOT_PORT);

  // init pose:
  proxy_audio.say("I will stand up.");
  std::vector<std::string> names_body= proxy_motion.getJointNames("Body");
  AL::ALValue angle_lists, time_lists;
  angle_lists.arraySetSize(names_body.size());
  time_lists.arraySetSize(names_body.size());
  for(size_t j(0);j<names_body.size();++j)
  {
    angle_lists[j] = 0.0f;
    time_lists[j] = 2.0f;
    if(names_body[j]=="LShoulderPitch"
      || names_body[j]=="RShoulderPitch")  angle_lists[j] = 1.57f;
  }
  proxy_motion.angleInterpolation("Body", angle_lists, time_lists, true);

  string names= "RArm";
  proxy_audio.say("Right arm stiffness is removed. Hold on the right arm, please.");
  proxy_motion.stiffnessInterpolation(names, 0.0, 1.0);

  proxy_audio.post.say("Move the right arm, then type something and press the enter key.");
  while(true)
  {
    string tmp;
    cout<<" type something & press enter > ";
    cin>>tmp;
    if(tmp=="q" || tmp=="Q")  break;

    proxy_audio.post.say("Measuring the angles.");
    bool use_sensors= true; // true: using sensors, false: using commands
    vector<float> angles = proxy_motion.getAngles(names, use_sensors);
    cout << "Angles of " << names << ":" << endl << angles << endl;

    proxy_audio.post.say("Okey. Type Q to exit. Otherwise, I will continue to measure.");
  }

  proxy_audio.say("Done.");

  // Sitting pose:
  proxy_audio.say("I will sit down.");
  float angles_sit[]= {-0.00924587, 0.0429101, 1.4726, 0.144154, 0.00609398, -0.0475121, -0.0291879, 0.0268, -0.19631, 0.098218, -0.782298, 2.11255, -1.18944, -0.055182, -0.19631, 0.251618, -0.750168, 2.11255, -1.1863, -0.0889301, 1.39598, -0.142704, 0.400332, 0.299172, 0.0919981, 0.9148};
  for(size_t j(0);j<names_body.size();++j)  angle_lists[j]= angles_sit[j];
  proxy_motion.angleInterpolation("Body", angle_lists, time_lists, true);
  // Stiff off:
  proxy_audio.say("Stiffness removed.");
  proxy_motion.stiffnessInterpolation("Body", 0.0, 1.0);

  proxy_audio.say("Bye bye.");
  return 0;
}
