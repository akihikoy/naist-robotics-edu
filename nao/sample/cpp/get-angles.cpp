#include "naoconfig.h"
#include <alproxies/almotionproxy.h>
#include <iostream>
using namespace std;
int main()
{
  AL::ALMotionProxy proxy_motion(ROBOT_IP,ROBOT_PORT);
  // proxy_motion.stiffnessInterpolation("Body", 1.0, 1.0);

  string names= "Body";
  bool use_sensors= true; // true: using sensors, false: using commands

  while(true)
  {
    vector<float> angles = proxy_motion.getAngles(names, use_sensors);
    cout << "Angles: " << endl << angles << endl;
  }
  return 0;
}
