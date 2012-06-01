//-------------------------------------------------------------------------------------------
/*! \file    test.cpp
    \brief   test program to control the real nao
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \version 0.2
    \date    May.13, 2011
    \date    Jun.1, 2012
*/
//-------------------------------------------------------------------------------------------
#include "naoconfig.h"
#include <alcommon/alproxy.h>
#include <alcore/alptr.h>
// #include <alproxies/almotionproxy.h>
//-------------------------------------------------------------------------------------------
using namespace std;

int main(int argc, char**argv)
{

  AL::ALPtr<AL::ALProxy> proxy_audio(new AL::ALProxy("ALTextToSpeech",ROBOT_IP,ROBOT_PORT));
  proxy_audio->callVoid("say",string("I am controlled by C++"));


  AL::ALPtr<AL::ALProxy> proxy_motion(new AL::ALProxy("ALMotion",ROBOT_IP,ROBOT_PORT));
          // pointer automatically destroyed after try block
          // or ALProxy *proxy_motion = new ALProxy("ALMotion",ROBOT_IP,ROBOT_PORT);
          // don't forget to delete proxy manually

  {
    // Example showing how to interpolate to maximum stiffness in 1 second
    string names  = "Body";
    double stiffness_lists  = 1.0;  // NOTE: it seems not working in Choregraphe
    double time_lists  = 1.0;
    proxy_motion->callVoid("stiffnessInterpolation", names, stiffness_lists, time_lists);
  }

  {
    // Example showing how to set angles, using a fraction of max speed
    vector<string> tmp_names;
    tmp_names.push_back("HeadYaw");
    tmp_names.push_back("HeadPitch");
    AL::ALValue names(tmp_names);

    vector<float> tmp_angles;
    tmp_angles.push_back(-1.0);
    tmp_angles.push_back(-0.2);
    AL::ALValue angles(tmp_angles);

    double fractionMaxSpeed  = 0.2;
    proxy_motion->callVoid("setAngles", names, angles, fractionMaxSpeed);
  }

  {
    // Example showing a single target angle for one joint
    // Interpolate the head yaw to 1.0 radian in 1.0 second
    vector<string> tmp_names;
    tmp_names.push_back("HeadYaw");
    tmp_names.push_back("HeadPitch");
    AL::ALValue names(tmp_names);

    AL::ALValue angles;
    angles.arraySetSize(2);
    angles[0] = AL::ALValue::array(1.0f, 0.0f);
    angles[1] = AL::ALValue::array(-0.5f, 0.5f, 0.0f);

    AL::ALValue times;
    times.arraySetSize(2);
    times[0] = AL::ALValue::array(1.0f, 2.0f);
    times[1] = AL::ALValue::array(1.0f, 2.0f, 3.0f);

    bool isAbsolute  = true;
    proxy_motion->callVoid("angleInterpolation", names, angles, times, isAbsolute);
  }

  return 0;
}
//-------------------------------------------------------------------------------------------
