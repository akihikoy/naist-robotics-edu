#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#include "naoconfig.h"
#include <alproxies/almotionproxy.h>

#include <cmath>

#define JOY_DEV "/dev/input/js0"

using namespace std;

inline void constrain(double &x, const double &min=-1.0, const double &max=1.0)
{
  if(x>max)  x= max;
  else if(x<min)  x= min;
}

int main()
{
  int joy_fd(-1), num_of_axis(0), num_of_buttons(0);
  char name_of_joystick[80];
  vector<char> joy_button;
  vector<int> joy_axis;

  if((joy_fd=open(JOY_DEV,O_RDONLY)) < 0)
  {
    cerr<<"Failed to open "<<JOY_DEV<<endl;
    return -1;
  }

  ioctl(joy_fd, JSIOCGAXES, &num_of_axis);
  ioctl(joy_fd, JSIOCGBUTTONS, &num_of_buttons);
  ioctl(joy_fd, JSIOCGNAME(80), &name_of_joystick);

  joy_button.resize(num_of_buttons,0);
  joy_axis.resize(num_of_axis,0);

  cout<<"Joystick: "<<name_of_joystick<<endl
    <<"  axis: "<<num_of_axis<<endl
    <<"  buttons: "<<num_of_buttons<<endl;

  fcntl(joy_fd, F_SETFL, O_NONBLOCK);   // using non-blocking mode

  // setup nao
  AL::ALMotionProxy proxy_motion(ROBOT_IP,ROBOT_PORT);
  proxy_motion.stiffnessInterpolation("Body", 1.0, 1.0);

  int i(0), ctrl_cycle(500);
  while(true)
  {
    js_event js;

    read(joy_fd, &js, sizeof(js_event));

    switch (js.type & ~JS_EVENT_INIT)
    {
    case JS_EVENT_AXIS:
      if((int)js.number>=joy_axis.size())  {cerr<<"err:"<<(int)js.number<<endl; continue;}
      joy_axis[(int)js.number]= js.value;
      break;
    case JS_EVENT_BUTTON:
      if((int)js.number>=joy_button.size())  {cerr<<"err:"<<(int)js.number<<endl; continue;}
      joy_button[(int)js.number]= js.value;
      break;
    }

    cout<<"axis/10000: ";
    for(size_t i(0);i<joy_axis.size();++i)
      cout<<" "<<setw(2)<<joy_axis[i]/10000;
    cout<<endl;

    cout<<"  button: ";
    for(size_t i(0);i<joy_button.size();++i)
      cout<<" "<<(int)joy_button[i];
    cout<<endl;

    ++i;
    if(i==ctrl_cycle)
    {
      double x_spd= -double(joy_axis[1])/30000.0;
      double y_spd= -double(joy_axis[0])/30000.0;
      double rot_spd= -double(joy_axis[2])/30000.0;
      double frequency= 1.0;
      constrain(x_spd);
      constrain(y_spd);
      constrain(rot_spd);
      if(fabs(x_spd)<0.1 || fabs(y_spd)<0.1 || fabs(rot_spd)<0.1)
      {
        proxy_motion.setWalkTargetVelocity(x_spd, y_spd, rot_spd, frequency);
      }

      i= 0;
    }

    usleep(100);
  }

  close(joy_fd);
  return 0;
}
