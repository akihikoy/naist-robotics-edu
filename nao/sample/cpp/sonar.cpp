#include "naoconfig.h"
#include <alproxies/alsonarproxy.h>
#include <alproxies/almemoryproxy.h>
#include <iostream>
using namespace std;
int main()
{
  AL::ALSonarProxy proxy_sonar(ROBOT_IP,ROBOT_PORT);

  string sn_name("my_sonar");
  proxy_sonar.subscribe(sn_name);

  AL::ALMemoryProxy proxy_mem(ROBOT_IP,ROBOT_PORT);

  while(true)
  {
    AL::ALValue l= proxy_mem.getData("Device/SubDeviceList/US/Left/Sensor/Value");
    AL::ALValue r= proxy_mem.getData("Device/SubDeviceList/US/Right/Sensor/Value");

    cout<<l<<" : "<<r<<endl;
  }

  proxy_sonar.unsubscribe(sn_name);

  return 0;
}
