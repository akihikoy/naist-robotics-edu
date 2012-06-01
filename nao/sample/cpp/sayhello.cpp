// #include "naoconfig.h"
#include <alcommon/alproxy.h>
#include <alcore/alptr.h>
using namespace std;

static const char ROBOT_IP[]="192.168.1.10";
static const int ROBOT_PORT=9559;

int main()
{
  AL::ALPtr<AL::ALProxy> proxy_audio(new AL::ALProxy("ALTextToSpeech",ROBOT_IP,ROBOT_PORT));
  proxy_audio->callVoid("say",string("Hello world"));
  return 0;
}
