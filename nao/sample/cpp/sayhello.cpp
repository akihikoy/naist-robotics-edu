#include "naoconfig.h"
#include <alproxies/altexttospeechproxy.h>
using namespace std;
int main()
{
  AL::ALTextToSpeechProxy proxy_audio(ROBOT_IP,ROBOT_PORT);
  proxy_audio.say("Hello world");
  return 0;
}
