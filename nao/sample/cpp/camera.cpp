#include "naoconfig.h"
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alvisiondefinitions.h>
// #include <cv.h>
#include <highgui.h>
#include <iostream>
using namespace std;
int main()
{
  AL::ALVideoDeviceProxy proxy_camera(ROBOT_IP,ROBOT_PORT);

  string vm_name("my_video");
  int resolution(AL::kQVGA); // {kQQVGA, kQVGA, kVGA, k4VGA}
  int color_space(AL::kBGRColorSpace); // {kYuv, kYUV422, kYUV, kRGB, kHSY, kBGR}
  int fps(15);
  vm_name= proxy_camera.subscribe(vm_name, resolution, color_space, fps);

  AL::ALValue results;

  cv::namedWindow("camera",1);

  while(true)
  {
    cerr<<"capture.."<<endl;
    results= proxy_camera.getImageRemote(vm_name);
    int width= results[0];
    int height= results[1];
    const unsigned char *image_data=  static_cast<const unsigned char*>(results[6].GetBinary());
    cv::Mat frame(height, width, CV_8UC3, const_cast<unsigned char*>(image_data));

    cv::imshow("camera", frame);
    int c(cv::waitKey(10));
    if(c=='\x1b'||c=='q') break;
  }

  proxy_camera.releaseImage(vm_name);
  proxy_camera.unsubscribe(vm_name);

  return 0;
}
