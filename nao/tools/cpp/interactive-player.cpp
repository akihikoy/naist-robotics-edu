#include "naoconfig.h"
#include <alproxies/almotionproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <list>
using namespace std;
struct TSequence
{
  double Time;
  double Angles[32];  // enough for NAO
};
int main(int argc,const char**argv)
{
  if(argc<=1)
  {
    cerr<<"usage: ./interactive-player.out FILE_NAME"<<endl;
    return 1;
  }
  string file_name= argv[1];
  double step_size= 0.1;
  string names= "RArm";
  int dof= 5;

  // Load motion from file
  /* file format is like:
  RArm 5 0.5
  1.57 0 0 0 0
  1.0 0 0 0 0
  0.5 0 0 0 0
  1.0 0 0 0 0
  1.57 0 0 0 0
  ...
  */
  list<TSequence> motion_data;
  {
    ifstream ifs(file_name.c_str());
    string line;
    getline(ifs,line,'\n');
    stringstream ss(line);
    ss>>names;
    ss>>dof;
    ss>>step_size;

    double time(1.0);
    while(getline(ifs,line,'\n'))
    {
      TSequence seq;
      seq.Time= time;
      stringstream ss(line);
      for(int j(0);j<dof;++j)  ss>>seq.Angles[j];
      motion_data.push_back(seq);
      time+= step_size;
    }
  }

  AL::ALMotionProxy proxy_motion(ROBOT_IP,ROBOT_PORT);
  AL::ALTextToSpeechProxy proxy_audio(ROBOT_IP,ROBOT_PORT);

  // Copy motion_data to Qi's data structure
  std::vector<std::string> names_detail= proxy_motion.getJointNames(names);
  AL::ALValue angle_lists;
  AL::ALValue time_lists;
  angle_lists.arraySetSize(names_detail.size());
  time_lists.arraySetSize(names_detail.size());

  for(int j(0);j<(int)names_detail.size();++j)
  {
    angle_lists[j].arraySetSize(motion_data.size());
    time_lists[j].arraySetSize(motion_data.size());
    if(j<dof)
    {
      int t(0);
      for(list<TSequence>::const_iterator itr(motion_data.begin()),last(motion_data.end());itr!=last;++itr,++t)
      {
        angle_lists[j][t]= itr->Angles[j];
        time_lists[j][t]= itr->Time;
      }
    }
    else
    {
      int t(0);
      for(list<TSequence>::const_iterator itr(motion_data.begin()),last(motion_data.end());itr!=last;++itr,++t)
      {
        angle_lists[j][t]= 0.0;
        time_lists[j][t]= itr->Time;
      }
    }
  }
  cerr<<"names:"<<names<<endl;
  cerr<<"step size:"<<step_size<<endl;
  cerr<<"motion-dof:"<<dof<<endl;
  cerr<<names<<"-dof:"<<names_detail.size()<<endl;
  cerr<<"num of poses:"<<motion_data.size()<<endl;
  cerr<<"angle_lists:"<<endl<<angle_lists<<endl;
  cerr<<"time_lists:"<<endl<<time_lists<<endl;

  proxy_motion.stiffnessInterpolation("Body", 1.0, 1.0);

  // init pose:
  proxy_audio.say("I will stand up.");
  {
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
  }

  proxy_audio.say(names+" stiffness is removed. Hold on "+names+", please.");
  proxy_motion.stiffnessInterpolation(names, 0.0, 1.0);

  proxy_audio.post.say("Move "+names+" to the start position, then type something and press the enter key.");
  string tmp;
  cout<<" type something & press enter > ";
  cin>>tmp;

  proxy_motion.stiffnessInterpolation(names, 1.0, 1.0);
  proxy_audio.say("I will play the motion.");

  proxy_motion.angleInterpolation(names, angle_lists, time_lists, true);

  proxy_audio.say("Done.");

  proxy_audio.say("Bye bye.");
  return 0;
}
