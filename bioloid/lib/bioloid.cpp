//-------------------------------------------------------------------------------------------
/*! \file    bioloid.cpp
    \brief   ROBOTIS bioloid control library (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \version 0.1
    \date    Mar.08, 2010
*/
//-------------------------------------------------------------------------------------------
#include <bioloid.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace bioloid
{
using namespace std;
// using namespace boost;
using namespace serial;


static unsigned char GetChecksum (unsigned char *beginitr, unsigned char *enditr)
{
  int sum(0);
  for(; beginitr!=enditr; ++beginitr)  sum+= *beginitr;
  return 255 - (sum % 256);
}
//-------------------------------------------------------------------------------------------

termios GetDefaultTermios (void)
{
  termios ios;
  memset(&ios, 0, sizeof(ios));
  // ios.c_cflag |= B115200 | CREAD | CLOCAL | CS8;  /* control mode flags */
  ios.c_cflag |= B57600 | CREAD | CLOCAL | CS8;  /* control mode flags */
  ios.c_iflag |= IGNPAR;  /* input mode flags */
  ios.c_cc[VMIN] = 0;
  ios.c_cc[VTIME] = 1;  // *[1/10 sec] for timeout
  return ios;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TBioloidController
//===========================================================================================

void TBioloidController::Connect (const std::string &v_tty, const termios &v_ios)
{
  if (serial_.IsOpen())  serial_.Close();
  serial_.setting(v_tty,v_ios);
  serial_.Open();
}
//-------------------------------------------------------------------------------------------

void TBioloidController::Disconnect ()
{
  if (serial_.IsOpen())  serial_.Close();
}
//-------------------------------------------------------------------------------------------

//!\brief read from where the data starts with 0xffff
int TBioloidController::ReadFromFFFF (unsigned char *buf, int N, int max_trial, const int pr_max_trial)
{
  int dn,offset(N);
  for(;max_trial>0;--max_trial)
  {
    dn= serial_.PersistingRead (buf+N-offset, offset, pr_max_trial);
    if(dn!=offset)  return N-offset+dn;
    for(offset=0;offset<N-1;++offset)
      if(buf[offset]==0xff && buf[offset+1]==0xff)  break;
    if(offset==0) return N;
    for(int i(0);i<N-offset;++i)
      buf[i]= buf[offset+i];
  }
  return offset;
}
//-------------------------------------------------------------------------------------------

//!\brief read a valid status packet from dynamixels
int TBioloidController::ReadStatusPacket (unsigned char id)
{
  // int N= serial_.Read (buffer_, 4);
  // int N= serial_.PersistingRead (buffer_, 4);
  int N= ReadFromFFFF (buffer_, 4);
  if (N!=4 || buffer_[0]!=0xff || buffer_[1]!=0xff || buffer_[2]!=id)
  {
    int Nr= serial_.Read(buffer_+N, SIZE_OF_ARRAY(buffer_)-N);
    LERROR("in reading a status packet from dynamixels: "<<PrintBuffer(buffer_,N)<<";"<<PrintBuffer(buffer_+N,Nr));
    LDBGVAR(N);
    LDBGVAR((int)id);
    return -1;
  }
  // N= serial_.Read (buffer_+4, buffer_[3])+4;
  N= serial_.PersistingRead (buffer_+4, buffer_[3])+4;
  if (N!=buffer_[3]+4 || buffer_[4]!=0 || GetChecksum(buffer_+2,buffer_+N-1)!=buffer_[N-1])
  {
    LERROR("in reading a status packet from dynamixels: "<<PrintBuffer(buffer_,N));
    // LASSERT1op1(N,==,buffer_[3]+4);
    // LASSERT1op1(buffer_[4],==,0);
    // LASSERT1op1(GetChecksum(buffer_+2,buffer_+N-1),==,buffer_[N-1]);
    return -1;
  }
  return N;
}
//-------------------------------------------------------------------------------------------

void TBioloidController::ReadAndEcho (int N)
{
  // memset(buffer_,0,sizeof(buffer_));
  N= serial_.Read (buffer_, N);
  const char *rbufptr(reinterpret_cast<const char*>(buffer_));
  for (int i(0);i<N;++i,++rbufptr)
    std::cerr<<*rbufptr;
    // cout<<EncodeChar2(*rbufptr);
  std::cerr<<std::endl;
}
//-------------------------------------------------------------------------------------------

//!\brief binary communicating mode
void TBioloidController::TossMode(void)
{
  str_writes_<<"t\n";
  str_writes_>>serial_;
  /*dbg*/ReadAndEcho();
}
//-------------------------------------------------------------------------------------------

//! LED of (#1,#2) changes (on,off),(off,on),(on,off),(off,on),...
void TBioloidController::TossTest (void)
{
  for (int i(0);i<20;++i)
  {
    sync_writes_.Init (0x19, 1);
    sync_writes_<<1<<1;
    sync_writes_<<2<<0;
    sync_writes_>>serial_;
    usleep(100000);
    sync_writes_.Init (0x19, 1);
    sync_writes_<<1<<0;
    sync_writes_<<2<<1;
    sync_writes_>>serial_;
    usleep(100000);
  }
}
//-------------------------------------------------------------------------------------------

void TBioloidController::SetLightDetectCompare (unsigned char id, unsigned char value)
{
  biol_writes_.Init(id);
  biol_writes_<<0x03<<0x35<<value;
  biol_writes_>>serial_;

  int N= ReadStatusPacket(id);
  if (N>0)
  {
    std::cerr<<"SetLightDetectCompare("<<(int)id<<"): read "<<N<<" bytes: "<<PrintBuffer(buffer_,N)<<std::endl;
  }
}
//-------------------------------------------------------------------------------------------

bool TBioloidController::EnableTorque (unsigned char id)
{
  biol_writes_.Init(id);
  biol_writes_<<0x03<<0x18<<1;
  biol_writes_>>serial_;

  int N= ReadStatusPacket(id);
  if (N>0)
  {
    std::cerr<<"read "<<N<<" bytes: "<<PrintBuffer(buffer_,N)<<std::endl;
    if (buffer_[5]==1)  {LMESSAGE("torque is enabled."); return true;}
  }
  return false;
}
//-------------------------------------------------------------------------------------------

int TBioloidController::GetAngle (unsigned char id, double &angle)
{
  biol_writes_.Init(id);
  biol_writes_<<0x02<<0x24<<2;
  biol_writes_>>serial_;

  int N= ReadStatusPacket(id);
  #if 0
  static int ERR(0),TOTAL(0);
  if (N>0)  std::cerr<<"read "<<N<<" bytes: "<<PrintBuffer(buffer_,N)<<std::endl;
  else ++ERR;
  ++TOTAL;
  std::cerr<<"GetAngle: err= "<<(double)ERR/(double)TOTAL*100.0<<"%"<<std::endl;
  #endif
  angle= CommandToDegree<double>(buffer_[5],buffer_[6]);
  return N;
}
//-------------------------------------------------------------------------------------------

int TBioloidController::PersistingGetAngle (unsigned char id, double &angle, int max_trial)
{
  int N(0);
  for(;max_trial>0;--max_trial)
  {
    if ((N=GetAngle(id,angle))>0)  break;
    // EnableTorque(id);
  }
  #if 0
  static int ERR(0),TOTAL(0);
  if (N>0)  std::cerr<<"read "<<N<<" bytes: "<<PrintBuffer(buffer_,N)<<std::endl;
  else ++ERR;
  ++TOTAL;
  std::cerr<<"GetAngle: err= "<<(double)ERR/(double)TOTAL*100.0<<"%"<<std::endl;
  #endif
  return N;
}
//-------------------------------------------------------------------------------------------

/*!\brief get distance to an object via IR-sensor
    \param [in]sensor_pos  : -1:left, 0:center, 1:right  */
int TBioloidController::GetDistance (unsigned char id, int sensor_pos, double &distance)
{
  if (sensor_pos<-1 || sensor_pos>1)
    {LERROR("in GetDistance: invalid sensor_pos: "<<sensor_pos);  return -1;}
  biol_writes_.Init(id);
  biol_writes_<<0x02<<(0x1B+sensor_pos)<<1;
  biol_writes_>>serial_;

  int N= ReadStatusPacket(id);
  #if 0
  static int ERR(0),TOTAL(0);
  if (N>0)  std::cerr<<"read "<<N<<" bytes: "<<PrintBuffer(buffer_,N)<<std::endl;
  else ++ERR;
  ++TOTAL;
  std::cerr<<"GetDistance: err= "<<(double)ERR/(double)TOTAL*100.0<<"%"<<std::endl;
  #endif
  // distance= static_cast<double>(256-buffer_[5]);
  const double offset(28.4);
  distance= (offset+255.0)/(offset+static_cast<double>(buffer_[5]));
    //!\warning: this conversion is incorrect!
  return N;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace bioloid
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
