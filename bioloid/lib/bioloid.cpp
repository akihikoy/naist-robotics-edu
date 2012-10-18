//-------------------------------------------------------------------------------------------
/*! \file    bioloid.cpp
    \brief   ROBOTIS bioloid control library (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \version 0.1
    \date    Mar.08, 2010
*/
//-------------------------------------------------------------------------------------------
#include <bioloid.h>
#include <linux/serial.h>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace bioloid
{
using namespace std;
// using namespace boost;
using namespace serial;


static inline unsigned char* V2UC(void *p)
{
  return static_cast<unsigned char*>(p);
}
static inline const unsigned char* V2UC(const void *p)
{
  return static_cast<const unsigned char*>(p);
}
//-------------------------------------------------------------------------------------------

//! Get clock in [ms]
static inline long GetClock()
{
  struct timeval tv;
  gettimeofday (&tv, NULL);
  return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
}
//-------------------------------------------------------------------------------------------

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
// class TBioloidSerial : public serial::TSerialCom
//===========================================================================================
/*override*/bool TBioloidSerial::Open(void)
{
  struct termios newtio;
  struct serial_struct serinfo;

  memset(&newtio, 0, sizeof(newtio));
  Close();

  if((fd = open(s_tty.c_str(), O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0)
  {
    LERROR("Device open error: "<<s_tty.c_str());
    Close();
    return false;
  }

  newtio.c_cflag    = B38400|CS8|CLOCAL|CREAD;
  newtio.c_iflag    = IGNPAR;
  newtio.c_oflag    = 0;
  newtio.c_lflag    = 0;
  newtio.c_cc[VTIME]  = 0;
  newtio.c_cc[VMIN]  = 0;

  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);

  if(fd == -1)
    return false;

  if(ioctl(fd, TIOCGSERIAL, &serinfo) < 0)
  {
    LERROR("Cannot get serial info");
    return false;
  }

  serinfo.flags &= ~ASYNC_SPD_MASK;
  serinfo.flags |= ASYNC_SPD_CUST;
  serinfo.custom_divisor = serinfo.baud_base / baudrate;

  if(ioctl(fd, TIOCSSERIAL, &serinfo) < 0)
  {
    LERROR("Cannot set serial info");
    return false;
  }

  Close();

  byte_trans_time= (float)((1000.0f / baudrate) * 12.0f);
// LDBGVAR(byte_trans_time);

  memset(&newtio, 0, sizeof(newtio));
  Close();

  if((fd = open(s_tty.c_str(), O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0)
  {
    LERROR("device open error: "<<s_tty.c_str());
    Close();
    return false;
  }

  newtio.c_cflag    = B38400|CS8|CLOCAL|CREAD;
  newtio.c_iflag    = IGNPAR;
  newtio.c_oflag    = 0;
  newtio.c_lflag    = 0;
  newtio.c_cc[VTIME]  = 0;
  newtio.c_cc[VMIN]  = 0;

  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);

  status= COMM_RXSUCCESS;

  SetTimeout(6);

  return true;
}
//-------------------------------------------------------------------------------------------

/*override*/int TBioloidSerial::Write(const void *buff, size_t size)
{
  if(status == COMM_RXTIMEOUT || status == COMM_RXCORRUPT)
    Clear();
//*dbg*/while(!CheckTimeout())
//*dbg*/{/*usleep(100);*/}

  int num= TSerialCom::Write(buff, size);

  if((int)size != num)
  {
    status = COMM_TXFAIL;
    return num;
  }

  if(V2UC(buff)[4]==0x02)  // buff[4]:instruction == read(0x02)
    SetTimeout(V2UC(buff)[6] + 6);  // buff[6]:length of data to be read
  else
    SetTimeout(6);

  status= COMM_TXSUCCESS;

  if(V2UC(buff)[2] == 0xfe)  // buff[2]:id == broadcast-id(0xfe)
    status= COMM_RXSUCCESS;

  return num;
}
//-------------------------------------------------------------------------------------------

int TBioloidSerial::low_read(void *buff)
{
  unsigned char i, j, num;

  if(status == COMM_TXSUCCESS)
  {
    read_base= 0;
    read_size= 6;
  }

  num= TSerialCom::Read(V2UC(buff)+read_base, read_size-read_base);
  read_base+= num;
  if(read_base < read_size)
  {
    if(CheckTimeout())
    {
      if(read_base == 0)
        status= COMM_RXTIMEOUT;
      else
        status= COMM_RXCORRUPT;
      return read_base;
    }
  }

  // find a correct packet header
  for(i=0; i<(read_base-1); i++)
  {
    if(V2UC(buff)[i] == 0xff && V2UC(buff)[i+1] == 0xff)
      break;
    else if(i == read_base-2 && V2UC(buff)[read_base-1] == 0xff)
      break;
  }
  if(i > 0)
  {
    for(j=0; j<(read_base-i); ++j)
      V2UC(buff)[j] = V2UC(buff)[j + i];

    read_base-= i;
  }

  if(read_base < read_size)
  {
    status= COMM_RXWAITING;
    return read_base;
  }

  read_size= V2UC(buff)[3] + 4;  // buff[3]:length
  if(read_base < read_size)
  {
    num = TSerialCom::Read(V2UC(buff)+read_base, read_size-read_base);
    read_base+= num;
    if(read_base < read_size)
    {
      status= COMM_RXWAITING;
      return read_base;
    }
  }

  // checksum
  unsigned char checksum= 0;
  for(i=0; i<(V2UC(buff)[3]+1); ++i)  // buff[3]:length
    checksum+= V2UC(buff)[i+2];
  checksum= ~checksum;

  if(V2UC(buff)[V2UC(buff)[3]+3] != checksum)  // buff[3]:length
  {
    status= COMM_RXCORRUPT;
    return read_base;
  }

  status= COMM_RXSUCCESS;
  return read_base;
}
//-------------------------------------------------------------------------------------------

//! size is never used
/*override*/int TBioloidSerial::Read(void *buff, size_t size)
{
  if(status != COMM_TXSUCCESS)
    return -1;

  int num(0);
  do
  {
    num= low_read(buff);
//*dbg*/LWARNING(status);
  } while(status==COMM_RXWAITING);
  return num;
}
//-------------------------------------------------------------------------------------------

//! rcv_size: receive size in byte
/*virtual*/void TBioloidSerial::SetTimeout(size_t rcv_size)
{
  start_time= GetClock();
  rcv_wait_time= (float)(byte_trans_time*(float)rcv_size + 5.0f);
// LDBGVAR(byte_trans_time);
// LDBGVAR(rcv_size);
// LDBGVAR(rcv_wait_time);
}
//-------------------------------------------------------------------------------------------

/*virtual*/bool TBioloidSerial::CheckTimeout(void)
{
  long time;

  time= GetClock() - start_time;

  if(time > rcv_wait_time)
    return true;
  else if(time < 0)
    start_time= GetClock();

  return false;
}
//-------------------------------------------------------------------------------------------



//===========================================================================================
// class TBioloidController
//===========================================================================================

//! using TSerialCom for serial communication (CM-5,(CM-500))
void TBioloidController::Connect (const std::string &v_tty, const termios &v_ios)
{
  if (serial_std_.IsOpen())  serial_std_.Close();
  serial_std_.setting(v_tty,v_ios);
  serial_std_.Open();
  serial_type_= sctStandard;
  serial_= &serial_std_;
}
//-------------------------------------------------------------------------------------------
//! using TBioloidSerial for serial communication (USB2Dynamixel)
void TBioloidController::ConnectBS (const std::string &v_tty, int baudnum)
{
  if (serial_bio_.IsOpen())  serial_bio_.Close();
  serial_bio_.setting(v_tty,baudnum);
  serial_bio_.Open();
  serial_type_= sctBioloid;
  serial_= &serial_bio_;
}
//-------------------------------------------------------------------------------------------

void TBioloidController::Disconnect ()
{
  if (serial_!=NULL && serial_->IsOpen())  serial_->Close();
  serial_= NULL;
  serial_type_= sctNone;
}
//-------------------------------------------------------------------------------------------

//!\brief read from where the data starts with 0xffff
int TBioloidController::ReadFromFFFF (unsigned char *buf, int N, int max_trial, const int pr_max_trial)
{
  LASSERT(serial_!=NULL);
  int dn,offset(N);
  for(;max_trial>0;--max_trial)
  {
    dn= serial_->PersistingRead (buf+N-offset, offset, pr_max_trial);
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
  if(serial_type_==sctStandard)
  {
    // int N= serial_std_.Read (buffer_, 4);
    // int N= serial_std_.PersistingRead (buffer_, 4);
    int N= ReadFromFFFF (buffer_, 4);
    if (N!=4 || buffer_[0]!=0xff || buffer_[1]!=0xff || buffer_[2]!=id)
    {
      int Nr= serial_std_.Read(buffer_+N, SIZE_OF_ARRAY(buffer_)-N);
      LERROR("in reading a status packet from dynamixels: "<<PrintBuffer(buffer_,N)<<";"<<PrintBuffer(buffer_+N,Nr));
      LDBGVAR(N);
      LDBGVAR((int)id);
      return -1;
    }
    // N= serial_std_.Read (buffer_+4, buffer_[3])+4;
    N= serial_std_.PersistingRead (buffer_+4, buffer_[3])+4;
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
  else if (serial_type_==sctBioloid)
  {
    return serial_bio_.Read(buffer_);
  }
  else
  {
    LERROR("serial communication is not configured!");
    lexit(df);
  }
  return 0;
}
//-------------------------------------------------------------------------------------------

void TBioloidController::ReadAndEcho (int N)
{
  LASSERT(serial_!=NULL);
  // memset(buffer_,0,sizeof(buffer_));
  N= serial_->Read (buffer_, N);
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
  LASSERT(serial_!=NULL);
  str_writes_<<"t\n";
  str_writes_>>*serial_;
  /*dbg*/ReadAndEcho();
}
//-------------------------------------------------------------------------------------------

//! LED of (#1,#2) changes (on,off),(off,on),(on,off),(off,on),...
void TBioloidController::TossTest (void)
{
  LASSERT(serial_!=NULL);
  for (int i(0);i<20;++i)
  {
    sync_writes_.Init (0x19, 1);
    sync_writes_<<1<<1;
    sync_writes_<<2<<0;
    sync_writes_>>*serial_;
    usleep(100000);
    sync_writes_.Init (0x19, 1);
    sync_writes_<<1<<0;
    sync_writes_<<2<<1;
    sync_writes_>>*serial_;
    usleep(100000);
  }
}
//-------------------------------------------------------------------------------------------

int TBioloidController::Ping (unsigned char id)
{
  LASSERT(serial_!=NULL);
  biol_writes_.Init(id);
  biol_writes_<<0x01;
  biol_writes_>>*serial_;

  int N= ReadStatusPacket(id);
  if (N>0)
  {
    std::cerr<<"Ping("<<(int)id<<"): read "<<N<<" bytes: "<<PrintBuffer(buffer_,N)<<std::endl;
    return buffer_[4];
  }
  return -1;
}
//-------------------------------------------------------------------------------------------

//! state:0:off, 1:on
void TBioloidController::LED (unsigned char id, unsigned int state)
{
  LASSERT(serial_!=NULL);
  biol_writes_.Init(id);
  biol_writes_<<0x03<<0x19<<state;
  biol_writes_>>*serial_;

  int N= ReadStatusPacket(id);
  if (N>0)
  {
    std::cerr<<"LED("<<(int)id<<"): read "<<N<<" bytes: "<<PrintBuffer(buffer_,N)<<std::endl;
  }
}
//-------------------------------------------------------------------------------------------

void TBioloidController::SetLightDetectCompare (unsigned char id, unsigned char value)
{
  LASSERT(serial_!=NULL);
  biol_writes_.Init(id);
  biol_writes_<<0x03<<0x35<<value;
  biol_writes_>>*serial_;

  int N= ReadStatusPacket(id);
  if (N>0)
  {
    std::cerr<<"SetLightDetectCompare("<<(int)id<<"): read "<<N<<" bytes: "<<PrintBuffer(buffer_,N)<<std::endl;
  }
}
//-------------------------------------------------------------------------------------------

bool TBioloidController::EnableTorque (unsigned char id)
{
  LASSERT(serial_!=NULL);
  biol_writes_.Init(id);
  biol_writes_<<0x03<<0x18<<1;
  biol_writes_>>*serial_;

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
  LASSERT(serial_!=NULL);
  biol_writes_.Init(id);
  biol_writes_<<0x02<<0x24<<2;
  biol_writes_>>*serial_;

  int N= ReadStatusPacket(id);
  #if 0
  static int ERR(0),TOTAL(0);
  if (N>0)  std::cerr<<"read "<<N<<" bytes: "<<PrintBuffer(buffer_,N)<<std::endl;
  else ++ERR;
  ++TOTAL;
  std::cerr<<"GetAngle: err= "<<(double)ERR/(double)TOTAL*100.0<<"%"<<std::endl;
  #endif
  angle= CommandToDegree<double>(buffer_[5],buffer_[6]);

  if(serial_type_==sctBioloid)
  {
    if(serial_bio_.GetStatus()!=TBioloidSerial::COMM_RXSUCCESS)  return 0;
//*dbg*/if(angle>50) std::cerr<<"read "<<N<<" bytes: "<<PrintBuffer(buffer_,N)<<std::endl;
//*dbg*/if(angle>50) LDBGVAR(serial_bio_.GetStatus());
//*dbg*/if(angle>50) LDBGVAR(angle);
//*dbg*/if(angle>50) exit(1);
  }

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
  LASSERT(serial_!=NULL);
  if (sensor_pos<-1 || sensor_pos>1)
    {LERROR("in GetDistance: invalid sensor_pos: "<<sensor_pos);  return -1;}
  biol_writes_.Init(id);
  biol_writes_<<0x02<<(0x1B+sensor_pos)<<1;
  biol_writes_>>*serial_;

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
