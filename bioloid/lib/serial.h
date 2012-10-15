//-------------------------------------------------------------------------------------------
/*! \file    serial.h
    \brief   serial communication library (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \version 2.1
    \date    2008,2009,2010
*/
//-------------------------------------------------------------------------------------------
#ifndef serial_h
#define serial_h
//-------------------------------------------------------------------------------------------
#include <lora_small.h>
#include <serial_config.h>
#include <string>
#include <termios.h>
#include <iomanip>
//-------------------------------------------------------------------------------------------
#ifdef HAVE_LIBFTD2XX
  #include <libftd2xx/ftd2xx.h>
#endif
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace serial
{
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TComBase
//===========================================================================================
{
private:
protected:
public:
  TComBase(void)  {};
  virtual ~TComBase(void) {};

  virtual bool Open(void) = 0;
  virtual bool Close(void) = 0;
  virtual int Write(const void *buff, size_t size) = 0;
  virtual int Read(void *buff, size_t size) = 0;
  virtual int PersistingRead(unsigned char *buff, size_t size, int max_trial=10);
  virtual bool IsOpen(void) const = 0;
  virtual void Clear() {}
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TSerialCom : public TComBase
//===========================================================================================
{
protected:
  int fd;
  std::string s_tty;
  struct termios s_ios;

  TSerialCom(const TSerialCom&);
  const TSerialCom& operator=(const TSerialCom&);

public:
  TSerialCom(void) : TComBase(), fd(-1) {};
  ~TSerialCom(void) {};

  const int& getFd (void) const {return fd;};

  void setting(const std::string &_tty, const termios &_ios)
    {s_tty=_tty; s_ios=_ios;};
  override bool Open(void);
  override bool Close(void);
  override int Write(const void *buff, size_t size);
  override int Read(void *buff, size_t size);
  override bool IsOpen(void) const {return (fd>=0);};
  override void Clear();

};
//-------------------------------------------------------------------------------------------


//===========================================================================================
#ifdef HAVE_LIBFTD2XX
class TUSBSerial : public TComBase
//===========================================================================================
{
public:
  struct TUSBSerialConf
  {
    DWORD iVID;
    DWORD iPID;
    ULONG BaudRate;
    ULONG ReadTimeout;
    ULONG WriteTimeout;
    UCHAR WordLength;
    UCHAR StopBits;
    UCHAR Parity;
    TUSBSerialConf(void) :
      iVID(0x165c), iPID(0x0002), BaudRate(115200),
      ReadTimeout(1000), WriteTimeout(0),
      WordLength(8), StopBits(1), Parity(FT_PARITY_NONE)  {};
  };
private:
  FT_HANDLE       handle;
  int             device_num;
  TUSBSerialConf  conf;
  bool            is_opened;

  FT_STATUS iFT_ListDevices(PVOID pArg1, PVOID pArg2, DWORD Flags);
  FT_STATUS iFT_Open(int deviceNumber, FT_HANDLE *pHandle);
  FT_STATUS iFT_OpenEx(PVOID pArg1, DWORD Flags, FT_HANDLE *pHandle);

public:
  TUSBSerial(void) : TComBase(), device_num(0), conf(), is_opened(false) {};
  ~TUSBSerial(void) {};

  void setting(int dvn, const TUSBSerialConf &_conf)
    {device_num=dvn; conf=_conf;};
  override bool Open(void);
  override bool Close(void);
  override int Write(const void *buff, size_t size);
  override int Read(void *buff, size_t size);
  override bool IsOpen(void) const {return is_opened;};
};
//-------------------------------------------------------------------------------------------
#endif // HAVE_LIBFTD2XX
//-------------------------------------------------------------------------------------------


//===========================================================================================
// utility
//===========================================================================================

struct PrintBuffer
{
  const unsigned char *buffer;
  int length;
  PrintBuffer(const unsigned char *v_buffer,int v_length) : buffer(v_buffer), length(v_length) {}
  PrintBuffer(const char *v_buffer,int v_length) : buffer(reinterpret_cast<const unsigned char *>(v_buffer)), length(v_length) {}
};
std::ostream& operator<< (std::ostream &lhs, const PrintBuffer &rhs);
//-------------------------------------------------------------------------------------------

struct PrintVector
{
  const double *start, *end;
  PrintVector(const double *v_start,const double *v_end) : start(v_start), end(v_end) {}
};
std::ostream& operator<< (std::ostream &lhs, const PrintVector &rhs);
//-------------------------------------------------------------------------------------------

class TSerialStreamBase
{
protected:
  const size_t max_length_;
  unsigned char buffer_[2048];
  const unsigned char * const buf_end_;
  unsigned char *buf_ptr_;

  void oferror(void)
    {
      LERROR("in TSerialStreamBase: overflow!");
    }
public:
  TSerialStreamBase (void) :
      max_length_(sizeof(buffer_)/sizeof(buffer_[0])),
      buf_end_ (buffer_+max_length_),
      buf_ptr_ (buffer_) {};

  void Clear (void)
    {
      buffer_[0]= 0;
      buf_ptr_= buffer_;
    }
  size_t  Bytes (void) const
    {
      return  (buf_ptr_-buffer_);
    };
  size_t  Count (void) const
    {
      return  Bytes()/sizeof(buffer_[0]);
    }

  virtual TSerialStreamBase& operator>> (TSerialCom &serial)
    {
      serial.Write (buffer_, Bytes());
      Clear();
      return *this;
    }
};
//-------------------------------------------------------------------------------------------

class TStringWriteStream : public TSerialStreamBase
{
public:
  TStringWriteStream (void) :
      TSerialStreamBase() {};

  TStringWriteStream& operator<< (const unsigned char *s)
    {
      for (; buf_ptr_!=buf_end_ && *s!='\0'; ++s, ++buf_ptr_)
        *buf_ptr_= *s;
      if (*s!='\0')  oferror();
      return *this;
    }
  TStringWriteStream& operator<< (const char *s)
    {
      return operator<<(reinterpret_cast<const unsigned char *>(s));
    }
  TStringWriteStream& operator<< (const int &i);
};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace serial
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // serial_h
//-------------------------------------------------------------------------------------------
