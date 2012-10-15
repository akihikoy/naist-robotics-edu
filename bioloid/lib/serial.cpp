//-------------------------------------------------------------------------------------------
/*! \file    serial.cpp
    \brief   serial communication library (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \version 0.1
    \date    Mar.08, 2010
*/
//-------------------------------------------------------------------------------------------
#include <serial.h>
//-------------------------------------------------------------------------------------------
#include<cerrno>
#include<cstdio>
#include<unistd.h>
#include<fcntl.h>

#include <iostream>
#include <sstream>
#include <sys/ioctl.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace serial
{
using namespace std;
// using namespace boost;


//===========================================================================================
// class TComBase
//===========================================================================================

/*virtual*/int TComBase::PersistingRead (unsigned char *buff, size_t N, int max_trial)
{
  size_t rN(0),dn;
  do
  {
    dn= Read(buff+rN,N-rN);
    if(dn==0)  break;
    rN+=dn;
    --max_trial;
  } while (rN<N && max_trial>0);
  return rN;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TSerialCom : public TComBase
//===========================================================================================

/*override*/bool TSerialCom::Open(void)
{
  errno=0;
  if((fd = open(s_tty.c_str(), O_RDWR)) < 0)
  {
    perror("open");
    return false;
  }
  else
  {
    //fprintf(stderr,"Can open %s \n", tty);
    fcntl(fd, F_SETFL, 0);
  }

  errno=0;
  if ( !isatty(fd) )
  {
    perror("isatty");
    close(fd);
    return false;
  }

  errno=0;
  if( tcsetattr(fd, TCSANOW, &s_ios) < 0 )
  {
    perror("tcsetattr");
    return false;
  }

  // control the device
  int status;
  ioctl (fd, TIOCMGET, &status);
  status &= ~TIOCM_CTS;  // clear to send
  status &= ~TIOCM_DTR;  // data terminal ready
  status &= ~TIOCM_RTS;  // request to send
  status &= ~TIOCM_DSR;  // data set ready
  ioctl (fd, TIOCMSET, &status);

  return true;
}
//-------------------------------------------------------------------------------------------

/*override*/bool TSerialCom::Close()
{
  if(fd != -1)
  {
    close(fd);
    // cout << "Port Close" << endl;
  }
  fd = -1;
  return true;
}
//-------------------------------------------------------------------------------------------

/*override*/int TSerialCom::Write(const void *buff,size_t size)
{
  errno=0;
  int res=write(fd, buff, size);
  if (res==-1)
    perror("write");
  // cerr<<"dbg: TSerialCom::Write res= "<<res<<endl;
  return res;
}
//-------------------------------------------------------------------------------------------

/*override*/int TSerialCom::Read(void *buff,size_t size)
{
  errno=0;
  int res;
  // while ((res=read(fd, buff, size))==0);
  res=read(fd, buff, size);
  if (res==-1)
    perror("read");
  // cerr<<"dbg: TSerialCom::Read res= "<<res<<endl;
  return res;
}
//-------------------------------------------------------------------------------------------

/*override*/void TSerialCom::Clear()
{
  tcflush(fd, TCIFLUSH);
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
#ifdef HAVE_LIBFTD2XX
// class TUSBSerial : public TComBase
//===========================================================================================

//-------------------------------------------------------------------------------------------
// following routines are based on /root/install/libftd/sample/SetVIDPID/main.c
//-------------------------------------------------------------------------------------------
/*
  A wrapper function to allow safe listing of devices
*/
FT_STATUS TUSBSerial::iFT_ListDevices(PVOID pArg1, PVOID pArg2, DWORD Flags)
{
  FT_STATUS  ftStatus;
  // DWORD iVID, iPID;
  DWORD iOldVID, iOldPID;

  // CheckConfigFile(&iVID, &iPID);    // get our VID and PID from config file or other source
  FT_GetVIDPID(&iOldVID, &iOldPID);  // get original VID and PID
  FT_SetVIDPID(conf.iVID, conf.iPID);              // use our VID and PID
  ftStatus = FT_ListDevices(pArg1, pArg2, Flags);    // Call FTDI function
  FT_SetVIDPID(iOldVID, iOldPID);            // restore original VID and PID

  return ftStatus;
}
//-------------------------------------------------------------------------------------------

/*
  A wrapper function to allow safe open of devices
*/
FT_STATUS TUSBSerial::iFT_Open(int deviceNumber, FT_HANDLE *pHandle)
{
  FT_STATUS  ftStatus;
  // DWORD iVID, iPID;
  DWORD iOldVID, iOldPID;

  // CheckConfigFile(&iVID, &iPID);    // get our VID and PID from config file or other source
  FT_GetVIDPID(&iOldVID, &iOldPID);  // get original VID and PID
  FT_SetVIDPID(conf.iVID, conf.iPID);              // use our VID and PID
  ftStatus = FT_Open(deviceNumber, pHandle);
  FT_SetVIDPID(iOldVID, iOldPID);            // restore original VID and PID

  return ftStatus;
}
//-------------------------------------------------------------------------------------------

/*
  A wrapper function to allow safe openex of devices
*/
FT_STATUS TUSBSerial::iFT_OpenEx(PVOID pArg1, DWORD Flags, FT_HANDLE *pHandle)
{
  FT_STATUS  ftStatus;
  // DWORD iVID, iPID;
  DWORD iOldVID, iOldPID;

  // CheckConfigFile(&iVID, &iPID);    // get our VID and PID from config file or other source
  FT_GetVIDPID(&iOldVID, &iOldPID);  // get original VID and PID
  FT_SetVIDPID(conf.iVID, conf.iPID);              // use our VID and PID
  ftStatus = FT_OpenEx(pArg1, Flags, pHandle);
  FT_SetVIDPID(iOldVID, iOldPID);            // restore original VID and PID

  return ftStatus;
}
//-------------------------------------------------------------------------------------------

/*override*/bool TUSBSerial::Open(void)
{
  FT_STATUS  ftStatus;
  ftStatus = iFT_Open(device_num, &handle);
  if(ftStatus != FT_OK)
  {
    /*
      This can fail if the ftdi_sio driver is loaded
       use lsmod to check this and rmmod ftdi_sio to remove
      also rmmod usbserial
     */
    cerr<<"TUSBSerial::Open(): fatal! FT_Open("<<device_num<<") failed"<<endl;
    is_opened= false;
    return false;
  }

  // setting...
  if( (ftStatus=FT_SetBaudRate(handle, conf.BaudRate))!=FT_OK
    ||(ftStatus=FT_SetTimeouts(handle, conf.ReadTimeout, conf.WriteTimeout))!=FT_OK
    ||(ftStatus=FT_SetDataCharacteristics(handle, conf.WordLength, conf.StopBits, conf.Parity))!=FT_OK)
  {
    cerr<<"TUSBSerial::Open(): fatal! ftailed to set configulations"<<endl;
    is_opened= false;
    return false;
  }
  is_opened= true;
  return true;
}
//-------------------------------------------------------------------------------------------

/*override*/bool TUSBSerial::Close(void)
{
  is_opened= false;
  FT_STATUS ftStatus = FT_Close(handle);
  if(ftStatus != FT_OK)
  {
    cerr<<"TUSBSerial::Close(): fatal! failed to close"<<endl;
    return false;
  }
  return true;
}
//-------------------------------------------------------------------------------------------

/*override*/int TUSBSerial::Write(const void *buff, size_t size)
{
  DWORD dwBytesWritten(0);
  FT_STATUS ftStatus = FT_Write(handle, const_cast<void*>(buff), size, &dwBytesWritten);
  if(ftStatus != FT_OK)
  {
    cerr<<"TUSBSerial::Write(): fatal! failed to write"<<endl;
    return -1;
  }
  return dwBytesWritten;
}
//-------------------------------------------------------------------------------------------

/*override*/int TUSBSerial::Read(void *buff, size_t size)
{
  DWORD dwBytesRead(0);
  FT_STATUS ftStatus = FT_Read(handle, buff, size, &dwBytesRead);
  if(ftStatus != FT_OK)
  {
    cerr<<"TUSBSerial::Read(): fatal! failed to read"<<endl;
    return -1;
  }
  return dwBytesRead;
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
#endif // HAVE_LIBFTD2XX
//-------------------------------------------------------------------------------------------


//===========================================================================================
// utility
//===========================================================================================

std::ostream& operator<< (std::ostream &lhs, const PrintBuffer &rhs)
{
  const unsigned char *b (rhs.buffer);
  for(int i(rhs.length); i>0; --i,++b)
    lhs<<" "<<std::hex<<std::setfill('0')<<std::setw(2)<<(int)(*b)<<std::dec;
  return lhs;
}
//-------------------------------------------------------------------------------------------

std::ostream& operator<< (std::ostream &lhs, const PrintVector &rhs)
{
  for(const double *ptr=rhs.start; ptr!=rhs.end; ++ptr)
    lhs<<" "<<*ptr;
  return lhs;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TStringWriteStream : public TSerialStreamBase
//===========================================================================================

TStringWriteStream& TStringWriteStream::operator<< (const int &i)
{
  stringstream ss;
  ss<<i;
  return operator<<(ss.str().c_str());
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace serial
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

