//-------------------------------------------------------------------------------------------
/*! \file    lora_small.h
    \brief   small library
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \version 0.1
    \date    Jun.07, 2011
*/
//-------------------------------------------------------------------------------------------
#ifndef lora_small_h
#define lora_small_h
//-------------------------------------------------------------------------------------------
#include <sstream>
#include <iostream>
#include <sys/time.h>  // getrusage, gettimeofday
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

// extended keywords:
#define override  // that describes a function is overridden

//-------------------------------------------------------------------------------------------
// convert _code to string; if the _code is a macro, it is not expanded, i.e. the result is "macro"
// FIXME we strongly recommend BOOST_PP_STRINGIZE
#define CODE_TO_STR(_code) #_code
/* convert _cdoe to string, if the _code is a macro, it is expanded
    WARNING: this will depend on gcc/g++ because
        `The order of evaluation of # and ## operators is unspecified.'
        (ISO/IEC 14882:2003 (E): 16.3.2 The # operator)  */
#define CODE_TO_STR_M(_code) CODE_TO_STR(_code)
//-------------------------------------------------------------------------------------------
#define LORA_CURRENT_FUNCTION  __PRETTY_FUNCTION__
//-------------------------------------------------------------------------------------------
namespace message_system
{
  enum TMessageType {mtMessage=0, mtError, mtWarning, mtDebug, mtTodo, mtFixme};
  namespace detail
  {
    void OutputMessage(TMessageType type, int linenum, const char *filename, const char *functionname, std::stringstream &ss);
  }
  void DefaultFormat(TMessageType type, int linenum, const char *filename, const char *functionname, std::stringstream &ss);
  typedef void(*LORA_MESSAGE_FORMAT_FUNCTION)(::loco_rabbits::message_system::TMessageType,int,const char*,const char*,::std::stringstream&);
  /*!\brief Set message format
      \note SetFormat is instantiated for
              t_format_function = LORA_MESSAGE_FORMAT_FUNCTION , and
              t_format_function = void (*)(TMessageType,int,const char*,const char*,std::stringstream&) */
  template<typename t_format_function> void SetFormat(t_format_function fmt);
  /*!\brief Get message format
      \note SetFormat is instantiated only for LORA_MESSAGE_FORMAT_FUNCTION.  The reason of being a template
              is not to include boost/function.hpp in this header file */
  template<typename t_format_function> t_format_function GetFormat(void);
}
#define L_OUTPUT_MESSAGE(x_type,x_msg) ::loco_rabbits::message_system::detail::OutputMessage( \
              ::loco_rabbits::message_system::x_type, __LINE__,  __FILE__, LORA_CURRENT_FUNCTION, x_msg)
#define LMESSAGE(msg)  do{std::stringstream ss; ss<<msg; L_OUTPUT_MESSAGE(mtMessage,ss);}while(0)
#define LERROR(msg)    do{std::stringstream ss; ss<<msg; L_OUTPUT_MESSAGE(mtError  ,ss);}while(0)
#define LWARNING(msg)  do{std::stringstream ss; ss<<msg; L_OUTPUT_MESSAGE(mtWarning,ss);}while(0)
#define LDEBUG(msg)    do{std::stringstream ss; ss<<msg; L_OUTPUT_MESSAGE(mtDebug  ,ss);}while(0)
#define LDBGVAR(var)   do{std::stringstream ss; ss<<#var"= "<<(var); L_OUTPUT_MESSAGE(mtDebug,ss);}while(0)
#define LTODO(msg)     do{std::stringstream ss; ss<<msg; L_OUTPUT_MESSAGE(mtTodo   ,ss);}while(0)
#define FIXME(msg)     do{std::stringstream ss; ss<<msg; L_OUTPUT_MESSAGE(mtFixme  ,ss);lexit(df);}while(0)
//-------------------------------------------------------------------------------------------
#define LASSERT(eqn)                                        \
    do{if(!(eqn)){                                          \
      LERROR("assertion failed: (" #eqn ")");               \
      lexit(abort);                                         \
    }}while(0)
#define LASSERT1op1(lhs,op,rhs)                             \
    do{if(!((lhs) op (rhs))){                               \
      LERROR("assertion failed: (" #lhs")" #op "("#rhs")"); \
      std::cerr<<"  ("#lhs")= "<<(lhs)<<std::endl;          \
      std::cerr<<"  ("#rhs")= "<<(rhs)<<std::endl;          \
      lexit(abort);                                         \
    }}while(0)
//-------------------------------------------------------------------------------------------
#define SIZE_OF_ARRAY(array)  (sizeof(array)/sizeof((array)[0]))
//-------------------------------------------------------------------------------------------

namespace exitlv
{
struct TException
{
  int LineNum;
  const char *FileName;
  const char *FunctionName;
  TException (int v_linenum,const char *v_filename,const char *v_functionname)
    : LineNum(v_linenum), FileName(v_filename), FunctionName(v_functionname)  {}
};

enum TExitLevel {
  success=0  /*! exit quietly, and return success code */,
  qfail      /*! exit with printing where the program is terminated,
                  and return failure code*/,
  btfail     /*! exit with printing where the program is terminated
                  and printing the backtrace of stack,
                  and return failure code*/,
  abort      /*! exit with printing where the program is terminated,
                  and abort. note that a core file will be generated */,
  th         /*! throw an exception of the type TException */,
  df=1000    /*! exit with default level which is modifiable by ChangeDefault;
                  many functions in loco_rabbits use this exit level */};

namespace detail
{
  void i_lexit (
      TExitLevel exit_level,
      int linenum,
      const char *filename,
      const char *functionname);
}
//! \brief Change default exit scheme
void ChangeDefault (TExitLevel exit_level);
//-------------------------------------------------------------------------------------------
} // end of namespace exitlv
//-------------------------------------------------------------------------------------------
#define lexit(x_level)  ::loco_rabbits::exitlv::detail::i_lexit( \
                                ::loco_rabbits::exitlv::x_level, \
                                __LINE__,  __FILE__, LORA_CURRENT_FUNCTION)
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// colored out
//-------------------------------------------------------------------------------------------
namespace ioscc
{
  enum TColorCode {none=0, red=1, blue, green};

  namespace detail
  {
    extern const char *code_esc   ;
    extern const char *code_reset ;
    extern const char *code_red   ;
    extern const char *code_green ;
    extern const char *code_blue  ;
  }

  //!\brief Disable colored out
  void Disable(void);
  //!\brief Change color scheme
  void SetScheme(const char *v_red, const char *v_green, const char *v_blue);
  //!\brief Reset color scheme to default
  void ResetScheme();
}

struct ColoredStream
{
  std::ostream &Os;
};

inline ColoredStream operator<< (std::ostream &os, ioscc::TColorCode ccode)
{
  switch (ccode)
  {
    case ioscc::none   : os<<ioscc::detail::code_esc<<ioscc::detail::code_reset; break;
    case ioscc::red    : os<<ioscc::detail::code_esc<<ioscc::detail::code_red;   break;
    case ioscc::green  : os<<ioscc::detail::code_esc<<ioscc::detail::code_green; break;
    case ioscc::blue   : os<<ioscc::detail::code_esc<<ioscc::detail::code_blue;  break;
    default  :  std::cerr<<"invalid color code: "<<static_cast<int>(ccode)<<std::endl;  lexit(df);
  }
  ColoredStream colos={os};
  return colos;
}

template <typename T>
inline ColoredStream operator<< (ColoredStream os, const T     &rhs)
{
  os.Os << rhs;
  return os;
}
template <>
inline ColoredStream operator<< (ColoredStream os, const ioscc::TColorCode &ccode)
{
  os.Os << ioscc::detail::code_esc<<ioscc::detail::code_reset;
  os.Os << ccode;
  return os;
}
inline ColoredStream operator<< (ColoredStream os, std::ostream& (*pf)(std::ostream&))
{
  os.Os << ioscc::detail::code_esc<<ioscc::detail::code_reset;
  pf(os.Os);
  return os;
}
//-------------------------------------------------------------------------------------------


inline double GetCurrentTime (void)
{
  struct timeval time;
  gettimeofday (&time, NULL);
  return static_cast<double>(time.tv_sec) + static_cast<double>(time.tv_usec)*1.0e-6;
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // lora_small_h
//-------------------------------------------------------------------------------------------
