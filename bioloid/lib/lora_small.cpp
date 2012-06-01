//-------------------------------------------------------------------------------------------
/*! \file    lora_small.cpp
    \brief   small library
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \version 0.1
    \date    Jun.07, 2011
*/
//-------------------------------------------------------------------------------------------
#include "lora_small.h"
#include <cstdlib>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;

namespace message_system
{
  LORA_MESSAGE_FORMAT_FUNCTION  fmt_function= &DefaultFormat;
  namespace detail
  {
    void OutputMessage(TMessageType type, int linenum, const char *filename, const char *functionname, std::stringstream &ss)
    {
      fmt_function(type, linenum, filename, functionname, ss);
    }
  }
  void DefaultFormat(TMessageType type, int linenum, const char *filename, const char *functionname, std::stringstream &ss)
  {
    switch (type)
    {
      case  mtMessage  : std::cerr<<ioscc::blue<<ss.str()<<std::endl; return;
      case  mtError    : std::cerr<<ioscc::red<<"error("<<filename<<":"<<linenum<<"): "<<ss.str()<<std::endl; return;
      case  mtWarning  : std::cerr<<ioscc::red<<"warning("<<filename<<":"<<linenum<<"): "<<ss.str()<<std::endl; return;
      case  mtDebug    : std::cerr<<ioscc::red<<"debug("<<filename<<":"<<linenum<<"): "<<ss.str()<<std::endl; return;
      case  mtTodo     : std::cerr<<ioscc::green<<"todo("<<filename<<":"<<linenum<<"): "<<ss.str()<<std::endl; return;
      case  mtFixme    : std::cerr<<ioscc::red<<"FIX "<<filename<<":"<<linenum<<": "<<ss.str()<<std::endl; return;
      default : std::cerr<<ioscc::red<<"loco_rabbits: systam fatal!"<<std::endl;
    }
  }
  // template SetFormat and GetFormat:
  template<typename t_format_function>
  void SetFormat(t_format_function fmt)
  {
    fmt_function= fmt;
  }
  template<typename t_format_function> t_format_function GetFormat(void)
  {
    return fmt_function;
  }
  // instantiations
  template void SetFormat(LORA_MESSAGE_FORMAT_FUNCTION);
  // template void SetFormat(void (*)(TMessageType,int,const char*,const char*,std::stringstream&));
  template LORA_MESSAGE_FORMAT_FUNCTION GetFormat(void);
}
//-------------------------------------------------------------------------------------------

static void StackTrace (void)
{
// #ifdef __GLIBC__
  // void *trace[256];
  // int n = backtrace(trace, sizeof(trace) / sizeof(trace[0]));
  // backtrace_symbols_fd(trace, n, STDERR_FILENO);
// #else
  LWARNING("StackTrace is not implemented to this system.");
// #endif
}
//-------------------------------------------------------------------------------------------

namespace exitlv
{
  // static const TExitLevel SYSTEM_DEFAULT_EXIT_LEVEL(btfail);
  static const TExitLevel SYSTEM_DEFAULT_EXIT_LEVEL(abort);
  static TExitLevel DEFAULT_EXIT_LEVEL(SYSTEM_DEFAULT_EXIT_LEVEL);

  namespace detail
  {
    void i_lexit (
        TExitLevel exit_level,
        int linenum,
        const char *filename,
        const char *functionname)
    {
      if (exit_level==df)
        exit_level= DEFAULT_EXIT_LEVEL;
      if (exit_level!=success && exit_level!=th)
        std::cerr<<ioscc::green<<"------"<<std::endl
          <<ioscc::green<<"the program is terminated"<<std::endl
          <<ioscc::green<<"  at "<<ioscc::blue<<filename<<ioscc::green<<":"
                                <<ioscc::blue<<linenum<<ioscc::green<<":"<<std::endl
          <<ioscc::green<<"  within the function: "<<ioscc::blue<<functionname<<std::endl;
      switch (exit_level)
      {
        case success :
          std::exit(EXIT_SUCCESS);
        case qfail   :
          std::exit(EXIT_FAILURE);
        case btfail  :
          std::cerr<<"backtrace:"<<std::endl;
          StackTrace();
          std::exit(EXIT_FAILURE);
        case abort   :
          std::abort();
        case th      :
          throw TException(linenum,filename,functionname);
        default      :
          std::cerr<<"improper usage of lexit(exit_level); invalid exit_level: "
            <<static_cast<int>(exit_level)<<std::endl;
          std::exit(EXIT_FAILURE);
      }
    }
  }  // end of detail

  void ChangeDefault (TExitLevel exit_level)
  {
    if(exit_level==df)
      DEFAULT_EXIT_LEVEL= SYSTEM_DEFAULT_EXIT_LEVEL;
    else
      DEFAULT_EXIT_LEVEL= exit_level;
  }
//-------------------------------------------------------------------------------------------
} // end of namespace exitlv
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
// colored out
//-------------------------------------------------------------------------------------------
namespace ioscc
{
  static const char *const default_code_esc    = "\033[";
  static const char *const default_code_reset  = "0m"   ;
  static const char *const default_code_red    = "31;1m";
  static const char *const default_code_green  = "32;1m";
  static const char *const default_code_blue   = "34;1m";

  namespace detail
  {
    const char *code_esc   = default_code_esc  ;
    const char *code_reset = default_code_reset;
    const char *code_red   = default_code_red  ;
    const char *code_green = default_code_green;
    const char *code_blue  = default_code_blue ;
  }
  //!\brief Disable colored out
  void Disable(void)
  {
    using namespace detail;
    code_esc   = "";
    code_reset = "";
    code_red   = "";
    code_blue  = "";
    code_green = "";
  }
  //!\brief Change color scheme
  void SetScheme(const char *v_red, const char *v_green, const char *v_blue)
  {
    using namespace detail;
    ResetScheme();
    code_red   = v_red  ;
    code_green = v_green;
    code_blue  = v_blue ;
  }
  //!\brief Reset color scheme to default
  void ResetScheme()
  {
    using namespace detail;
    code_esc   = default_code_esc  ;
    code_reset = default_code_reset;
    code_red   = default_code_red  ;
    code_green = default_code_green;
    code_blue  = default_code_blue ;
  }
}  // end of ioscc
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

