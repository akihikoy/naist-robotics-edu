#!/bin/bash
usage="usage: compile.sh [options] FILE [-- ADDITIONAL OPTIONS FOR g++]
  options:
    -eig         : using Eigen
    -sg          : using Shogun
    -cv          : using OpenCV
    -ode         : using ODE
    -nao         : using NAO
    -bio         : using Bioloid
    -help        : show this"
base_dir=`dirname $0`/..
config_file=`dirname $0`/config.sh
if [ ! -f $config_file ];then
  echo "$config_file not found."
  echo "create it by reference to $config_file.sample"
  exit 1
fi
source $config_file

TARGET=
LIBS=-I$base_dir
LDLIBS=
while true; do
  case "$1" in
    -help|--help) echo "usage: $usage"; exit 0 ;;
    -eig) LIBS="$LIBS $LIBS_EIG"; shift 1 ;;
    -sg)  LIBS="$LIBS $LIBS_SHOGUN"; LDLIBS="$LDLIBS $LDLIBS_SHOGUN"; shift 1 ;;
    -cv)  LIBS="$LIBS $LIBS_CV"; LDLIBS="$LDLIBS $LDLIBS_CV"; shift 1 ;;
    -ode) LIBS="$LIBS $LIBS_ODE"; LDLIBS="$LDLIBS $LDLIBS_ODE"; shift 1 ;;
    -nao) LIBS="$LIBS $LIBS_NAO"; LDLIBS="$LDLIBS $LDLIBS_NAO"; shift 1 ;;
    -bio) LIBS="$LIBS $LIBS_BIO"; LDLIBS="$LDLIBS $LDLIBS_BIO";
      if ! [ -f $BIO_PATH/libbioloid.a ];then
        echo "$BIO_PATH/libbioloid.a: library not found"
        echo "type: cd $BIO_PATH; make"
        exit 1
      fi
      shift 1 ;;
    '') break ;;
    --) shift 1; break ;;
    *)
      if [ -n "$TARGET" ];then echo "usage: $usage"; exit 0; fi
      if [ "$1" != "${1/.cpp/.out}" ];then TARGET="$1 -o ${1/.cpp/.out}"
      else TARGET="$1"; fi
      shift 1 ;;
  esac
done

echo g++ $CXXFLAGS $TARGET $@ $LIBS $LDLIBS
g++ $CXXFLAGS $TARGET $@ $LIBS $LDLIBS
