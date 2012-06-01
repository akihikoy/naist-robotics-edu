##-------------------------------------------------------------------------------------------
# \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
# \version 0.1
# \date    Jun.07, 2011
##-------------------------------------------------------------------------------------------
ARC = libbioloid.a
LIBOBJS :=
CXX := g++
AR := ar
ifeq ($(DEBUG),true)
  CXXFLAGS := -g -Wall -rdynamic
else
  CXXFLAGS := -g -Wall -rdynamic -O3
endif
LDFLAGS  := $(CXXFLAGS)
RMFLAGS  :=  -vf
##-------------------------------------------------------------------------------------------
BASEPREFIX := $(wildcard ~/prg)
##-------------------------------------------------------------------------------------------
## make  *.o  from  *.c and *.cpp
OBJS    := \
      $(patsubst %.c,%.o,$(wildcard *.c)) \
      $(patsubst %.cpp,%.o,$(wildcard *.cpp)) #
##-------------------------------------------------------------------------------------------
LIBS    := -I/usr/include -I/usr/local/include
LDLIBS  := -lm -L/usr/local/lib
##-------------------------------------------------------------------------------------------
LIBS += -I.
##-------------------------------------------------------------------------------------------
all : $(ARC)
##-------------------------------------------------------------------------------------------
## generic make rules

clean   :
	@rm $(RMFLAGS) *.o *.a *.il $(OBJS) $(EXEC) $(ARC) $(EXTRA_CLEAN)
	@find -name '*.d' -type f | xargs rm $(RMFLAGS)

$(EXEC) : $(OBJS) $(LIBOBJS)
	$(CXX) -o $@ $(OBJS) $(LDFLAGS) $(LIBOBJS) $(LDLIBS)
	@(becho "$(EXEC) is generated")

$(ARC) : $(OBJS) $(LIBOBJS)
	$(AR) r $(ARC) $(OBJS) $(LIBOBJS)
	@(becho "$(ARC) is generated")

%.o : %.cpp
	$(CXX) -o $@ $(LIBS) $(CXXFLAGS) -c $<

DEPOPT  ?= -MM

%.d: %.cpp
	@(rm -f $@; \
	$(CXX) $(DEPOPT) $(LIBS) $(CXXFLAGS) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	[ -s $@ ] || rm -f $@; \
	rm -f $@.$$$$; \
	)
##-------------------------------------------------------------------------------------------
include $(OBJ:.o=.d) $(OBJS:.o=.d) $(objects:.o=.d) $(addsuffix .d, $(BINARY))
##-------------------------------------------------------------------------------------------
