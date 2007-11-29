#------------------------------------------------------------
#                   Choosing platform
#------------------------------------------------------------
UNAME=$(shell uname)
PLATFORM=linux
SUPPORTED_PLATFORM_FOUND=yes

ifeq "$(PLATFORM)" "winnt"
  WINDOWS_TYPE_PLATFORM=yes
  SUPPORTED_PLATFORM_FOUND=yes
endif
ifeq "$(PLATFORM)" "winxp"
  WINDOWS_TYPE_PLATFORM=yes
  SUPPORTED_PLATFORM_FOUND=yes
endif

#------------------------------------------------------------
#                   VARIOUS VARIABLES
#------------------------------------------------------------
MODE = nopt

INCLUDEDIRS=./include /opt/local/include /irg/packages/i386_darwin_gcc4.0/local/include
LINKDIRS=./lib
LINKLIBS=

include local_settings

ifdef USE_OPEN_GL
  ifdef WINDOWS_TYPE_PLATFORM
    GRAPHICS_LIBS=glut32.lib opengl32.lib
  else
    GRAPHICS_LIBS=-lglut -lGLU -lGL
  endif
endif

ifdef WINDOWS_TYPE_PLATFORM
  LIBPATH_PREFIX=-LIBPATH:
endif
ifeq "$(PLATFORM)" "linux"
  LIBPATH_PREFIX=-L
endif


#------------------------------------------------------------
#  Choosing C compiler and linker, and setting specifics
#------------------------------------------------------------
CC	="A_C_compiler_for_use_on_$(PLATFORM)"

ifdef WINDOWS_TYPE_PLATFORM
#ifeq "$(PLATFORM)" "winnt"
  CC		= cl
  LD            = link
  ifeq "$(MODE)" "opt"
    CFLAGS		=-GX -G6 -GA -Gs -Gf -Gy -Ox -Ob2 -nologo -MD
    CDEFS		=-DWIN32 -DMICROSOFT -DNDEBUG
    LDFLAGS		=-nologo 
  else
    CFLAGS		=-GX -nologo -Yd -Z7 -MDd
    CDEFS		=-DWIN32 -DMICROSOFT
    LDFLAGS		=-nologo
  endif
endif

ifeq "$(PLATFORM)" "linux"

  CC		=gcc

  ifeq "$(MODE)" "opt"
    CFLAGS	=-O2 -Wall
    CDEFS	=-DNDEBUG
  else
    CFLAGS	=-g -Wall
    CDEFS	=
  endif

endif

COPTS = $(CFLAGS) $(addprefix -I, $(INCLUDEDIRS)) $(CDEFS)

#------------------------------------------------------------
#  Choosing C++ compiler and linker, and setting specifics
#------------------------------------------------------------
CXX			="A_C++_compiler_for_use_on_$(PLATFORM)"

ifdef WINDOWS_TYPE_PLATFORM
#ifeq "$(PLATFORM)" "winnt"
  CXX			=cl
  LDXX			=link
  ifeq "$(MODE)" "opt"
    CXXFLAGS		=-TP -GX -GR -G6 -GA -Gs -Gf -Gy -Ox -Ob2 -nologo -MD
    CXXDEFS		=-DWIN32 -DMICROSOFT -DNDEBUG
    LDXXFLAGS		=-nologo 
  else
    CXXFLAGS		=-TP -GX -GR -nologo -Yd -Z7 -MDd
    CXXDEFS		=-DWIN32 -DMICROSOFT
    LDXXFLAGS		=-nologo -DEBUGTYPE:BOTH 
  endif
endif

ifeq "$(PLATFORM)" "linux"
    CXX		=g++
    LDXX	=$(CXX)
  ifeq "$(MODE)" "opt"
    CXXFLAGS	=-O2 -Wall
    CXXDEFS	=-DNDEBUG
  else
    CXXFLAGS	=-g -Wall
    CXXDEFS	=
  endif
endif

		
CXXOPTS =$(CXXFLAGS) $(addprefix -I, $(INCLUDEDIRS)) $(CXXDEFS)

#------------------------------------------------------------
#                   Deciding library name
#------------------------------------------------------------
LIBNAME		=

ifdef WINDOWS_TYPE_PLATFORM
#ifeq "$(PLATFORM)" "winnt"
  LIBNAME	=lib$(notdir $(shell pwd))_$(MODE).lib
endif

ifeq "$(PLATFORM)" "linux"
  LIBNAME	=lib$(notdir $(shell pwd))_$(MODE).a
endif

#------------------------------------------------------------
#               Deciding object file names
#------------------------------------------------------------
CSRCS		=$(wildcard src/*.c)
COBJS0		=$(CSRCS:.c=.o)
COBJS		=$(subst src/,lib/,$(COBJS0))

CXXSRCS		=$(wildcard src/*.cpp)
CXXOBJS0	=$(CXXSRCS:.cpp=.o)
CXXOBJS		=$(subst src/,lib/,$(CXXOBJS0))

CXXAPPSRCS	=$(wildcard *.cpp)
CXXAPPS		=$(CXXAPPSRCS:.cpp=)


#------------------------------------------------------------
#                   Rules
#------------------------------------------------------------

ifdef SUPPORTED_PLATFORM_FOUND

#------------------------------------------------------------
#            THE DETECTED PLATFORM IS SUPPORTED
#------------------------------------------------------------

all:		lib

.PHONY:		all clean

clean:;		rm -f $(wildcard lib/*.o lib/*.a lib/*.lib) $(CXXAPPS) \
			$(addsuffix .exe, $(CXXAPPS))

lib:		$(LIBNAME)

$(LIBNAME):	$(COBJS) $(CXXOBJS)
		@echo ""
		@echo Remember to compile the object files with the correct MODE!
		@echo \(opt/nopt\) represents optimized/unoptimized mode
		@echo \(Use 'make clean' first, if in doubt.\)
		@echo ""
  ifdef WINDOWS_TYPE_PLATFORM
		link -lib $(LDXXFLAGS) /out:"lib/$(LIBNAME)" $^
  else
		ar -sr lib/$(LIBNAME) $^
  endif

lib/%.o:	src/%.c
		$(CC) $(COPTS) -c src/$*.c -o $@
  ifdef WINDOWS_TYPE_PLATFORM
		@mv $*.obj $@
  endif


lib/%.o:	src/%.cpp
		$(CXX) $(CXXOPTS) -c src/$*.cpp -o $@
  ifdef WINDOWS_TYPE_PLATFORM
		@mv $*.obj $@
  endif

$(CXXAPPS):	$(LIBNAME)
		$(CXX) $(CXXOPTS) -c $(notdir $@.cpp) -o lib/$@.o
  ifdef WINDOWS_TYPE_PLATFORM
		@mv $@.obj lib/$@.o 
		$(LDXX) $(LDXXFLAGS) -out:$@ \
		$(addprefix $(LIBPATH_PREFIX), $(LINKDIRS)) lib/$@.o \
		$(addsuffix _$(MODE).lib, $(addprefix lib, $(LINKLIBS))) $(GRAPHICS_LIBS)
		@mv $@ $@.exe

  else 
		$(LDXX) $(LDXXFLAGS) $(addprefix $(LIBPATH_PREFIX), $(LINKDIRS)) lib/$@.o\
		$(addsuffix _$(MODE), $(addprefix -l, $(LINKLIBS))) $(GRAPHICS_LIBS) \
		-o $@
  endif

varcheck:;	@echo "testing..."
#		@echo $(wildcard src/*.c)
#		@echo $(wildcard include/*.h)
#		@echo $(wildcard lib/*.o)
		@echo "CSRCS=" $(CSRCS)
		@echo "COBJS=" $(COBJS)
		@echo "CXXOBJS=" $(CXXOBJS)
		@echo "CXXAPPSRCS=" $(CXXAPPSRCS)
		@echo "CXXAPPS=" $(CXXAPPS)
		@echo "LIBNAME=" $(LIBNAME)
		@echo "LIBNAME2=" $(LIBNAME:t.a=rtrt.qwqw)
		@echo notdir shell output= $(notdir $(shell pwd))

else

#------------------------------------------------------------
#                UNKNOWN PLATFORM TYPE
#------------------------------------------------------------
.PHONY:		all lib clean default

all lib clean varcheck: default

default:;	@echo ""
		@echo "Makefile does not support your platform type."
		@echo "You need to edit the Makefile to set the required"
		@echo "variables for your platform type."
		@echo ""
endif
