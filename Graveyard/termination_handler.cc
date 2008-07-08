//
// Copyright (C) 2006 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
// 
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
// 
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
//

//
// termination_handler.cc
// 
// Stereo Pipeline for processing linescan stereo imagery from MGS and MRO.
// 
// Created 30 May 2006 by rsargent.
//
#if defined(__GNUC__) && ((__GNUC__>3)||((__GNUC__==3)&&(__GNUC_MINOR__>1)))

#include <stdio.h>
#include <stdexcept>
#include <cxxabi.h>
#include <execinfo.h>
#include <dlfcn.h>
#include <unistd.h>
using namespace abi;
using namespace std;

// Adapted by Randy Sargent 5/29/2006
// Much comes from glibc verbose termination handler

static void termination_handler()
{
  // Make sure there was an exception; terminate is also called for an
  // attempt to rethrow when there is no suitable exception.
  type_info *t = __cxa_current_exception_type();
  if (t) {
    char const *name = t->name();
    // Note that "name" is the mangled name.
    
    int status = -1;
    char *dem = __cxa_demangle(name, 0, 0, &status);
    printf("Terminate program after a `%s' was thrown.\n", 
           status == 0 ? dem : name);
    if (status == 0) free(dem);
  
    // If the exception is derived from std::exception, we can give more
    // information.
    try { __throw_exception_again; }
    catch (std::exception &exc) {
      fprintf(stderr, "\twhat(): %s\n", exc.what());
    }
    catch (...) { }
  }
  else
    fprintf(stderr, "Terminate called without an active exception.\n");

  void *backtrace_buffer[200];
  int backtrace_size= backtrace(backtrace_buffer, 200);
  if (backtrace_size) {
    fprintf(stderr, "Stack backtrace:\n");
    for (int i= 0; i< backtrace_size; i++) {
      Dl_info info;
      fprintf(stderr, "  ");
      if (dladdr(backtrace_buffer[i], &info)) {
        if (info.dli_fname) fprintf(stderr, "%s", info.dli_fname);
        if (info.dli_sname) {
          int status= -1;
          char *dem = __cxa_demangle(info.dli_sname, 0, 0, &status);
          fprintf(stderr, "(%s+0x%lx)", status == 0 ? dem : info.dli_sname,
                  ((long)backtrace_buffer[i])-((long)info.dli_saddr));
          if (status == 0) free(dem);
        }
      }
      fprintf(stderr, " [0x%lx]\n", (long)backtrace_buffer[i]);
    }
  }
  fprintf(stderr, "If you'd like to connect gdb to this process, use:\n");
  // This works on linux:
  // fprintf(stderr, "gdb `readlink /proc/%d/exe` %d\n", getpid(), getpid());
  // Looks like gdb doesn't even care what the first argument is since it looks up the symbols after connecting to the PID
  fprintf(stderr, "gdb ignored %d\n", getpid());
  fprintf(stderr, "Press 'enter' to abort...\n");
  getchar();
  abort();
}

static int install_termination_handler()
{
  std::set_terminate(termination_handler);
  return 0;
}

static int init_termination_handler= install_termination_handler();

#endif
