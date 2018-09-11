
# This file generates config.h from config.h.in

include(CheckIncludeFiles)
include(CheckSymbolExists)
include(CheckFunctionExists)
include(CheckTypeSize)
include(CheckCXXSymbolExists)
include(CheckCXXSourceCompiles)


# TODO: How much of this do we need?

###########################################################################

# Check if certain include files are present
# - Define to 1 if present, blank otherwise.
#CHECK_INCLUDE_FILES(ext/stdio_filebuf.h VW_HAVE_EXT_STDIO_FILEBUF_H) # TODO
#CHECK_INCLUDE_FILES(fenv.h              VW_HAVE_FENV_H)
CHECK_INCLUDE_FILES(inttypes.h          ASP_HAVE_INTTYPES_H)
#CHECK_INCLUDE_FILES(memory.h            VW_HAVE_MEMORY_H)
CHECK_INCLUDE_FILES(pwd.h               ASP_HAVE_PWD_H)
#CHECK_INCLUDE_FILES(stdint.h            VW_HAVE_STDINT_H)
#CHECK_INCLUDE_FILES(stdlib.h            VW_HAVE_STDLIB_H)
#CHECK_INCLUDE_FILES(strings.h           VW_HAVE_STRINGS_H)
#CHECK_INCLUDE_FILES(string.h            VW_HAVE_STRING_H)
#CHECK_INCLUDE_FILES(sys/stat.h          VW_HAVE_SYS_STAT_H)
#CHECK_INCLUDE_FILES(sys/types.h         VW_HAVE_SYS_TYPES_H)
CHECK_INCLUDE_FILES(dlfcn.h             ASP_HAVE_DLFCN_H)
#CHECK_INCLUDE_FILES(unistd.h            VW_HAVE_UNISTD_H)

# Ignore, only used by plate.
# Define to 1 if you have the ANSI C header files. 
#define VW_STDC_HEADERS @STDC_HEADERS@


###########################################################################
# Check if certain compiler features are available

set(emptyIncludeList )

CHECK_CXX_SOURCE_COMPILES("void testFunc() __attribute__((deprecated));         void testFunc(){}   int main(){return 0;}" ASP_COMPILER_HAS_ATTRIBUTE_DEPRECATED)
CHECK_CXX_SOURCE_COMPILES("void testFunc() __attribute__((noreturn));           void testFunc(){}   int main(){return 0;}" ASP_COMPILER_HAS_ATTRIBUTE_NORETURN)
#CHECK_CXX_SOURCE_COMPILES("void testFunc() __attribute__((warn_unused_result)); void testFunc(){}   int main(){return 0;}" VW_COMPILER_HAS_ATTRIBUTE_WARN_UNUSED_RESULT)


# Check for some supported functions (could probably streamline ssize_t check)
#check_cxx_symbol_exists(exp2            cmath                  VW_HAVE_EXP2)
#check_cxx_symbol_exists(fabsl           cmath                  VW_HAVE_FABSL)
#check_cxx_symbol_exists(feenableexcept  "fenv.h"               VW_HAVE_FEENABLEEXCEPT)
check_cxx_symbol_exists(getpid          "unistd.h;sys/types.h" ASP_HAVE_GETPID)
check_cxx_symbol_exists(getpwuid        "pwd.h;sys/types.h"    ASP_HAVE_GETPWUID)
#check_cxx_symbol_exists(llabs           "stdlib.h"             VW_HAVE_LLABS)
#check_cxx_symbol_exists(log2            cmath                  VW_HAVE_LOG2)
#check_cxx_symbol_exists(mkstemps        "stdlib.h"             VW_HAVE_MKSTEMPS)
#check_cxx_symbol_exists(tgamma          cmath                  VW_HAVE_TGAMMA)
#CHECK_CXX_SOURCE_COMPILES("
#                          #include <sys/types.h>
#                          int main(){ssize_t a=2; return a;}" VW_HAVE_SSIZET)




###########################################################################
# Determine which libraries we can build

# If we made it to here we can build these modules
set(ASP_HAVE_PKG_CORE 1)
set(ASP_HAVE_PKG_CAMERA 1)
set(ASP_HAVE_PKG_SPICEIO 1)
set(ASP_HAVE_PKG_ISISIO 1)
set(ASP_HAVE_PKG_SESSIONS 1)
set(ASP_HAVE_PKG_GUI 1)
set(ASP_HAVE_PKG_TOOLS 1)





#######################################################################
# Finished setting up variables, now call the function to paste them into a file

# Each value like "@VAR@ is replaced by the CMake variable of the same name
message("Generating config file: ${CMAKE_SOURCE_DIR}/src/asp/asp_config.h")
configure_file(${CMAKE_SOURCE_DIR}/src/asp/asp_config.h.in ${CMAKE_SOURCE_DIR}/src/asp/asp_config.h)














