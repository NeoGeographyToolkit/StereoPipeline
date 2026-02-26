# Cross-compile toolchain for building on Mac ARM64 targeting Linux x86_64.
#
# Both VisionWorkbench and StereoPipeline have a copy of this file. Keep them
# in sync when making changes.
#
# Prerequisites:
#   - conda-forge clang 16+ with -fopenmp support (in MAC_ASP_DEPS env)
#   - lld (LLVM linker) in MAC_ASP_DEPS env
#   - Linux deps prefix with sysroot and GCC 12.4.0 libraries
#
# Usage (VW):
#   cmake ../src \
#     -DCMAKE_TOOLCHAIN_FILE=../cmake/linux_cross_toolchain.cmake \
#     -DLINUX_DEPS_PREFIX=$HOME/miniconda3/envs/asp_deps_linux \
#     -DMAC_ASP_DEPS=$HOME/anaconda3/envs/asp_deps \
#     -DASP_DEPS_DIR=$HOME/miniconda3/envs/asp_deps_linux \
#     -DCMAKE_INSTALL_PREFIX=$HOME/projects/StereoPipeline/install_linux
#
# Usage (ASP):
#   cmake ../src \
#     -DCMAKE_TOOLCHAIN_FILE=../cmake/linux_cross_toolchain.cmake \
#     -DLINUX_DEPS_PREFIX=$HOME/miniconda3/envs/asp_deps_linux \
#     -DMAC_ASP_DEPS=$HOME/anaconda3/envs/asp_deps \
#     -DASP_DEPS_DIR=$HOME/miniconda3/envs/asp_deps_linux \
#     -DVISIONWORKBENCH_INSTALL_DIR=$HOME/projects/StereoPipeline/install_linux \
#     -DCMAKE_INSTALL_PREFIX=$HOME/projects/StereoPipeline/install_linux \
#     -DOpenMP_C_FLAGS=-fopenmp \
#     -DOpenMP_CXX_FLAGS=-fopenmp \
#     -DOpenMP_C_LIB_NAMES=omp \
#     -DOpenMP_CXX_LIB_NAMES=omp \
#     -DOpenMP_omp_LIBRARY=${LINUX_DEPS_PREFIX}/lib/libomp.so

# Validate required variables.
if(NOT DEFINED LINUX_DEPS_PREFIX)
  message(FATAL_ERROR "Set -DLINUX_DEPS_PREFIX=/path/to/asp_deps_linux")
endif()
if(NOT DEFINED MAC_ASP_DEPS)
  message(FATAL_ERROR "Set -DMAC_ASP_DEPS=/path/to/mac/asp_deps")
endif()

# Derived paths.
set(CROSS_SYSROOT "${LINUX_DEPS_PREFIX}/x86_64-conda-linux-gnu/sysroot")
set(GCC_LIB "${LINUX_DEPS_PREFIX}/lib/gcc/x86_64-conda-linux-gnu/12.4.0")
set(GCC_INC "${GCC_LIB}/include/c++")

# Target platform.
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

# Compilers (conda-forge clang from the Mac env).
set(CMAKE_C_COMPILER "${MAC_ASP_DEPS}/bin/clang")
set(CMAKE_CXX_COMPILER "${MAC_ASP_DEPS}/bin/clang++")

# Sysroot and GCC toolchain.
set(CMAKE_SYSROOT "${CROSS_SYSROOT}")

# Compiler flags.
set(CROSS_COMMON_FLAGS
  "--target=x86_64-unknown-linux-gnu \
   --gcc-toolchain=${LINUX_DEPS_PREFIX} \
   -B${GCC_LIB} \
   -fuse-ld=${MAC_ASP_DEPS}/bin/ld.lld \
   -I${CROSS_SYSROOT}/usr/include \
   -L${GCC_LIB} \
   -L${LINUX_DEPS_PREFIX}/lib")

set(CMAKE_C_FLAGS_INIT "${CROSS_COMMON_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT
  "${CROSS_COMMON_FLAGS} \
   -isystem ${GCC_INC} \
   -isystem ${GCC_INC}/x86_64-conda-linux-gnu \
   -Wno-enum-constexpr-conversion")

# Linker flags.
set(CMAKE_EXE_LINKER_FLAGS_INIT "-L${GCC_LIB} -L${LINUX_DEPS_PREFIX}/lib")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "-L${GCC_LIB} -L${LINUX_DEPS_PREFIX}/lib")

# Search only the cross-prefix for libraries and headers.
set(CMAKE_FIND_ROOT_PATH "${LINUX_DEPS_PREFIX}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
