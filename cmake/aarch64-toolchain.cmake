# aarch64 toolchain file.
#
# In practice this project is built NATIVELY on the Jetson (abra), which is
# already aarch64/Linux, so cross-compilation from the Mac is not used and this
# file is optional. It exists to (a) document the target and (b) support a
# future cross-build host if ever needed.
#
# To use a cross-build host, pass:
#   -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-toolchain.cmake
# and set the *_PROG cache vars below to your cross toolchain.

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# When building natively on the Jetson, leave these as the default system
# compilers. For a cross build, point them at aarch64-linux-gnu-{gcc,g++}.
if(DEFINED ENV{AARCH64_CC})
  set(CMAKE_C_COMPILER $ENV{AARCH64_CC})
endif()
if(DEFINED ENV{AARCH64_CXX})
  set(CMAKE_CXX_COMPILER $ENV{AARCH64_CXX})
endif()
