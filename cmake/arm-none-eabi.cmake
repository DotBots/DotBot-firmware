# gcc-arm-none-eabi toolchain file for DotBot-firmware CMake builds.
#
# Usage:
#   cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake -G Ninja

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Pre-empt CMake's compiler check, which would try to link a host binary.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Toolchain executables. Override on the command line with
#   -DARM_TOOLCHAIN_PREFIX=/path/to/bin/arm-none-eabi-
# if Homebrew / your installer puts them somewhere CMake can't find.
set(ARM_TOOLCHAIN_PREFIX "arm-none-eabi-" CACHE STRING "Toolchain command prefix")
set(CMAKE_C_COMPILER   ${ARM_TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER ${ARM_TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${ARM_TOOLCHAIN_PREFIX}g++)
set(CMAKE_OBJCOPY      ${ARM_TOOLCHAIN_PREFIX}objcopy CACHE INTERNAL "")
set(CMAKE_SIZE         ${ARM_TOOLCHAIN_PREFIX}size    CACHE INTERNAL "")
