set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_SYSTEM_PROCESSOR aarch64)

# cross compilers
set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# avoid finding host includes/libraries
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -march=armv8-a -mtune=cortex-a72 -O3 -pipe -ftree-vectorize")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a -mtune=cortex-a72 -O3 -pipe -ftree-vectorize")

set(BLASFEO_TARGET "ARMV8A_ARM_CORTEX_A57")
