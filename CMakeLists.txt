include(ExternalProject)

cmake_minimum_required(VERSION 3.16)
project(mpcc LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enable with: cmake -DENABLE_ASAN=ON ..
option(ENABLE_ASAN "Build with AddressSanitizer" OFF)

if (ENABLE_ASAN)
    message(STATUS "AddressSanitizer enabled")

    # GCC/Clang flags
    set(ASAN_FLAGS 
        "-fsanitize=address"
        "-fno-omit-frame-pointer"
    )

    # Apply to all targets (or restrict to specific ones)
    add_compile_options(${ASAN_FLAGS})
    add_link_options(${ASAN_FLAGS})
endif()

# option(ANDROID_BUILD "Build for Android with NDK" OFF)

# --------------------------------------------------
# Optional Android cross-compile
# Usage for Android:
#   cmake -B build-android \
#     -DANDROID_BUILD=ON \
#     -DCMAKE_TOOLCHAIN_FILE=$NDK/build/cmake/android.toolchain.cmake \
#     -DANDROID_ABI=arm64-v8a \
#     -DANDROID_PLATFORM=android-24
# --------------------------------------------------
#
# if(ANDROID_BUILD)
#     message(STATUS "Configuring for Android ARMv8-A (Cortex-A53)")
#     set(BLASFEO_TARGET "ARMV8A_ARM_CORTEX_A53" STRING "" FORCE)
# else()
#     message(STATUS "Configuring for native x86_64 (Haswell)")
#     set(BLASFEO_TARGET "X64_INTEL_HASWELL" STRING "" FORCE)
# endif()

ExternalProject_Add(
    blasfeo
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/deps/blasfeo
    CMAKE_ARGS 
        -DTARGET=${BLASFEO_TARGET} 
        -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
        -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/blasfeo-install
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        -DBUILD_SHARED_LIBS=OFF
        -DCMAKE_C_FLAGS=-ffunction-sections
        -DCMAKE_CXX_FLAGS=-ffunction-sections
)

ExternalProject_Add(
    fatrop
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/deps/fatrop
    CMAKE_ARGS 
        -DBUILD_TESTS=OFF
        -DBUILD_EXAMPLES=OFF
        -DBUILD_WITH_LEGACY=ON 
        -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
        -DCMAKE_PREFIX_PATH=${CMAKE_BINARY_DIR}/blasfeo-install
        -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/fatrop-install
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        -DCMAKE_C_FLAGS=-ffunction-sections
        -DCMAKE_CXX_FLAGS=-ffunction-sections
)

# Your app source files, including fatrop usage and casadi_codegen wrappers
set(SRC
    src/cpp/solver.cpp
    #src/solver_jni.cpp
    codegen/f.c  # generated code as part of build
    src/cpp/solver_daemon.cpp
)

#add_library(solver SHARED ${SRC})
add_executable(solver ${SRC})

target_include_directories(solver PRIVATE
    ${CMAKE_SOURCE_DIR}/codegen
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}/blasfeo-install/include
    ${CMAKE_BINARY_DIR}/fatrop-install/include
)

target_link_libraries(solver PRIVATE
 "-Wl,-whole-archive" 
        ${CMAKE_BINARY_DIR}/blasfeo-install/lib/libblasfeo.a
        ${CMAKE_BINARY_DIR}/fatrop-install/lib/libfatrop.a
    "-Wl,-no-whole-archive"
)

# Enable linker to remove unused functions
set_target_properties(solver PROPERTIES
    LINK_FLAGS "-Wl,--gc-sections"
)

set(VENV_PYTHON "${CMAKE_SOURCE_DIR}/venv/bin/python")
set(CODEGEN_DIR "${CMAKE_SOURCE_DIR}/codegen")

# Determine if codegen should run
message(STATUS "Running codegen")
# Delete all files inside codegen
file(GLOB_RECURSE CODEGEN_FILES "${CODEGEN_DIR}/*")
if(CODEGEN_FILES)
    file(REMOVE ${CODEGEN_FILES})
endif()

# Run the Python script
execute_process(
    COMMAND ${VENV_PYTHON} "${CMAKE_SOURCE_DIR}/src/py/gen_mpc.py" "${CODEGEN_DIR}/"
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    RESULT_VARIABLE result
    OUTPUT_VARIABLE output
    ERROR_VARIABLE error
)

if(NOT result EQUAL 0)
    message(FATAL_ERROR "Python codegen script failed: ${error}")
endif()
