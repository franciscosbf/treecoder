cmake_minimum_required(VERSION 3.28)

project(treecoder
  VERSION 0.1.0
  DESCRIPTION "Huffman text encoder and decoder."
  LANGUAGES CXX
)

# Enforce C++ 17

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Make include files available

include_directories(include)

# Prepare dependencies

include(FetchContent)
set(FETCHCONTENT_QUIET FALSE)

FetchContent_Declare(argparse
  GIT_REPOSITORY https://github.com/p-ranav/argparse.git
  GIT_TAG v3.2
)
FetchContent_MakeAvailable(argparse)

FetchContent_Declare(googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.17.0
)
FetchContent_MakeAvailable(googletest)

set(FLATBUFFERS_BUILD_TESTS OFF)
set(FLATBUFFERS_INSTALL OFF)
set(FLATBUFFERS_BUILD_FLATC OFF)
FetchContent_Declare(flatbuffers
  GIT_REPOSITORY git@github.com:google/flatbuffers.git
  GIT_TAG v25.12.19
)
FetchContent_MakeAvailable(flatbuffers)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/deps/endian endian)

# Define project executable

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/treecoder.cpp
  src/file.cpp
)

target_compile_options(${PROJECT_NAME}
  PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
  $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

target_link_libraries(${PROJECT_NAME}
  argparse
  flatbuffers
  steinwurf::endian
)

# Define tests

enable_testing()

set(TREECODER_TEST treecoder_test)

add_executable(${TREECODER_TEST}
  test/treecoder_test.cpp
  src/treecoder.cpp
)

target_link_libraries(${TREECODER_TEST}
  GTest::gtest_main
  flatbuffers
  steinwurf::endian
)

include(GoogleTest)
gtest_discover_tests(${TREECODER_TEST})

# Make project-level variables available within the source code.

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/version.hpp.in
  ${CMAKE_CURRENT_SOURCE_DIR}/include/version.hpp
  @ONLY
)
