cmake_minimum_required(VERSION 3.21)

project(loose_quadtree
  VERSION 2.0.0
  DESCRIPTION "Loose Quadtree (Region Tree) simple implementation"
  HOMEPAGE_URL "https://github.com/alexv-ds/loose_quadtree"
  LANGUAGES CXX
)

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
  message(FATAL_ERROR "In-source builds not allowed")
endif()

set(QUADTREE_ENABLE_WALL ${PROJECT_IS_TOP_LEVEL} CACHE BOOL "Enable all warnings")
set(QUADTREE_ENABLE_WERROR ${PROJECT_IS_TOP_LEVEL} CACHE BOOL "Interpret warnings as errors")
set(QUADTREE_BUILD_TESTING ${PROJECT_IS_TOP_LEVEL} CACHE BOOL "Build tests")

add_library(loose_quadtree INTERFACE
  src/include/loose_quadtree/loose_quadtree.hpp
  src/loose_quadtree_impl.hpp
)
add_library(loose_quadtree::loose_quadtree ALIAS loose_quadtree)

target_include_directories(loose_quadtree INTERFACE
  src/include
)

set_target_properties(loose_quadtree PROPERTIES
  CXX_EXTENSIONS OFF
  CXX_STANDARD 11
  CXX_STANDARD_REQUIRED ON
)

target_compile_options(loose_quadtree INTERFACE
  # Enable -Wall flags if option QUADTREE_ENABLE_WALL enabled and compiler is MSVC
  $<$<BOOL:${QUADTREE_ENABLE_WALL}>:$<$<CXX_COMPILER_ID:MSVC>:/W4>>
  # Enable -Wall flags if option QUADTREE_ENABLE_WALL enabled and compiler is not MSVC
  $<$<BOOL:${QUADTREE_ENABLE_WALL}>:$<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>>
  # Enable -Werror flags if option QUADTREE_ENABLE_WERROR enabled and compiler is MSVC
  $<$<BOOL:${QUADTREE_ENABLE_WERROR}>:$<$<CXX_COMPILER_ID:MSVC>:/WX>>
  # Enable -Werror flags if option QUADTREE_ENABLE_WERROR enabled and compiler is not MSVC
  $<$<BOOL:${QUADTREE_ENABLE_WERROR}>:$<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Werror>>
)

if (QUADTREE_BUILD_TESTING)
  include(cmake/get_cpm.cmake)
  
  CPMAddPackage(
    NAME Catch2
    URL "https://github.com/catchorg/Catch2/archive/refs/tags/v3.6.0.tar.gz"
    URL_HASH "SHA3_256=3d6e6a833f3b0209b871c05f0658f5993b8613d2ea293084a1da04b53bfb9d90"
    OPTIONS
    "CATCH_INSTALL_DOCS OFF"
    "CATCH_INSTALL_EXTRAS OFF"
  )
  list(APPEND CMAKE_MODULE_PATH "${Catch2_SOURCE_DIR}/extras")
  
  add_executable(tests_loose_quadtree
    test/loose_quadtree.test.cpp
  )
  target_link_libraries(tests_loose_quadtree PRIVATE Catch2::Catch2WithMain loose_quadtree::loose_quadtree)
  include(CTest)
  include(Catch)
  catch_discover_tests(tests_loose_quadtree)
endif ()