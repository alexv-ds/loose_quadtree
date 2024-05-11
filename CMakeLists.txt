cmake_minimum_required(VERSION 3.21)

project(loose_quadtree
  VERSION 2.0.0
  DESCRIPTION "Loose Quadtree (Region Tree) simple implementation"
  HOMEPAGE_URL "https://github.com/alexv-ds/loose_quadtree"
  LANGUAGES CXX
)

if (PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
  message(FATAL_ERROR "In-source builds not allowed")
endif ()

set(QUADTREE_ENABLE_WALL ${PROJECT_IS_TOP_LEVEL} CACHE BOOL "Enable all warnings")
set(QUADTREE_ENABLE_WERROR ${PROJECT_IS_TOP_LEVEL} CACHE BOOL "Interpret warnings as errors")
set(QUADTREE_BUILD_TESTING ${PROJECT_IS_TOP_LEVEL} CACHE BOOL "Build tests")

include(CMakeDependentOption)
cmake_dependent_option(QUADTREE_CODE_COVERAGE "Enable code coverage" OFF "QUADTREE_BUILD_TESTING" OFF)
cmake_dependent_option(QUADTREE_CODE_COVERAGE_REPORT_XML "Add coverage_xml target" OFF "QUADTREE_CODE_COVERAGE" OFF)
cmake_dependent_option(QUADTREE_CODE_COVERAGE_REPORT_HTML "Add coverage_html target" ON "QUADTREE_CODE_COVERAGE" OFF)
# Automatically running tests when creating a coverage report is useful, but completely unnecessary in github actions
cmake_dependent_option(QUADTREE_CODE_COVERAGE_AUTORUN_TESTS "Automatically run test on coverage target" ON "QUADTREE_CODE_COVERAGE" OFF)

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

# load dependencies via CPM.cmake
# separate file for easy configuration of caching in github actions
include(cmake/cpm-dependencies.cmake)

if (QUADTREE_BUILD_TESTING)
  add_executable(tests_loose_quadtree
    test/loose_quadtree.test.cpp
  )
  target_link_libraries(tests_loose_quadtree PRIVATE Catch2::Catch2WithMain loose_quadtree::loose_quadtree)
  include(CTest)
  include(Catch)
  catch_discover_tests(tests_loose_quadtree)
endif ()

if (QUADTREE_CODE_COVERAGE)
  include(cmake/CodeCoverage.cmake)
  append_coverage_compiler_flags_to_target(tests_loose_quadtree)
  
  if (QUADTREE_CODE_COVERAGE_AUTORUN_TESTS)
    set(COVERAGE_EXECUTABLE_ARG "ctest")
  else ()
    set(COVERAGE_EXECUTABLE_ARG "")
  endif ()
  
  if (QUADTREE_CODE_COVERAGE_REPORT_HTML)
    setup_target_for_coverage_gcovr_html(
      NAME coverage_html
      EXECUTABLE "${COVERAGE_EXECUTABLE_ARG}"
      DEPENDENCIES tests_loose_quadtree
      BASE_DIRECTORY "${PROJECT_SOURCE_DIR}/src"
      EXCLUDE "${PROJECT_SOURCE_DIR}/test"
    )
  endif ()
  
  if (QUADTREE_CODE_COVERAGE_REPORT_XML)
    setup_target_for_coverage_gcovr_xml(
      NAME coverage_xml
      EXECUTABLE "${COVERAGE_EXECUTABLE_ARG}"
      DEPENDENCIES tests_loose_quadtree
      BASE_DIRECTORY "${PROJECT_SOURCE_DIR}/src"
      EXCLUDE "${PROJECT_SOURCE_DIR}/test"
    )
  endif ()
endif ()
