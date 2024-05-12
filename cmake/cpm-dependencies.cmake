if (QUADTREE_BUILD_TESTING)
  include(${PROJECT_SOURCE_DIR}/cmake/get_cpm.cmake)
  
  CPMAddPackage(
    NAME Catch2
    URL "https://github.com/catchorg/Catch2/archive/refs/tags/v3.6.0.tar.gz"
    URL_HASH "SHA3_256=3d6e6a833f3b0209b871c05f0658f5993b8613d2ea293084a1da04b53bfb9d90"
    OPTIONS
    "CATCH_INSTALL_DOCS OFF"
    "CATCH_INSTALL_EXTRAS OFF"
  )
  list(APPEND CMAKE_MODULE_PATH "${Catch2_SOURCE_DIR}/extras")
endif ()