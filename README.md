# loose_quadtree

[![CI](https://github.com/alexv-ds/loose_quadtree/actions/workflows/ci.yml/badge.svg?branch=master&event=push)](https://github.com/alexv-ds/loose_quadtree/actions)
[![codecov](https://codecov.io/gh/alexv-ds/loose_quadtree/graph/badge.svg?token=5V9SUJ5YT7)](https://codecov.io/gh/alexv-ds/loose_quadtree)
![GitHub](https://img.shields.io/github/license/alexv-ds/loose_quadtree)
![GitHub tag (with filter)](https://img.shields.io/github/v/tag/alexv-ds/loose_quadtree)

Loose Quadtree (Region Tree) simple C++11 implementation

---

Loose quadtree (unlike normal quadtrees which are for points only) is a region tree designed to store bounding boxes
effectively.
See boost::geometry::index::rtree for a more advanced, general solution!

This implementation features:

* Fully adaptive behavior, adjusts its bounds, height and memory on demand
* Every object will be stored on the level to which its size corresponds to
* Gives theoretically optimal search results (see previous)
* Uses tree structure instead of hashed (smaller memory footprint, cache friendly)
* Uses as much data in-place as it can (by using its own allocator)
* Allocates memory in big chunks
* Uses axis-aligned bounding boxes for calculations
* Uses left-top-width-height bounds for better precision (no right-bottom)
* Uses left-top closed right-bottom open interval logic (for integral types)
* Uses X-towards-right Y-towards-bottom screen-like coordinate system
* It is suitable for both floating- and fixed-point logic
* This library is not thread-safe but multiple queries can be run at once
* Generic parameters are:
  * NumberT generic number type allows its floating- and fixed-point usage
  * ObjectT* only pointer is stored, no object copying is done, not an inclusive container
  * BoundingBoxExtractorT allows using your own bounding box type/source (see code)

---

# Update 2024.05 (v2.0.0)

* Fully rewritten cmake project
* Rewritten tests using the Catch2 library
* Continuous integration into github-actions has been set up
* Added analysis of code coverage by tests using gcovr
* Added valgrind memcheck
* Fixed corrupted iterator assertions by MSVC
* Fixed MSVC warnings
* Fixed memory leak in BlockAllocator
* Refactoring CamelCase into sneak_case

# License

MIT. See LICENSE file