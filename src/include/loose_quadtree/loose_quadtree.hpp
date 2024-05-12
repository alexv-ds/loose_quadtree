#ifndef LOOSEQUADTREE_LOOSE_QUADTREE_HPP
#define LOOSEQUADTREE_LOOSE_QUADTREE_HPP

/*
 * Under MIT License
 * See LICENSE file
 */

namespace loose_quadtree {

  template<typename NumberT>
  struct bounding_box {
    using Number = NumberT;

    bounding_box(Number _left, Number _top, Number _width, Number _height) :
      left(_left), top(_top), width(_width), height(_height) {}

    bool intersects(const bounding_box<Number>& other) const;

    bool contains(const bounding_box<Number>& other) const;

    bool contains(Number x, Number y) const;

    Number left;
    Number top;
    Number width;
    Number height;
  };


  template<typename NumberT, typename ObjectT, typename BoundingBoxExtractorT>
  class quad_tree {
  public:
    using Number = NumberT;
    using Object = ObjectT;
    using BoundingBoxExtractor = BoundingBoxExtractorT;

  private:
    class impl;

  public:
    class query {
    public:
      ~query();

      query() = delete;

      query(const query&) = delete;

      query& operator=(const query&) = delete;

      query(query&&) noexcept;

      query& operator=(query&&);

      bool end_of_query() const;

      Object* get_current() const;

      void next();

    private:
      friend class quad_tree<Number, Object, BoundingBoxExtractor>::impl;

      class Impl;

      explicit query(Impl* pimpl);

      Impl* pimpl_;
    };

    quad_tree() = default;

    ~quad_tree() = default;

    quad_tree(const quad_tree&) = delete;

    quad_tree& operator=(const quad_tree&) = delete;

    bool insert(Object* object); ///< true if it was inserted (else updated)
    bool update(Object* object); ///< true if it was updated (else inserted)
    bool remove(Object* object); ///< true if it was removed
    bool contains(Object* object) const; ///< true if object is in tree
    query query_intersects_region(const bounding_box<Number>& region);

    query query_inside_region(const bounding_box<Number>& region);

    query query_contains_region(const bounding_box<Number>& region);

    const bounding_box<Number>& get_loose_bounding_box() const;

    ///< double its size to get a bounding box including everything contained for sure
    int get_size() const;

    bool is_empty() const;

    void clear();

    void force_cleanup(); ///< does a full data structure and memory cleanup
    ///< cleanup is semi-automatic during queries so you needn't call this normally

  private:
    impl impl_;
  };

} //loose_quadtree

#include "../../loose_quadtree_impl.hpp"

#endif //LOOSEQUADTREE_LOOSE_QUADTREE_HPP
