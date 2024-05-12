#include <vector>
#include <random>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "loose_quadtree/loose_quadtree.hpp"

#define TYPES_FOR_TESTING \
  float, double, long double, int, long, short, \
  unsigned int, unsigned long, unsigned short, \
  long long

template<class NumberT>
class TrivialBBExtractor {
public:
  static void ExtractBoundingBox(const loose_quadtree::bounding_box<NumberT>* object,
                                 loose_quadtree::bounding_box<NumberT>* bbox) {
    bbox->left = object->left;
    bbox->top = object->top;
    bbox->width = object->width;
    bbox->height = object->height;
  }
};


TEMPLATE_TEST_CASE("TestBoundingBox", "", TYPES_FOR_TESTING) {
  loose_quadtree::bounding_box<TestType> big(100, 100, 200, 50);
  loose_quadtree::bounding_box<TestType> small_inside(200, 125, 5, 5);
  loose_quadtree::bounding_box<TestType> edge_inside(110, 110, 190, 40);
  loose_quadtree::bounding_box<TestType> edge_outside(300, 150, 20, 5);
  loose_quadtree::bounding_box<TestType> intersecting1(290, 90, 29, 25);
  loose_quadtree::bounding_box<TestType> intersecting2(290, 110, 29, 25);
  loose_quadtree::bounding_box<TestType> outside(290, 210, 29, 25);

  REQUIRE(big.contains(100, 100));
  REQUIRE_FALSE(big.contains(300, 150));
  REQUIRE(big.contains(big));
  REQUIRE(big.contains(small_inside));
  REQUIRE_FALSE(small_inside.contains(big));
  REQUIRE(big.contains(edge_inside));
  REQUIRE_FALSE(edge_inside.contains(big));
  REQUIRE_FALSE(big.contains(edge_outside));
  REQUIRE_FALSE(edge_outside.contains(big));
  REQUIRE_FALSE(big.contains(intersecting1));
  REQUIRE_FALSE(intersecting1.contains(big));
  REQUIRE_FALSE(big.contains(intersecting2));
  REQUIRE_FALSE(intersecting1.contains(big));
  REQUIRE_FALSE(intersecting1.contains(intersecting2));
  REQUIRE_FALSE(big.contains(outside));
  REQUIRE_FALSE(outside.contains(big));

  REQUIRE(big.intersects(big));
  REQUIRE(big.intersects(small_inside));
  REQUIRE(small_inside.intersects(big));
  REQUIRE(big.intersects(edge_inside));
  REQUIRE(edge_inside.intersects(big));
  REQUIRE_FALSE(big.intersects(edge_outside));
  REQUIRE_FALSE(edge_outside.intersects(big));
  REQUIRE(big.intersects(intersecting1));
  REQUIRE(intersecting1.intersects(big));
  REQUIRE(big.intersects(intersecting2));
  REQUIRE(intersecting2.intersects(big));
  REQUIRE(intersecting1.intersects(intersecting2));
  REQUIRE_FALSE(big.intersects(outside));
  REQUIRE_FALSE(outside.intersects(big));
}

TEMPLATE_TEST_CASE("TestForwardTreeTraversal", "", TYPES_FOR_TESTING) {
  loose_quadtree::detail::BlocksAllocator allocator;
  loose_quadtree::detail::ForwardTreeTraversal<TestType, loose_quadtree::bounding_box<TestType>> fortt;
  loose_quadtree::detail::TreeNode<loose_quadtree::bounding_box<TestType>> root(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::bounding_box<TestType>> tl(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::bounding_box<TestType>> tr(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::bounding_box<TestType>> br(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::bounding_box<TestType>> bl(allocator);
  root.top_left = &tl;
  tl.top_right = &tr;
  tr.bottom_right = &br;
  root.bottom_left = &bl;
  fortt.StartAt(&root, loose_quadtree::bounding_box<TestType>(0, 0, 64, 64));
  REQUIRE(fortt.GetDepth() == 0);
  REQUIRE(fortt.GetNode() == &root);
  REQUIRE(fortt.GetNodeBoundingBox().left == 0);
  REQUIRE(fortt.GetNodeBoundingBox().top == 0);
  REQUIRE(fortt.GetNodeBoundingBox().width == 64);
  REQUIRE(fortt.GetNodeBoundingBox().height == 64);
  fortt.GoTopLeft();
  REQUIRE(fortt.GetDepth() == 1);
  REQUIRE(fortt.GetNode() == &tl);
  REQUIRE(fortt.GetNodeBoundingBox().left == 0);
  REQUIRE(fortt.GetNodeBoundingBox().top == 0);
  REQUIRE(fortt.GetNodeBoundingBox().width == 32);
  REQUIRE(fortt.GetNodeBoundingBox().height == 32);
  fortt.GoTopRight();
  REQUIRE(fortt.GetDepth() == 2);
  REQUIRE(fortt.GetNode() == &tr);
  REQUIRE(fortt.GetNodeBoundingBox().left == 16);
  REQUIRE(fortt.GetNodeBoundingBox().top == 0);
  REQUIRE(fortt.GetNodeBoundingBox().width == 16);
  REQUIRE(fortt.GetNodeBoundingBox().height == 16);
  fortt.GoBottomRight();
  REQUIRE(fortt.GetDepth() == 3);
  REQUIRE(fortt.GetNode() == &br);
  REQUIRE(fortt.GetNodeBoundingBox().left == 24);
  REQUIRE(fortt.GetNodeBoundingBox().top == 8);
  REQUIRE(fortt.GetNodeBoundingBox().width == 8);
  REQUIRE(fortt.GetNodeBoundingBox().height == 8);
  fortt.StartAt(&root, loose_quadtree::bounding_box<TestType>(0, 0, 64, 64));
  REQUIRE(fortt.GetDepth() == 0);
  REQUIRE(fortt.GetNode() == &root);
  REQUIRE(fortt.GetNodeBoundingBox().left == 0);
  REQUIRE(fortt.GetNodeBoundingBox().top == 0);
  REQUIRE(fortt.GetNodeBoundingBox().width == 64);
  REQUIRE(fortt.GetNodeBoundingBox().height == 64);
  fortt.GoBottomLeft();
  REQUIRE(fortt.GetDepth() == 1);
  REQUIRE(fortt.GetNode() == &bl);
  REQUIRE(fortt.GetNodeBoundingBox().left == 0);
  REQUIRE(fortt.GetNodeBoundingBox().top == 32);
  REQUIRE(fortt.GetNodeBoundingBox().width == 32);
  REQUIRE(fortt.GetNodeBoundingBox().height == 32);
}

TEMPLATE_TEST_CASE("TestFullTreeTraversal", "", TYPES_FOR_TESTING) {
  loose_quadtree::detail::BlocksAllocator allocator;
  loose_quadtree::detail::FullTreeTraversal<TestType, loose_quadtree::bounding_box<TestType>> fultt;
  loose_quadtree::detail::TreeNode<loose_quadtree::bounding_box<TestType>> root(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::bounding_box<TestType>> tl(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::bounding_box<TestType>> tr(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::bounding_box<TestType>> br(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::bounding_box<TestType>> bl(allocator);
  root.top_left = &tl;
  tl.top_right = &tr;
  tr.bottom_right = &br;
  br.bottom_left = &bl;
  fultt.StartAt(&root, loose_quadtree::bounding_box<TestType>(0, 0, 64, 64));
  REQUIRE(fultt.GetDepth() == 0);
  REQUIRE(fultt.GetNode() == &root);
  REQUIRE(fultt.GetNodeCurrentChild() == loose_quadtree::detail::ChildPosition::kNone);
  fultt.GoTopLeft();
  REQUIRE(fultt.GetDepth() == 1);
  REQUIRE(fultt.GetNode() == &tl);
  REQUIRE(fultt.GetNodeCurrentChild() == loose_quadtree::detail::ChildPosition::kNone);
  fultt.GoTopRight();
  REQUIRE(fultt.GetDepth() == 2);
  REQUIRE(fultt.GetNode() == &tr);
  REQUIRE(fultt.GetNodeCurrentChild() == loose_quadtree::detail::ChildPosition::kNone);
  fultt.GoBottomRight();
  REQUIRE(fultt.GetDepth() == 3);
  REQUIRE(fultt.GetNode() == &br);
  REQUIRE(fultt.GetNodeCurrentChild() == loose_quadtree::detail::ChildPosition::kNone);
  fultt.GoBottomLeft();
  REQUIRE(fultt.GetDepth() == 4);
  REQUIRE(fultt.GetNode() == &bl);
  REQUIRE(fultt.GetNodeCurrentChild() == loose_quadtree::detail::ChildPosition::kNone);
  fultt.GoUp();
  REQUIRE(fultt.GetDepth() == 3);
  REQUIRE(fultt.GetNode() == &br);
  REQUIRE(fultt.GetNodeCurrentChild() == loose_quadtree::detail::ChildPosition::kBottomLeft);
  fultt.GoUp();
  REQUIRE(fultt.GetDepth() == 2);
  REQUIRE(fultt.GetNode() == &tr);
  REQUIRE(fultt.GetNodeCurrentChild() == loose_quadtree::detail::ChildPosition::kBottomRight);
  fultt.GoUp();
  REQUIRE(fultt.GetDepth() == 1);
  REQUIRE(fultt.GetNode() == &tl);
  REQUIRE(fultt.GetNodeCurrentChild() == loose_quadtree::detail::ChildPosition::kTopRight);
  fultt.GoUp();
  REQUIRE(fultt.GetDepth() == 0);
  REQUIRE(fultt.GetNode() == &root);
  REQUIRE(fultt.GetNodeCurrentChild() == loose_quadtree::detail::ChildPosition::kTopLeft);
}

TEMPLATE_TEST_CASE("TestBoundingBoxDiscrepancy", "", TYPES_FOR_TESTING) {
  loose_quadtree::detail::BlocksAllocator allocator;
  loose_quadtree::detail::FullTreeTraversal<TestType, loose_quadtree::bounding_box<TestType>> ftt;
  loose_quadtree::detail::TreeNode<loose_quadtree::bounding_box<TestType>> root(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::bounding_box<TestType>> tl(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::bounding_box<TestType>> tr(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::bounding_box<TestType>> br(allocator);
  root.top_left = &tl;
  root.top_right = &tr;
  root.bottom_right = &br;
  ftt.StartAt(&root, loose_quadtree::bounding_box<TestType>(10, 10, 17, 19));
  TestType orig_width = ftt.GetNodeBoundingBox().width;
  TestType orig_height = ftt.GetNodeBoundingBox().height;
  ftt.GoTopLeft();
  TestType tl_width = ftt.GetNodeBoundingBox().width;
  TestType tl_height = ftt.GetNodeBoundingBox().height;
  ftt.GoUp();
  ftt.GoTopRight();
  TestType tr_width = ftt.GetNodeBoundingBox().width;
  TestType tr_height = ftt.GetNodeBoundingBox().height;
  ftt.GoUp();
  ftt.GoBottomRight();
  TestType br_width = ftt.GetNodeBoundingBox().width;
  TestType br_height = ftt.GetNodeBoundingBox().height;
  ftt.GoUp();

  REQUIRE(orig_width == tl_width + tr_width);
  REQUIRE(orig_height == tl_height + br_height);
  REQUIRE(tr_width == br_width);
  REQUIRE(tl_height == tr_height);
}

TEMPLATE_TEST_CASE("TestInsertRemove", "", TYPES_FOR_TESTING) {
  const bool reclaim_losses = GENERATE(true, false);

  std::vector<loose_quadtree::bounding_box<TestType>> objects;
  objects.push_back({1000, 1300, 50, 30});
  objects.push_back({1060, 1300, 50, 30});
  objects.push_back({1060, 1300, 5, 3});
  loose_quadtree::quad_tree<TestType, loose_quadtree::bounding_box<TestType>, TrivialBBExtractor<TestType>> lqt;

  if (reclaim_losses) {
    lqt.force_cleanup();
  }

  REQUIRE(lqt.get_size() == 0);
  REQUIRE(lqt.is_empty());
  REQUIRE_FALSE(lqt.contains(&objects[0]));
  REQUIRE(lqt.get_size() == 0);
  lqt.insert(&objects[0]);
  REQUIRE(lqt.get_size() == 1);
  REQUIRE_FALSE(lqt.is_empty());
  REQUIRE(lqt.contains(&objects[0]));
  REQUIRE_FALSE(lqt.contains(&objects[1]));
  REQUIRE(lqt.get_loose_bounding_box().intersects(objects[0]));
  REQUIRE_FALSE(objects[0].contains(lqt.get_loose_bounding_box()));
  lqt.remove(&objects[0]);
  REQUIRE_FALSE(lqt.contains(&objects[0]));
  REQUIRE(lqt.get_size() == 0);
  REQUIRE(lqt.is_empty());

  if (reclaim_losses) {
    lqt.force_cleanup();
  }

  lqt.insert(&objects[1]);
  REQUIRE(lqt.get_size() == 1);
  REQUIRE_FALSE(lqt.contains(&objects[0]));
  REQUIRE(lqt.contains(&objects[1]));
  lqt.insert(&objects[0]);
  lqt.insert(&objects[0]);
  lqt.insert(&objects[0]);
  REQUIRE(lqt.get_size() == 2);
  REQUIRE(lqt.contains(&objects[0]));
  REQUIRE(lqt.contains(&objects[1]));
  REQUIRE_FALSE(lqt.contains(&objects[2]));

  if (reclaim_losses) {
    lqt.force_cleanup();
  }

  lqt.insert(&objects[2]);
  REQUIRE(lqt.get_size() == 3);
  REQUIRE(lqt.contains(&objects[0]));
  REQUIRE(lqt.contains(&objects[1]));
  REQUIRE(lqt.contains(&objects[2]));

  if (reclaim_losses) {
    lqt.force_cleanup();
  }

  REQUIRE(lqt.get_loose_bounding_box().intersects(objects[0]));
  REQUIRE_FALSE(objects[0].contains(lqt.get_loose_bounding_box()));
  REQUIRE(lqt.get_loose_bounding_box().intersects(objects[1]));
  REQUIRE_FALSE(objects[1].contains(lqt.get_loose_bounding_box()));
  REQUIRE(lqt.get_loose_bounding_box().intersects(objects[2]));
  REQUIRE_FALSE(objects[2].contains(lqt.get_loose_bounding_box()));
  lqt.remove(&objects[1]);
  lqt.remove(&objects[1]);
  lqt.remove(&objects[1]);
  REQUIRE(lqt.get_size() == 2);
  REQUIRE(lqt.contains(&objects[0]));
  REQUIRE_FALSE(lqt.contains(&objects[1]));
  REQUIRE(lqt.contains(&objects[2]));

  if (reclaim_losses) {
    lqt.force_cleanup();
  }

  lqt.remove(&objects[0]);
  REQUIRE(lqt.get_size() == 1);
  REQUIRE_FALSE(lqt.contains(&objects[0]));
  REQUIRE_FALSE(lqt.contains(&objects[1]));
  REQUIRE(lqt.contains(&objects[2]));

  if (reclaim_losses) {
    lqt.force_cleanup();
  }

  lqt.remove(&objects[0]);
  REQUIRE(lqt.get_size() == 1);
  REQUIRE_FALSE(lqt.contains(&objects[0]));
  REQUIRE_FALSE(lqt.contains(&objects[1]));
  REQUIRE(lqt.contains(&objects[2]));

  if (reclaim_losses) {
    lqt.force_cleanup();
  }

  lqt.remove(&objects[2]);
  REQUIRE(lqt.get_size() == 0);
  REQUIRE_FALSE(lqt.contains(&objects[0]));
  REQUIRE_FALSE(lqt.contains(&objects[1]));
  REQUIRE_FALSE(lqt.contains(&objects[2]));

  if (reclaim_losses) {
    lqt.force_cleanup();
  }
}

TEMPLATE_TEST_CASE("TestUpdate", "", TYPES_FOR_TESTING) {
  const bool reclaim_losses = GENERATE(true, false);

  std::vector<loose_quadtree::bounding_box<TestType>> objects;
  objects.push_back({1000, 1000, 50, 30});
  objects.push_back({1060, 1000, 50, 30});
  objects.push_back({1060, 1000, 5, 3});
  loose_quadtree::quad_tree<TestType, loose_quadtree::bounding_box<TestType>, TrivialBBExtractor<TestType>> lqt;

  if (reclaim_losses) {
    lqt.force_cleanup();
  }

  lqt.insert(&objects[0]);
  lqt.insert(&objects[1]);
  lqt.insert(&objects[2]);

  if (reclaim_losses) {
    lqt.force_cleanup();
  }

  REQUIRE(lqt.get_size() == 3);
  REQUIRE(lqt.contains(&objects[0]));
  REQUIRE(lqt.contains(&objects[1]));
  REQUIRE(lqt.contains(&objects[2]));
  REQUIRE(lqt.get_loose_bounding_box().intersects(objects[0]));
  REQUIRE(lqt.get_loose_bounding_box().intersects(objects[1]));
  REQUIRE(lqt.get_loose_bounding_box().intersects(objects[2]));
  REQUIRE_FALSE(objects[0].contains(lqt.get_loose_bounding_box()));
  REQUIRE_FALSE(objects[1].contains(lqt.get_loose_bounding_box()));
  REQUIRE_FALSE(objects[2].contains(lqt.get_loose_bounding_box()));
  objects[2].width = 50;
  objects[2].height = 30;
  lqt.update(&objects[2]);
  objects[0].left = 1060;
  lqt.update(&objects[0]);
  REQUIRE(lqt.get_size() == 3);
  REQUIRE(lqt.contains(&objects[0]));
  REQUIRE(lqt.contains(&objects[1]));
  REQUIRE(lqt.contains(&objects[2]));
  REQUIRE(lqt.get_loose_bounding_box().intersects(objects[0]));
  REQUIRE(lqt.get_loose_bounding_box().intersects(objects[1]));
  REQUIRE(lqt.get_loose_bounding_box().intersects(objects[2]));
  REQUIRE_FALSE(objects[0].contains(lqt.get_loose_bounding_box()));
  REQUIRE_FALSE(objects[1].contains(lqt.get_loose_bounding_box()));
  REQUIRE_FALSE(objects[2].contains(lqt.get_loose_bounding_box()));

  if (reclaim_losses) {
    lqt.force_cleanup();
  }

  REQUIRE(lqt.get_size() == 3);
  REQUIRE(lqt.contains(&objects[0]));
  REQUIRE(lqt.contains(&objects[1]));
  REQUIRE(lqt.contains(&objects[2]));
  REQUIRE(lqt.get_loose_bounding_box().intersects(objects[0]));
  REQUIRE(lqt.get_loose_bounding_box().intersects(objects[1]));
  REQUIRE(lqt.get_loose_bounding_box().intersects(objects[2]));
  REQUIRE_FALSE(objects[0].contains(lqt.get_loose_bounding_box()));
  REQUIRE_FALSE(objects[1].contains(lqt.get_loose_bounding_box()));
  REQUIRE_FALSE(objects[2].contains(lqt.get_loose_bounding_box()));
  lqt.remove(&objects[0]);
  REQUIRE(lqt.get_size() == 2);

  if (reclaim_losses) {
    lqt.force_cleanup();
  }

  lqt.remove(&objects[1]);
  REQUIRE(lqt.get_size() == 1);

  if (reclaim_losses) {
    lqt.force_cleanup();
  }

  lqt.remove(&objects[2]);
  REQUIRE(lqt.get_size() == 0);

  if (reclaim_losses) {
    lqt.force_cleanup();
  }
}

TEMPLATE_TEST_CASE("TestMoreTrees", "", TYPES_FOR_TESTING) {
  const bool reclaim_losses = GENERATE(true, false);

  std::vector<loose_quadtree::bounding_box<TestType>> objects;
  objects.push_back({1000, 1000, 50, 30});
  objects.push_back({1060, 1000, 50, 30});
  objects.push_back({1060, 1000, 5, 3});
  loose_quadtree::quad_tree<TestType, loose_quadtree::bounding_box<TestType>, TrivialBBExtractor<TestType>> lqt;

  if (reclaim_losses) {
    lqt.force_cleanup();
  }

  lqt.insert(&objects[0]);
  lqt.insert(&objects[1]);
  REQUIRE(lqt.get_size() == 2);
  REQUIRE(lqt.contains(&objects[0]));
  REQUIRE(lqt.contains(&objects[1]));
  REQUIRE(lqt.get_loose_bounding_box().intersects(objects[0]));
  REQUIRE(lqt.get_loose_bounding_box().intersects(objects[1]));
  REQUIRE_FALSE(objects[0].contains(lqt.get_loose_bounding_box()));
  REQUIRE_FALSE(objects[1].contains(lqt.get_loose_bounding_box()));

  if (reclaim_losses) {
    lqt.force_cleanup();
  }

  {
    loose_quadtree::quad_tree<TestType, loose_quadtree::bounding_box<TestType>, TrivialBBExtractor<TestType>> lqt2;

    if (reclaim_losses) {
      lqt2.force_cleanup();
    }

    REQUIRE(lqt2.get_size() == 0);
    lqt2.insert(&objects[1]);
    lqt2.insert(&objects[2]);
    lqt.insert(&objects[2]);

    if (reclaim_losses) {
      lqt.force_cleanup();
    }

    REQUIRE(lqt2.get_size() == 2);
    REQUIRE_FALSE(lqt2.contains(&objects[0]));
    REQUIRE(lqt2.contains(&objects[1]));
    REQUIRE(lqt2.contains(&objects[2]));

    if (reclaim_losses) {
      lqt2.force_cleanup();
    }

    lqt2.remove(&objects[1]);
    lqt2.remove(&objects[2]);
    REQUIRE(lqt2.get_size() == 0);

    if (reclaim_losses) {
      lqt2.force_cleanup();
    }
  }
  {
    loose_quadtree::quad_tree<TestType, loose_quadtree::bounding_box<TestType>, TrivialBBExtractor<TestType>> lqt2;
    lqt2.insert(&objects[1]);

    if (reclaim_losses) {
      lqt2.force_cleanup();
    }
  }
  {
    loose_quadtree::quad_tree<TestType, loose_quadtree::bounding_box<TestType>, TrivialBBExtractor<TestType>> lqt2;
    lqt2.insert(&objects[1]);
    lqt2.insert(&objects[2]);
    lqt2.insert(&objects[0]);
    REQUIRE(lqt2.contains(&objects[1]));
    lqt2.clear();
    REQUIRE(lqt2.get_size() == 0);
    REQUIRE_FALSE(lqt2.contains(&objects[1]));

    if (reclaim_losses) {
      lqt2.force_cleanup();
    }

    REQUIRE(lqt2.get_size() == 0);
    REQUIRE_FALSE(lqt2.contains(&objects[1]));
    lqt2.insert(&objects[1]);
    REQUIRE(lqt2.get_size() == 1);
    REQUIRE(lqt2.contains(&objects[1]));
  }
  REQUIRE(lqt.get_size() == 3);
  REQUIRE(lqt.contains(&objects[0]));
  REQUIRE(lqt.contains(&objects[1]));
  REQUIRE(lqt.contains(&objects[2]));
  REQUIRE(lqt.get_loose_bounding_box().intersects(objects[0]));
  REQUIRE(lqt.get_loose_bounding_box().intersects(objects[1]));
  REQUIRE(lqt.get_loose_bounding_box().intersects(objects[2]));
  REQUIRE_FALSE(objects[0].contains(lqt.get_loose_bounding_box()));
  REQUIRE_FALSE(objects[1].contains(lqt.get_loose_bounding_box()));
  REQUIRE_FALSE(objects[2].contains(lqt.get_loose_bounding_box()));
  lqt.remove(&objects[0]);
  lqt.remove(&objects[2]);
  REQUIRE_FALSE(lqt.contains(&objects[0]));
  REQUIRE(lqt.contains(&objects[1]));
  REQUIRE(lqt.get_size() == 1);

  if (reclaim_losses) {
    lqt.force_cleanup();
  }

  lqt.remove(&objects[1]);
  REQUIRE(lqt.get_size() == 0);

  if (reclaim_losses) {
    lqt.force_cleanup();
  }
}

TEMPLATE_TEST_CASE("TestQueryIntersects", "", TYPES_FOR_TESTING) {
  std::vector<loose_quadtree::bounding_box<TestType>> objects;
  objects.push_back({10000, 10000, 8000, 8000});//0
  objects.push_back({10000, 10000, 7000, 6000});//1
  objects.push_back({10000, 10000, 7, 6});//2
  objects.push_back({15000, 15000, 500, 600});//3
  objects.push_back({15100, 15100, 200, 200});//4
  objects.push_back({15000, 15000, 200, 200});//5
  objects.push_back({15100, 15100, 2, 2});//6
  loose_quadtree::quad_tree<TestType, loose_quadtree::bounding_box<TestType>, TrivialBBExtractor<TestType>> lqt;
  for (auto& obj: objects) {
    lqt.insert(&obj);
  }

  auto query = lqt.query_intersects_region(loose_quadtree::bounding_box<TestType>(33, 33, 1, 1));
  REQUIRE(query.end_of_query());

  query = lqt.query_intersects_region(loose_quadtree::bounding_box<TestType>(9000, 9000, 9000, 9000));
  int count = 0;
  while (!query.end_of_query()) {
    loose_quadtree::bounding_box<TestType>* obj = query.get_current();
    (void) obj;
    count++;
    query.next();
  }
  REQUIRE(count == 7);

  query = lqt.query_intersects_region(loose_quadtree::bounding_box<TestType>(10003, 10003, 3, 7));
  count = 0;
  while (!query.end_of_query()) {
    loose_quadtree::bounding_box<TestType>* obj = query.get_current();
    REQUIRE_FALSE((obj != &objects[0] && obj != &objects[1] && obj != &objects[2]));
    count++;
    query.next();
  }
  REQUIRE(count == 3);

  query = lqt.query_intersects_region(loose_quadtree::bounding_box<TestType>(14900, 14900, 200, 200));
  count = 0;
  while (!query.end_of_query()) {
    loose_quadtree::bounding_box<TestType>* obj = query.get_current();
    REQUIRE_FALSE((obj != &objects[0] && obj != &objects[1] && obj != &objects[3] && obj != &objects[5]));
    count++;
    query.next();
  }
  REQUIRE(count == 4);
}

TEMPLATE_TEST_CASE("TestQueryInside", "", TYPES_FOR_TESTING) {
  std::vector<loose_quadtree::bounding_box<TestType>> objects;
  objects.push_back({10000, 10000, 8000, 8000});//0
  objects.push_back({10000, 10000, 7000, 6000});//1
  objects.push_back({10000, 10000, 7, 6});//2
  objects.push_back({15000, 15000, 500, 600});//3
  objects.push_back({15100, 15100, 200, 200});//4
  objects.push_back({15000, 15000, 200, 200});//5
  objects.push_back({15100, 15100, 2, 2});//6
  loose_quadtree::quad_tree<TestType, loose_quadtree::bounding_box<TestType>, TrivialBBExtractor<TestType>> lqt;
  for (auto& obj: objects) {
    lqt.insert(&obj);
  }

  auto query = lqt.query_inside_region(loose_quadtree::bounding_box<TestType>(33, 33, 1, 1));
  REQUIRE(query.end_of_query());

  query = lqt.query_inside_region(loose_quadtree::bounding_box<TestType>(9000, 9000, 9000, 9000));
  int count = 0;
  while (!query.end_of_query()) {
    loose_quadtree::bounding_box<TestType>* obj = query.get_current();
    (void) obj;
    count++;
    query.next();
  }
  REQUIRE(count == 7);

  query = lqt.query_inside_region(loose_quadtree::bounding_box<TestType>(10003, 10003, 3, 7));
  REQUIRE(query.end_of_query());

  query = lqt.query_inside_region(loose_quadtree::bounding_box<TestType>(14900, 14900, 300, 300));
  count = 0;
  while (!query.end_of_query()) {
    loose_quadtree::bounding_box<TestType>* obj = query.get_current();
    REQUIRE_FALSE((obj != &objects[5] && obj != &objects[6]));
    count++;
    query.next();
  }
  REQUIRE(count == 2);
}

TEMPLATE_TEST_CASE("TestQueryContains", "", TYPES_FOR_TESTING) {
  std::vector<loose_quadtree::bounding_box<TestType>> objects;
  objects.push_back({10000, 10000, 8000, 8000});//0
  objects.push_back({10000, 10000, 7000, 6000});//1
  objects.push_back({10000, 10000, 7, 6});//2
  objects.push_back({15000, 15000, 500, 600});//3
  objects.push_back({15100, 15100, 200, 200});//4
  objects.push_back({15000, 15000, 200, 200});//5
  objects.push_back({15100, 15100, 2, 2});//6
  loose_quadtree::quad_tree<TestType, loose_quadtree::bounding_box<TestType>, TrivialBBExtractor<TestType>> lqt;
  for (auto& obj: objects) {
    lqt.insert(&obj);
  }

  auto query = lqt.query_contains_region(loose_quadtree::bounding_box<TestType>(33, 33, 1, 1));
  REQUIRE(query.end_of_query());

  query = lqt.query_contains_region(loose_quadtree::bounding_box<TestType>(9000, 9000, 9000, 9000));
  REQUIRE(query.end_of_query());

  query = lqt.query_contains_region(loose_quadtree::bounding_box<TestType>(10003, 10003, 3, 7));
  int count = 0;
  while (!query.end_of_query()) {
    loose_quadtree::bounding_box<TestType>* obj = query.get_current();
    REQUIRE_FALSE((obj != &objects[0] && obj != &objects[1]));
    count++;
    query.next();
  }
  REQUIRE(count == 2);

  query = lqt.query_contains_region(loose_quadtree::bounding_box<TestType>(14900, 14900, 200, 200));
  count = 0;
  while (!query.end_of_query()) {
    loose_quadtree::bounding_box<TestType>* obj = query.get_current();
    REQUIRE_FALSE((obj != &objects[0] && obj != &objects[1]));
    count++;
    query.next();
  }
  REQUIRE(count == 2);

  query = lqt.query_contains_region(loose_quadtree::bounding_box<TestType>(15000, 15000, 2, 2));
  count = 0;
  while (!query.end_of_query()) {
    loose_quadtree::bounding_box<TestType>* obj = query.get_current();
    REQUIRE_FALSE((obj != &objects[0] && obj != &objects[1] && obj != &objects[3] && obj != &objects[5]));
    count++;
    query.next();
  }
  REQUIRE(count == 4);
}

/*
// maybe banchmarck?
TEMPLATE_TEST_CASE("StressTest", "", TYPES_FOR_TESTING) {
#ifndef NDEBUG
  const int objects_generated = 10000;
  const int object_fluctuation = 1000;
#else
  const int objects_generated = 200000;
	const int object_fluctuation = 20000;
#endif
  const int full_rounds = 24;
  const int query_rounds = 4;
  std::minstd_rand rand;
  std::uniform_int_distribution<std::size_t> index(0, objects_generated - 1);
  std::uniform_real_distribution<float>
  coordinate(std::is_integral<TestType>::value ?
             (float)(std::is_signed<TestType>::value ?
                     -std::numeric_limits<TestType>::max() / 8 :
                     std::numeric_limits<TestType>::max() / 16 * 7) :
             -1.0f,
             std::is_integral<TestType>::value ?
             (float)(std::is_signed<TestType>::value ?
                     std::numeric_limits<TestType>::max() / 8 :
                     std::numeric_limits<TestType>::max() / 16 * 9) :
             1.0f);
  std::uniform_real_distribution<float>
    distance(0.0f, std::is_integral<TestType>::value ?
                   (float)(std::is_signed<TestType>::value ?
                           std::numeric_limits<TestType>::max() / 8 :
                           std::numeric_limits<TestType>::max() / 16) :
                   0.5f);

  std::vector<loose_quadtree::bbox<TestType>> objects;
  objects.reserve(objects_generated);
  std::vector<bool> flags(objects_generated, false);
  loose_quadtree::quad_tree<TestType, loose_quadtree::bbox<TestType>, TrivialBBExtractor<TestType>> lqt;
  for (std::size_t i = 0; i < objects_generated; i++) {
    objects.emplace_back((TestType)coordinate(rand), (TestType)coordinate(rand),
                         (TestType)distance(rand), (TestType)distance(rand));
    lqt.insert(&objects[i]);
  }
  REQUIRE(objects.size() == objects_generated);
  REQUIRE(flags.size() == objects_generated);

  for (int round = 0; round < full_rounds; round++) {
    for (int fluctobj = 0; fluctobj < object_fluctuation; fluctobj++) {
      std::size_t id = index(rand);
      objects[id] = loose_quadtree::bbox<TestType>((TestType)coordinate(rand), (TestType)coordinate(rand),
                                         (TestType)distance(rand), (TestType)distance(rand));
      lqt.update(&objects[id]);
    }
    for (int query_round = 0; query_round < query_rounds; query_round++) {
      loose_quadtree::bbox<TestType> query_region((TestType)coordinate(rand), (TestType)coordinate(rand),
                                        (TestType)distance(rand), (TestType)distance(rand));
      for (std::size_t i = 0; i < objects_generated; i++) {
        flags[i] = false;
      }

      auto query = lqt.query_intersects_region(query_region);
      while (!query.EndOfQuery()) {
        loose_quadtree::bbox<TestType>* obj = query.GetCurrent();
        REQUIRE(query_region.Intersects(*obj));
        auto id = (std::size_t)(obj - &objects[0]);
        REQUIRE((id >= 0 && id < objects_generated));
        flags[id] = true;
        query.Next();
      }
      for (std::size_t i = 0; i < objects_generated; i++) {
        REQUIRE(flags[i] == query_region.intersects(objects[i]));
        flags[i] = false;
      }

      query = lqt.query_inside_region(query_region);
      while (!query.end_of_query()) {
        loose_quadtree::bbox<TestType>* obj = query.GetCurrent();
        REQUIRE(query_region.Contains(*obj));
        std::size_t id = (std::size_t)(obj - &objects[0]);
        REQUIRE((id >= 0 && id < objects_generated));
        flags[id] = true;
        query.Next();
      }
      for (std::size_t i = 0; i < objects_generated; i++) {
        REQUIRE(flags[i] == query_region.contains(objects[i]));
        flags[i] = false;
      }

      query = lqt.query_contains_region(query_region);
      while (!query.end_of_query()) {
        loose_quadtree::bbox<TestType>* obj = query.get_current();
        REQUIRE(obj->contains(query_region));
        std::size_t id = (std::size_t)(obj - &objects[0]);
        REQUIRE((id >= 0 && id < objects_generated));
        flags[id] = true;
        query.next();
      }
      for (std::size_t i = 0; i < objects_generated; i++) {
        REQUIRE(flags[i] == objects[i].contains(query_region));
        //flags[i] = false;
      }
    }
  }
  lqt.force_cleanup();
}
//*/
