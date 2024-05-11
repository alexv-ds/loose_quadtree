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
  static void ExtractBoundingBox(const loose_quadtree::BoundingBox<NumberT>* object,
                                 loose_quadtree::BoundingBox<NumberT>* bbox) {
    bbox->left = object->left;
    bbox->top = object->top;
    bbox->width = object->width;
    bbox->height = object->height;
  }
};


TEMPLATE_TEST_CASE("TestBoundingBox", "", TYPES_FOR_TESTING) {
  loose_quadtree::BoundingBox<TestType> big(100, 100, 200, 50);
  loose_quadtree::BoundingBox<TestType> small_inside(200, 125, 5, 5);
  loose_quadtree::BoundingBox<TestType> edge_inside(110, 110, 190, 40);
  loose_quadtree::BoundingBox<TestType> edge_outside(300, 150, 20, 5);
  loose_quadtree::BoundingBox<TestType> intersecting1(290, 90, 29, 25);
  loose_quadtree::BoundingBox<TestType> intersecting2(290, 110, 29, 25);
  loose_quadtree::BoundingBox<TestType> outside(290, 210, 29, 25);

  REQUIRE(big.Contains(100, 100));
  REQUIRE_FALSE(big.Contains(300, 150));
  REQUIRE(big.Contains(big));
  REQUIRE(big.Contains(small_inside));
  REQUIRE_FALSE(small_inside.Contains(big));
  REQUIRE(big.Contains(edge_inside));
  REQUIRE_FALSE(edge_inside.Contains(big));
  REQUIRE_FALSE(big.Contains(edge_outside));
  REQUIRE_FALSE(edge_outside.Contains(big));
  REQUIRE_FALSE(big.Contains(intersecting1));
  REQUIRE_FALSE(intersecting1.Contains(big));
  REQUIRE_FALSE(big.Contains(intersecting2));
  REQUIRE_FALSE(intersecting1.Contains(big));
  REQUIRE_FALSE(intersecting1.Contains(intersecting2));
  REQUIRE_FALSE(big.Contains(outside));
  REQUIRE_FALSE(outside.Contains(big));

  REQUIRE(big.Intersects(big));
  REQUIRE(big.Intersects(small_inside));
  REQUIRE(small_inside.Intersects(big));
  REQUIRE(big.Intersects(edge_inside));
  REQUIRE(edge_inside.Intersects(big));
  REQUIRE_FALSE(big.Intersects(edge_outside));
  REQUIRE_FALSE(edge_outside.Intersects(big));
  REQUIRE(big.Intersects(intersecting1));
  REQUIRE(intersecting1.Intersects(big));
  REQUIRE(big.Intersects(intersecting2));
  REQUIRE(intersecting2.Intersects(big));
  REQUIRE(intersecting1.Intersects(intersecting2));
  REQUIRE_FALSE(big.Intersects(outside));
  REQUIRE_FALSE(outside.Intersects(big));
}

TEMPLATE_TEST_CASE("TestForwardTreeTraversal", "", TYPES_FOR_TESTING) {
  loose_quadtree::detail::BlocksAllocator allocator;
  loose_quadtree::detail::ForwardTreeTraversal<TestType, loose_quadtree::BoundingBox<TestType>> fortt;
  loose_quadtree::detail::TreeNode<loose_quadtree::BoundingBox<TestType>> root(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::BoundingBox<TestType>> tl(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::BoundingBox<TestType>> tr(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::BoundingBox<TestType>> br(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::BoundingBox<TestType>> bl(allocator);
  root.top_left = &tl;
  tl.top_right = &tr;
  tr.bottom_right = &br;
  root.bottom_left = &bl;
  fortt.StartAt(&root, loose_quadtree::BoundingBox<TestType>(0, 0, 64, 64));
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
  fortt.StartAt(&root, loose_quadtree::BoundingBox<TestType>(0, 0, 64, 64));
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
  loose_quadtree::detail::FullTreeTraversal<TestType, loose_quadtree::BoundingBox<TestType>> fultt;
  loose_quadtree::detail::TreeNode<loose_quadtree::BoundingBox<TestType>> root(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::BoundingBox<TestType>> tl(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::BoundingBox<TestType>> tr(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::BoundingBox<TestType>> br(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::BoundingBox<TestType>> bl(allocator);
  root.top_left = &tl;
  tl.top_right = &tr;
  tr.bottom_right = &br;
  br.bottom_left = &bl;
  fultt.StartAt(&root, loose_quadtree::BoundingBox<TestType>(0, 0, 64, 64));
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
  loose_quadtree::detail::FullTreeTraversal<TestType, loose_quadtree::BoundingBox<TestType>> ftt;
  loose_quadtree::detail::TreeNode<loose_quadtree::BoundingBox<TestType>> root(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::BoundingBox<TestType>> tl(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::BoundingBox<TestType>> tr(allocator);
  loose_quadtree::detail::TreeNode<loose_quadtree::BoundingBox<TestType>> br(allocator);
  root.top_left = &tl;
  root.top_right = &tr;
  root.bottom_right = &br;
  ftt.StartAt(&root, loose_quadtree::BoundingBox<TestType>(10, 10, 17, 19));
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

  std::vector<loose_quadtree::BoundingBox<TestType>> objects;
  objects.push_back({1000, 1300, 50, 30});
  objects.push_back({1060, 1300, 50, 30});
  objects.push_back({1060, 1300, 5, 3});
  loose_quadtree::LooseQuadtree<TestType, loose_quadtree::BoundingBox<TestType>, TrivialBBExtractor<TestType>> lqt;

  if (reclaim_losses) {
    lqt.ForceCleanup();
  }

  REQUIRE(lqt.GetSize() == 0);
  REQUIRE(lqt.IsEmpty());
  REQUIRE_FALSE(lqt.Contains(&objects[0]));
  REQUIRE(lqt.GetSize() == 0);
  lqt.Insert(&objects[0]);
  REQUIRE(lqt.GetSize() == 1);
  REQUIRE_FALSE(lqt.IsEmpty());
  REQUIRE(lqt.Contains(&objects[0]));
  REQUIRE_FALSE(lqt.Contains(&objects[1]));
  REQUIRE(lqt.GetLooseBoundingBox().Intersects(objects[0]));
  REQUIRE_FALSE(objects[0].Contains(lqt.GetLooseBoundingBox()));
  lqt.Remove(&objects[0]);
  REQUIRE_FALSE(lqt.Contains(&objects[0]));
  REQUIRE(lqt.GetSize() == 0);
  REQUIRE(lqt.IsEmpty());

  if (reclaim_losses) {
    lqt.ForceCleanup();
  }

  lqt.Insert(&objects[1]);
  REQUIRE(lqt.GetSize() == 1);
  REQUIRE_FALSE(lqt.Contains(&objects[0]));
  REQUIRE(lqt.Contains(&objects[1]));
  lqt.Insert(&objects[0]);
  lqt.Insert(&objects[0]);
  lqt.Insert(&objects[0]);
  REQUIRE(lqt.GetSize() == 2);
  REQUIRE(lqt.Contains(&objects[0]));
  REQUIRE(lqt.Contains(&objects[1]));
  REQUIRE_FALSE(lqt.Contains(&objects[2]));

  if (reclaim_losses) {
    lqt.ForceCleanup();
  }

  lqt.Insert(&objects[2]);
  REQUIRE(lqt.GetSize() == 3);
  REQUIRE(lqt.Contains(&objects[0]));
  REQUIRE(lqt.Contains(&objects[1]));
  REQUIRE(lqt.Contains(&objects[2]));

  if (reclaim_losses) {
    lqt.ForceCleanup();
  }

  REQUIRE(lqt.GetLooseBoundingBox().Intersects(objects[0]));
  REQUIRE_FALSE(objects[0].Contains(lqt.GetLooseBoundingBox()));
  REQUIRE(lqt.GetLooseBoundingBox().Intersects(objects[1]));
  REQUIRE_FALSE(objects[1].Contains(lqt.GetLooseBoundingBox()));
  REQUIRE(lqt.GetLooseBoundingBox().Intersects(objects[2]));
  REQUIRE_FALSE(objects[2].Contains(lqt.GetLooseBoundingBox()));
  lqt.Remove(&objects[1]);
  lqt.Remove(&objects[1]);
  lqt.Remove(&objects[1]);
  REQUIRE(lqt.GetSize() == 2);
  REQUIRE(lqt.Contains(&objects[0]));
  REQUIRE_FALSE(lqt.Contains(&objects[1]));
  REQUIRE(lqt.Contains(&objects[2]));

  if (reclaim_losses) {
    lqt.ForceCleanup();
  }

  lqt.Remove(&objects[0]);
  REQUIRE(lqt.GetSize() == 1);
  REQUIRE_FALSE(lqt.Contains(&objects[0]));
  REQUIRE_FALSE(lqt.Contains(&objects[1]));
  REQUIRE(lqt.Contains(&objects[2]));

  if (reclaim_losses) {
    lqt.ForceCleanup();
  }

  lqt.Remove(&objects[0]);
  REQUIRE(lqt.GetSize() == 1);
  REQUIRE_FALSE(lqt.Contains(&objects[0]));
  REQUIRE_FALSE(lqt.Contains(&objects[1]));
  REQUIRE(lqt.Contains(&objects[2]));

  if (reclaim_losses) {
    lqt.ForceCleanup();
  }

  lqt.Remove(&objects[2]);
  REQUIRE(lqt.GetSize() == 0);
  REQUIRE_FALSE(lqt.Contains(&objects[0]));
  REQUIRE_FALSE(lqt.Contains(&objects[1]));
  REQUIRE_FALSE(lqt.Contains(&objects[2]));

  if (reclaim_losses) {
    lqt.ForceCleanup();
  }
}

TEMPLATE_TEST_CASE("TestUpdate", "", TYPES_FOR_TESTING) {
  const bool reclaim_losses = GENERATE(true, false);

  std::vector<loose_quadtree::BoundingBox<TestType>> objects;
  objects.push_back({1000, 1000, 50, 30});
  objects.push_back({1060, 1000, 50, 30});
  objects.push_back({1060, 1000, 5, 3});
  loose_quadtree::LooseQuadtree<TestType, loose_quadtree::BoundingBox<TestType>, TrivialBBExtractor<TestType>> lqt;

  if (reclaim_losses) {
    lqt.ForceCleanup();
  }

  lqt.Insert(&objects[0]);
  lqt.Insert(&objects[1]);
  lqt.Insert(&objects[2]);

  if (reclaim_losses) {
    lqt.ForceCleanup();
  }

  REQUIRE(lqt.GetSize() == 3);
  REQUIRE(lqt.Contains(&objects[0]));
  REQUIRE(lqt.Contains(&objects[1]));
  REQUIRE(lqt.Contains(&objects[2]));
  REQUIRE(lqt.GetLooseBoundingBox().Intersects(objects[0]));
  REQUIRE(lqt.GetLooseBoundingBox().Intersects(objects[1]));
  REQUIRE(lqt.GetLooseBoundingBox().Intersects(objects[2]));
  REQUIRE_FALSE(objects[0].Contains(lqt.GetLooseBoundingBox()));
  REQUIRE_FALSE(objects[1].Contains(lqt.GetLooseBoundingBox()));
  REQUIRE_FALSE(objects[2].Contains(lqt.GetLooseBoundingBox()));
  objects[2].width = 50;
  objects[2].height = 30;
  lqt.Update(&objects[2]);
  objects[0].left = 1060;
  lqt.Update(&objects[0]);
  REQUIRE(lqt.GetSize() == 3);
  REQUIRE(lqt.Contains(&objects[0]));
  REQUIRE(lqt.Contains(&objects[1]));
  REQUIRE(lqt.Contains(&objects[2]));
  REQUIRE(lqt.GetLooseBoundingBox().Intersects(objects[0]));
  REQUIRE(lqt.GetLooseBoundingBox().Intersects(objects[1]));
  REQUIRE(lqt.GetLooseBoundingBox().Intersects(objects[2]));
  REQUIRE_FALSE(objects[0].Contains(lqt.GetLooseBoundingBox()));
  REQUIRE_FALSE(objects[1].Contains(lqt.GetLooseBoundingBox()));
  REQUIRE_FALSE(objects[2].Contains(lqt.GetLooseBoundingBox()));

  if (reclaim_losses) {
    lqt.ForceCleanup();
  }

  REQUIRE(lqt.GetSize() == 3);
  REQUIRE(lqt.Contains(&objects[0]));
  REQUIRE(lqt.Contains(&objects[1]));
  REQUIRE(lqt.Contains(&objects[2]));
  REQUIRE(lqt.GetLooseBoundingBox().Intersects(objects[0]));
  REQUIRE(lqt.GetLooseBoundingBox().Intersects(objects[1]));
  REQUIRE(lqt.GetLooseBoundingBox().Intersects(objects[2]));
  REQUIRE_FALSE(objects[0].Contains(lqt.GetLooseBoundingBox()));
  REQUIRE_FALSE(objects[1].Contains(lqt.GetLooseBoundingBox()));
  REQUIRE_FALSE(objects[2].Contains(lqt.GetLooseBoundingBox()));
  lqt.Remove(&objects[0]);
  REQUIRE(lqt.GetSize() == 2);

  if (reclaim_losses) {
    lqt.ForceCleanup();
  }

  lqt.Remove(&objects[1]);
  REQUIRE(lqt.GetSize() == 1);

  if (reclaim_losses) {
    lqt.ForceCleanup();
  }

  lqt.Remove(&objects[2]);
  REQUIRE(lqt.GetSize() == 0);

  if (reclaim_losses) {
    lqt.ForceCleanup();
  }
}

TEMPLATE_TEST_CASE("TestMoreTrees", "", TYPES_FOR_TESTING) {
  const bool reclaim_losses = GENERATE(true, false);

  std::vector<loose_quadtree::BoundingBox<TestType>> objects;
  objects.push_back({1000, 1000, 50, 30});
  objects.push_back({1060, 1000, 50, 30});
  objects.push_back({1060, 1000, 5, 3});
  loose_quadtree::LooseQuadtree<TestType, loose_quadtree::BoundingBox<TestType>, TrivialBBExtractor<TestType>> lqt;

  if (reclaim_losses) {
    lqt.ForceCleanup();
  }

  lqt.Insert(&objects[0]);
  lqt.Insert(&objects[1]);
  REQUIRE(lqt.GetSize() == 2);
  REQUIRE(lqt.Contains(&objects[0]));
  REQUIRE(lqt.Contains(&objects[1]));
  REQUIRE(lqt.GetLooseBoundingBox().Intersects(objects[0]));
  REQUIRE(lqt.GetLooseBoundingBox().Intersects(objects[1]));
  REQUIRE_FALSE(objects[0].Contains(lqt.GetLooseBoundingBox()));
  REQUIRE_FALSE(objects[1].Contains(lqt.GetLooseBoundingBox()));

  if (reclaim_losses) {
    lqt.ForceCleanup();
  }

  {
    loose_quadtree::LooseQuadtree<TestType, loose_quadtree::BoundingBox<TestType>, TrivialBBExtractor<TestType>> lqt2;

    if (reclaim_losses) {
      lqt2.ForceCleanup();
    }

    REQUIRE(lqt2.GetSize() == 0);
    lqt2.Insert(&objects[1]);
    lqt2.Insert(&objects[2]);
    lqt.Insert(&objects[2]);

    if (reclaim_losses) {
      lqt.ForceCleanup();
    }

    REQUIRE(lqt2.GetSize() == 2);
    REQUIRE_FALSE(lqt2.Contains(&objects[0]));
    REQUIRE(lqt2.Contains(&objects[1]));
    REQUIRE(lqt2.Contains(&objects[2]));

    if (reclaim_losses) {
      lqt2.ForceCleanup();
    }

    lqt2.Remove(&objects[1]);
    lqt2.Remove(&objects[2]);
    REQUIRE(lqt2.GetSize() == 0);

    if (reclaim_losses) {
      lqt2.ForceCleanup();
    }
  }
  {
    loose_quadtree::LooseQuadtree<TestType, loose_quadtree::BoundingBox<TestType>, TrivialBBExtractor<TestType>> lqt2;
    lqt2.Insert(&objects[1]);

    if (reclaim_losses) {
      lqt2.ForceCleanup();
    }
  }
  {
    loose_quadtree::LooseQuadtree<TestType, loose_quadtree::BoundingBox<TestType>, TrivialBBExtractor<TestType>> lqt2;
    lqt2.Insert(&objects[1]);
    lqt2.Insert(&objects[2]);
    lqt2.Insert(&objects[0]);
    REQUIRE(lqt2.Contains(&objects[1]));
    lqt2.Clear();
    REQUIRE(lqt2.GetSize() == 0);
    REQUIRE_FALSE(lqt2.Contains(&objects[1]));

    if (reclaim_losses) {
      lqt2.ForceCleanup();
    }

    REQUIRE(lqt2.GetSize() == 0);
    REQUIRE_FALSE(lqt2.Contains(&objects[1]));
    lqt2.Insert(&objects[1]);
    REQUIRE(lqt2.GetSize() == 1);
    REQUIRE(lqt2.Contains(&objects[1]));
  }
  REQUIRE(lqt.GetSize() == 3);
  REQUIRE(lqt.Contains(&objects[0]));
  REQUIRE(lqt.Contains(&objects[1]));
  REQUIRE(lqt.Contains(&objects[2]));
  REQUIRE(lqt.GetLooseBoundingBox().Intersects(objects[0]));
  REQUIRE(lqt.GetLooseBoundingBox().Intersects(objects[1]));
  REQUIRE(lqt.GetLooseBoundingBox().Intersects(objects[2]));
  REQUIRE_FALSE(objects[0].Contains(lqt.GetLooseBoundingBox()));
  REQUIRE_FALSE(objects[1].Contains(lqt.GetLooseBoundingBox()));
  REQUIRE_FALSE(objects[2].Contains(lqt.GetLooseBoundingBox()));
  lqt.Remove(&objects[0]);
  lqt.Remove(&objects[2]);
  REQUIRE_FALSE(lqt.Contains(&objects[0]));
  REQUIRE(lqt.Contains(&objects[1]));
  REQUIRE(lqt.GetSize() == 1);

  if (reclaim_losses) {
    lqt.ForceCleanup();
  }

  lqt.Remove(&objects[1]);
  REQUIRE(lqt.GetSize() == 0);

  if (reclaim_losses) {
    lqt.ForceCleanup();
  }
}

TEMPLATE_TEST_CASE("TestQueryIntersects", "", TYPES_FOR_TESTING) {
  std::vector<loose_quadtree::BoundingBox<TestType>> objects;
  objects.push_back({10000, 10000, 8000, 8000});//0
  objects.push_back({10000, 10000, 7000, 6000});//1
  objects.push_back({10000, 10000, 7, 6});//2
  objects.push_back({15000, 15000, 500, 600});//3
  objects.push_back({15100, 15100, 200, 200});//4
  objects.push_back({15000, 15000, 200, 200});//5
  objects.push_back({15100, 15100, 2, 2});//6
  loose_quadtree::LooseQuadtree<TestType, loose_quadtree::BoundingBox<TestType>, TrivialBBExtractor<TestType>> lqt;
  for (auto& obj: objects) {
    lqt.Insert(&obj);
  }

  auto query = lqt.QueryIntersectsRegion(loose_quadtree::BoundingBox<TestType>(33, 33, 1, 1));
  REQUIRE(query.EndOfQuery());

  query = lqt.QueryIntersectsRegion(loose_quadtree::BoundingBox<TestType>(9000, 9000, 9000, 9000));
  int count = 0;
  while (!query.EndOfQuery()) {
    loose_quadtree::BoundingBox<TestType>* obj = query.GetCurrent();
    (void) obj;
    count++;
    query.Next();
  }
  REQUIRE(count == 7);

  query = lqt.QueryIntersectsRegion(loose_quadtree::BoundingBox<TestType>(10003, 10003, 3, 7));
  count = 0;
  while (!query.EndOfQuery()) {
    loose_quadtree::BoundingBox<TestType>* obj = query.GetCurrent();
    REQUIRE_FALSE((obj != &objects[0] && obj != &objects[1] && obj != &objects[2]));
    count++;
    query.Next();
  }
  REQUIRE(count == 3);

  query = lqt.QueryIntersectsRegion(loose_quadtree::BoundingBox<TestType>(14900, 14900, 200, 200));
  count = 0;
  while (!query.EndOfQuery()) {
    loose_quadtree::BoundingBox<TestType>* obj = query.GetCurrent();
    REQUIRE_FALSE((obj != &objects[0] && obj != &objects[1] && obj != &objects[3] && obj != &objects[5]));
    count++;
    query.Next();
  }
  REQUIRE(count == 4);
}

TEMPLATE_TEST_CASE("TestQueryInside", "", TYPES_FOR_TESTING) {
  std::vector<loose_quadtree::BoundingBox<TestType>> objects;
  objects.push_back({10000, 10000, 8000, 8000});//0
  objects.push_back({10000, 10000, 7000, 6000});//1
  objects.push_back({10000, 10000, 7, 6});//2
  objects.push_back({15000, 15000, 500, 600});//3
  objects.push_back({15100, 15100, 200, 200});//4
  objects.push_back({15000, 15000, 200, 200});//5
  objects.push_back({15100, 15100, 2, 2});//6
  loose_quadtree::LooseQuadtree<TestType, loose_quadtree::BoundingBox<TestType>, TrivialBBExtractor<TestType>> lqt;
  for (auto& obj: objects) {
    lqt.Insert(&obj);
  }

  auto query = lqt.QueryInsideRegion(loose_quadtree::BoundingBox<TestType>(33, 33, 1, 1));
  REQUIRE(query.EndOfQuery());

  query = lqt.QueryInsideRegion(loose_quadtree::BoundingBox<TestType>(9000, 9000, 9000, 9000));
  int count = 0;
  while (!query.EndOfQuery()) {
    loose_quadtree::BoundingBox<TestType>* obj = query.GetCurrent();
    (void) obj;
    count++;
    query.Next();
  }
  REQUIRE(count == 7);

  query = lqt.QueryInsideRegion(loose_quadtree::BoundingBox<TestType>(10003, 10003, 3, 7));
  REQUIRE(query.EndOfQuery());

  query = lqt.QueryInsideRegion(loose_quadtree::BoundingBox<TestType>(14900, 14900, 300, 300));
  count = 0;
  while (!query.EndOfQuery()) {
    loose_quadtree::BoundingBox<TestType>* obj = query.GetCurrent();
    REQUIRE_FALSE((obj != &objects[5] && obj != &objects[6]));
    count++;
    query.Next();
  }
  REQUIRE(count == 2);
}

TEMPLATE_TEST_CASE("TestQueryContains", "", TYPES_FOR_TESTING) {
  std::vector<loose_quadtree::BoundingBox<TestType>> objects;
  objects.push_back({10000, 10000, 8000, 8000});//0
  objects.push_back({10000, 10000, 7000, 6000});//1
  objects.push_back({10000, 10000, 7, 6});//2
  objects.push_back({15000, 15000, 500, 600});//3
  objects.push_back({15100, 15100, 200, 200});//4
  objects.push_back({15000, 15000, 200, 200});//5
  objects.push_back({15100, 15100, 2, 2});//6
  loose_quadtree::LooseQuadtree<TestType, loose_quadtree::BoundingBox<TestType>, TrivialBBExtractor<TestType>> lqt;
  for (auto& obj: objects) {
    lqt.Insert(&obj);
  }

  auto query = lqt.QueryContainsRegion(loose_quadtree::BoundingBox<TestType>(33, 33, 1, 1));
  REQUIRE(query.EndOfQuery());

  query = lqt.QueryContainsRegion(loose_quadtree::BoundingBox<TestType>(9000, 9000, 9000, 9000));
  REQUIRE(query.EndOfQuery());

  query = lqt.QueryContainsRegion(loose_quadtree::BoundingBox<TestType>(10003, 10003, 3, 7));
  int count = 0;
  while (!query.EndOfQuery()) {
    loose_quadtree::BoundingBox<TestType>* obj = query.GetCurrent();
    REQUIRE_FALSE((obj != &objects[0] && obj != &objects[1]));
    count++;
    query.Next();
  }
  REQUIRE(count == 2);

  query = lqt.QueryContainsRegion(loose_quadtree::BoundingBox<TestType>(14900, 14900, 200, 200));
  count = 0;
  while (!query.EndOfQuery()) {
    loose_quadtree::BoundingBox<TestType>* obj = query.GetCurrent();
    REQUIRE_FALSE((obj != &objects[0] && obj != &objects[1]));
    count++;
    query.Next();
  }
  REQUIRE(count == 2);

  query = lqt.QueryContainsRegion(loose_quadtree::BoundingBox<TestType>(15000, 15000, 2, 2));
  count = 0;
  while (!query.EndOfQuery()) {
    loose_quadtree::BoundingBox<TestType>* obj = query.GetCurrent();
    REQUIRE_FALSE((obj != &objects[0] && obj != &objects[1] && obj != &objects[3] && obj != &objects[5]));
    count++;
    query.Next();
  }
  REQUIRE(count == 4);
}

/*
// maybe banchmarck?
TEMPLATE_TEST_CASE("StressTest", "", TYPES_FOR_TESTING) {
  return; // disable
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

  std::vector<loose_quadtree::BoundingBox<TestType>> objects;
  objects.reserve(objects_generated);
  std::vector<bool> flags(objects_generated, false);
  loose_quadtree::LooseQuadtree<TestType, loose_quadtree::BoundingBox<TestType>, TrivialBBExtractor<TestType>> lqt;
  for (std::size_t i = 0; i < objects_generated; i++) {
    objects.emplace_back((TestType)coordinate(rand), (TestType)coordinate(rand),
                         (TestType)distance(rand), (TestType)distance(rand));
    lqt.Insert(&objects[i]);
  }
  REQUIRE(objects.size() == objects_generated);
  REQUIRE(flags.size() == objects_generated);

  for (int round = 0; round < full_rounds; round++) {
    for (int fluctobj = 0; fluctobj < object_fluctuation; fluctobj++) {
      std::size_t id = index(rand);
      objects[id] = loose_quadtree::BoundingBox<TestType>((TestType)coordinate(rand), (TestType)coordinate(rand),
                                         (TestType)distance(rand), (TestType)distance(rand));
      lqt.Update(&objects[id]);
    }
    for (int query_round = 0; query_round < query_rounds; query_round++) {
      loose_quadtree::BoundingBox<TestType> query_region((TestType)coordinate(rand), (TestType)coordinate(rand),
                                        (TestType)distance(rand), (TestType)distance(rand));
      for (std::size_t i = 0; i < objects_generated; i++) {
        flags[i] = false;
      }

      auto query = lqt.QueryIntersectsRegion(query_region);
      while (!query.EndOfQuery()) {
        loose_quadtree::BoundingBox<TestType>* obj = query.GetCurrent();
        REQUIRE(query_region.Intersects(*obj));
        auto id = (std::size_t)(obj - &objects[0]);
        REQUIRE((id >= 0 && id < objects_generated));
        flags[id] = true;
        query.Next();
      }
      for (std::size_t i = 0; i < objects_generated; i++) {
        REQUIRE(flags[i] == query_region.Intersects(objects[i]));
        flags[i] = false;
      }

      query = lqt.QueryInsideRegion(query_region);
      while (!query.EndOfQuery()) {
        loose_quadtree::BoundingBox<TestType>* obj = query.GetCurrent();
        REQUIRE(query_region.Contains(*obj));
        std::size_t id = (std::size_t)(obj - &objects[0]);
        REQUIRE((id >= 0 && id < objects_generated));
        flags[id] = true;
        query.Next();
      }
      for (std::size_t i = 0; i < objects_generated; i++) {
        REQUIRE(flags[i] == query_region.Contains(objects[i]));
        flags[i] = false;
      }

      query = lqt.QueryContainsRegion(query_region);
      while (!query.EndOfQuery()) {
        loose_quadtree::BoundingBox<TestType>* obj = query.GetCurrent();
        REQUIRE(obj->Contains(query_region));
        std::size_t id = (std::size_t)(obj - &objects[0]);
        REQUIRE((id >= 0 && id < objects_generated));
        flags[id] = true;
        query.Next();
      }
      for (std::size_t i = 0; i < objects_generated; i++) {
        REQUIRE(flags[i] == objects[i].Contains(query_region));
        //flags[i] = false;
      }
    }
  }
  lqt.ForceCleanup();
}*/
