// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <gtest/gtest.h>
#include "MergeMethods.h"

using namespace ow_dynamic_terrain;

TEST(TestMergeMethods, invokeDirectly)
{
  // Maybe generate them at random (later)
  const float n1 = 2.0f;
  const float n2 = 3.0f;

  EXPECT_FLOAT_EQ(n1, MergeMethods::keep(n1, n2));
  EXPECT_FLOAT_EQ(n2, MergeMethods::replace(n1, n2));
  EXPECT_FLOAT_EQ(n1 + n2, MergeMethods::add(n1, n2));
  EXPECT_FLOAT_EQ(n1 - n2, MergeMethods::sub(n1, n2));
  EXPECT_FLOAT_EQ(n1, MergeMethods::min(n1, n2));
  EXPECT_FLOAT_EQ(n2, MergeMethods::max(n1, n2));
  EXPECT_FLOAT_EQ(0.5f * (n1 + n2), MergeMethods::avg(n1, n2));
}

TEST(TestMergeMethods, invokeIndirectly)
{
  const float n1 = 2.0f;
  const float n2 = 3.0f;
  
  auto merge_method = boost::optional<const MergeMethods::MergeMethod&>();
  ASSERT_FALSE(merge_method);

  merge_method = MergeMethods::mergeMethodFromString("keep");
  ASSERT_TRUE(merge_method);
  EXPECT_FLOAT_EQ(n1, (*merge_method)(n1, n2));

  merge_method = MergeMethods::mergeMethodFromString("replace");
  ASSERT_TRUE(merge_method);
  EXPECT_FLOAT_EQ(n2, (*merge_method)(n1, n2));

  merge_method = MergeMethods::mergeMethodFromString("add");
  ASSERT_TRUE(merge_method);
  EXPECT_FLOAT_EQ(n1 + n2, (*merge_method)(n1, n2));

  merge_method = MergeMethods::mergeMethodFromString("sub");
  ASSERT_TRUE(merge_method);
  EXPECT_FLOAT_EQ(n1 - n2, (*merge_method)(n1, n2));

  merge_method = MergeMethods::mergeMethodFromString("min");
  ASSERT_TRUE(merge_method);
  EXPECT_FLOAT_EQ(n1, (*merge_method)(n1, n2));

  merge_method = MergeMethods::mergeMethodFromString("max");
  ASSERT_TRUE(merge_method);
  EXPECT_FLOAT_EQ(n2, (*merge_method)(n1, n2));

  merge_method = MergeMethods::mergeMethodFromString("avg");
  ASSERT_TRUE(merge_method);
  EXPECT_FLOAT_EQ(0.5f * (n1 + n2), (*merge_method)(n1, n2));
}

TEST(TestMergeMethods, mergeMethodFromString_InvalidMethodName)
{
  ASSERT_FALSE(MergeMethods::mergeMethodFromString("invalid_method_name"));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
