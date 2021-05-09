#include "MathUtil.hpp"

#include <gtest/gtest.h>

TEST(MathUtilTest, BasicAssertions)
{
  EXPECT_STRNE("hello", "world");
}
