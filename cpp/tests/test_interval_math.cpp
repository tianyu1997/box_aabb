// Tests for interval arithmetic operations
#include <gtest/gtest.h>
#include "sbf/core/types.h"
#include "sbf/aabb/interval_math.h"
#include <cmath>

using namespace sbf;

TEST(IntervalTest, BasicArithmetic) {
    Interval a{1.0, 3.0};
    Interval b{2.0, 4.0};

    auto sum = a + b;
    EXPECT_DOUBLE_EQ(sum.lo, 3.0);
    EXPECT_DOUBLE_EQ(sum.hi, 7.0);

    auto diff = a - b;
    EXPECT_DOUBLE_EQ(diff.lo, -3.0);
    EXPECT_DOUBLE_EQ(diff.hi, 1.0);

    auto prod = a * b;
    EXPECT_DOUBLE_EQ(prod.lo, 2.0);
    EXPECT_DOUBLE_EQ(prod.hi, 12.0);
}

TEST(IntervalTest, MultiplyNegative) {
    Interval a{-2.0, 1.0};
    Interval b{-1.0, 3.0};
    auto prod = a * b;
    EXPECT_DOUBLE_EQ(prod.lo, -6.0);
    EXPECT_DOUBLE_EQ(prod.hi, 3.0);
}

TEST(IntervalTest, Width) {
    Interval a{1.0, 5.0};
    EXPECT_DOUBLE_EQ(a.width(), 4.0);
    EXPECT_DOUBLE_EQ(a.mid(), 3.0);
}

TEST(IntervalTest, ISinMonotone) {
    // sin on [0, pi/4] — monotone increasing
    auto s = I_sin(0.0, M_PI / 4.0);
    EXPECT_NEAR(s.lo, 0.0, 1e-12);
    EXPECT_NEAR(s.hi, std::sin(M_PI / 4.0), 1e-12);
}

TEST(IntervalTest, ISinCrossesMax) {
    // sin on [0, pi] — crosses pi/2, so max = 1
    auto s = I_sin(0.0, M_PI);
    EXPECT_NEAR(s.lo, 0.0, 1e-10);
    EXPECT_NEAR(s.hi, 1.0, 1e-12);
}

TEST(IntervalTest, ICosMonotone) {
    // cos on [0, pi/4] — monotone decreasing
    auto c = I_cos(0.0, M_PI / 4.0);
    EXPECT_NEAR(c.lo, std::cos(M_PI / 4.0), 1e-12);
    EXPECT_NEAR(c.hi, 1.0, 1e-12);
}

TEST(IntervalTest, ICosFullRange) {
    // cos on [-pi, pi]
    auto c = I_cos(-M_PI, M_PI);
    EXPECT_NEAR(c.lo, -1.0, 1e-12);
    EXPECT_NEAR(c.hi, 1.0, 1e-12);
}

TEST(IntervalTest, BuildDHJoint) {
    // Simple test: theta=[0,0], d=0, a=1, alpha=0
    // build_dh_joint(alpha, a, ct_lo, ct_hi, st_lo, st_hi, d_lo, d_hi, A_lo, A_hi)
    double A_lo[16], A_hi[16];
    build_dh_joint(0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, A_lo, A_hi);
    // cos(0)=1, sin(0)=0 → identity-like with a=1 translation in x
    EXPECT_NEAR(A_lo[0], 1.0, 1e-12);    // cos(0)
    EXPECT_NEAR(A_hi[0], 1.0, 1e-12);
}
