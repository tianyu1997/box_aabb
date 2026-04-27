#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "sbf/util/math_util.h"
#include "sbf/util/timer.h"

TEST(P0Smoke, ClampWorks) {
    EXPECT_DOUBLE_EQ(sbf::util::clamp(5.0, 0.0, 3.0), 3.0);
    EXPECT_DOUBLE_EQ(sbf::util::clamp(-1.0, 0.0, 3.0), 0.0);
    EXPECT_DOUBLE_EQ(sbf::util::clamp(2.0, 0.0, 3.0), 2.0);
}

TEST(P0Smoke, FaceOverlapDetected) {
    Eigen::AlignedBox3d a(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
    Eigen::AlignedBox3d b(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(2, 1, 1));
    EXPECT_TRUE(sbf::util::has_face_overlap(a, b));
    EXPECT_DOUBLE_EQ(sbf::util::face_overlap_area(a, b, 0), 1.0);
}

TEST(P0Smoke, NoFaceOverlapWhenDiagonal) {
    Eigen::AlignedBox3d a(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
    Eigen::AlignedBox3d b(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(2, 2, 2));
    EXPECT_FALSE(sbf::util::has_face_overlap(a, b));
}

TEST(P0Smoke, NoFaceOverlapWhenSeparated) {
    Eigen::AlignedBox3d a(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
    Eigen::AlignedBox3d b(Eigen::Vector3d(2, 0, 0), Eigen::Vector3d(3, 1, 1));
    EXPECT_FALSE(sbf::util::has_face_overlap(a, b));
}

TEST(P0Smoke, TimerWorks) {
    sbf::util::Timer t;
    t.start();
    volatile long x = 0;
    for (int i = 0; i < 1000000; ++i) x += i;
    EXPECT_GT(t.elapsed_ms(), 0.0);
    EXPECT_GE(t.elapsed_s(), 0.0);
}
