#include "gtest/gtest.h"
#include "utilities.hpp"

TEST(utilities_test_suite, euler){
    quaternion::Quaternion out = utils::euler_to_quat(0.0, 0.0, 0.0);
    out.normalize();

    EXPECT_DOUBLE_EQ(out.w(), 1.0);
    EXPECT_DOUBLE_EQ(out.x(), 0.0);
    EXPECT_DOUBLE_EQ(out.y(), 0.0);
    EXPECT_DOUBLE_EQ(out.z(), 0.0);
}

TEST(utilities_test_suite, rotm){
    Eigen::Matrix3d in;
    in << 1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0;

    quaternion::Quaternion out = utils::rotm_to_quat(in);
    out.normalize();

    EXPECT_DOUBLE_EQ(out.w(), 1.0);
    EXPECT_DOUBLE_EQ(out.x(), 0.0);
    EXPECT_DOUBLE_EQ(out.y(), 0.0);
    EXPECT_DOUBLE_EQ(out.z(), 0.0);
}
