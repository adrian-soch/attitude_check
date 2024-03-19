#include "gtest/gtest.h"
#include "initializers.hpp"

TEST(initializers_test_suite, euler_d){
    Eigen::Vector3d acc {0.0, 0.0, 9.81};

    quaternion::Quaternion out = init::acc_to_quat(acc);
    out.normalize();

    EXPECT_DOUBLE_EQ(out.w(), 1.0);
    EXPECT_DOUBLE_EQ(out.x(), 0.0);
    EXPECT_DOUBLE_EQ(out.y(), 0.0);
    EXPECT_DOUBLE_EQ(out.z(), 0.0);
}

TEST(initializers_test_suite, euler_f){
    Eigen::Vector3f acc {0.0f, 0.0f, 9.81f};

    quaternion::Quaternion out = init::acc_to_quat(acc);
    out.normalize();

    EXPECT_DOUBLE_EQ(out.w(), 1.0f);
    EXPECT_DOUBLE_EQ(out.x(), 0.0f);
    EXPECT_DOUBLE_EQ(out.y(), 0.0f);
    EXPECT_DOUBLE_EQ(out.z(), 0.0f);
}
