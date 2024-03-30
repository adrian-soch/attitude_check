#include "gtest/gtest.h"
#include "utilities.hpp"

const float MAX_ABS_ERROR { 0.002 }; // About 0.115 of a degree

TEST(utilities_test_suite, euler_d){
    quaternion::Quaternion out = utils::euler_to_quat(0.0, 0.0, 0.0);

    out.normalize();

    EXPECT_DOUBLE_EQ(out.w(), 1.0);
    EXPECT_DOUBLE_EQ(out.x(), 0.0);
    EXPECT_DOUBLE_EQ(out.y(), 0.0);
    EXPECT_DOUBLE_EQ(out.z(), 0.0);
}

TEST(utilities_test_suite, euler_f){
    quaternion::Quaternion out = utils::euler_to_quat(0.0f, 0.0f, 0.0f);

    out.normalize();

    EXPECT_FLOAT_EQ(out.w(), 1.0f);
    EXPECT_FLOAT_EQ(out.x(), 0.0f);
    EXPECT_FLOAT_EQ(out.y(), 0.0f);
    EXPECT_FLOAT_EQ(out.z(), 0.0f);
}

TEST(utilities_test_suite, euler1_f){
    quaternion::Quaternion out = utils::euler_to_quat(1.55f, 0.06f, -2.59f);

    out.normalize();

    EXPECT_NEAR(out.w(), 0.1742636f, MAX_ABS_ERROR);
    EXPECT_NEAR(out.x(), 0.2110758f, MAX_ABS_ERROR);
    EXPECT_NEAR(out.y(), -0.6671344f, MAX_ABS_ERROR);
    EXPECT_NEAR(out.z(), -0.6928282f, MAX_ABS_ERROR);
}

TEST(utilities_test_suite, euler2_f){
    quaternion::Quaternion out = utils::euler_to_quat(-20.0f, -0.005f, 3.140f);

    out.normalize();

    EXPECT_NEAR(out.w(), -0.0020282f, MAX_ABS_ERROR);
    EXPECT_NEAR(out.x(), -0.0016645f, MAX_ABS_ERROR);
    EXPECT_NEAR(out.y(), 0.5440209f, MAX_ABS_ERROR);
    EXPECT_NEAR(out.z(), -0.8390676f, MAX_ABS_ERROR);
}

TEST(utilities_test_suite, rotm_d){
    std::array<std::array<double, 3>, 3> in { {
        { { 1.0, 0.0, 0.0 } },
        { { 0.0, 1.0, 0.0 } },
        { { 0.0, 0.0, 1.0 } }
    } };

    quaternion::Quaternion out = utils::rotm_to_quat(in);

    out.normalize();

    EXPECT_DOUBLE_EQ(out.w(), 1.0);
    EXPECT_DOUBLE_EQ(out.x(), 0.0);
    EXPECT_DOUBLE_EQ(out.y(), 0.0);
    EXPECT_DOUBLE_EQ(out.z(), 0.0);
}

TEST(utilities_test_suite, rotm_f){
    std::array<std::array<double, 3>, 3> in { {
        { { 1.0, 0.0, 0.0 } },
        { { 0.0, 1.0, 0.0 } },
        { { 0.0, 0.0, 1.0 } }
    } };

    quaternion::Quaternion out = utils::rotm_to_quat(in);

    out.normalize();

    EXPECT_FLOAT_EQ(out.w(), 1.0f);
    EXPECT_FLOAT_EQ(out.x(), 0.0f);
    EXPECT_FLOAT_EQ(out.y(), 0.0f);
    EXPECT_FLOAT_EQ(out.z(), 0.0f);
}
