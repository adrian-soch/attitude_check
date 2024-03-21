#include "gtest/gtest.h"
#include "initializers.hpp"

const float MAX_ABS_ERROR { 0.002 }; // About 0.115 of a degree

/**
 * @brief Test acc_to_quat
 *
 */
TEST(initializers_test_suite, acc_d){
    Eigen::Vector3d acc { 0.090941, -0.031273, 9.759028 };
    acc.normalize();

    quaternion::Quaternion out = init::acc_to_quat(acc);
    out.normalize();

    EXPECT_NEAR(out.w(), 0.999987863, MAX_ABS_ERROR);
    EXPECT_NEAR(out.x(), -0.001602236, MAX_ABS_ERROR);
    EXPECT_NEAR(out.y(), -0.004659145, MAX_ABS_ERROR);
    EXPECT_NEAR(out.z(), -0.000007465, MAX_ABS_ERROR);
}

TEST(initializers_test_suite, acc1_f){
    Eigen::Vector3d acc { 0.090941f, -0.031273f, 9.759028f };
    acc.normalize();

    quaternion::Quaternion out = init::acc_to_quat(acc);
    out.normalize();

    EXPECT_NEAR(out.w(), 0.999987f, MAX_ABS_ERROR);
    EXPECT_NEAR(out.x(), -0.0016022f, MAX_ABS_ERROR);
    EXPECT_NEAR(out.y(), -0.0046f, MAX_ABS_ERROR);
    EXPECT_NEAR(out.z(), 0.0f, MAX_ABS_ERROR);
}

TEST(initializers_test_suite, acc2_f){
    Eigen::Vector3d acc { -9.81f, 0.0f, 0.0f };
    acc.normalize();

    quaternion::Quaternion out = init::acc_to_quat(acc);
    out.normalize();

    EXPECT_NEAR(out.w(), 0.7071f, MAX_ABS_ERROR);
    EXPECT_NEAR(out.x(), 0.0f, MAX_ABS_ERROR);
    EXPECT_NEAR(out.y(), 0.7071f, MAX_ABS_ERROR);
    EXPECT_NEAR(out.z(), 0.0f, MAX_ABS_ERROR);
}

TEST(initializers_test_suite, mag_d){
    Eigen::Vector3d acc { 0.0, 0.0, 9.81 };
    acc.normalize();

    Eigen::Vector3d mag { 16676.8, -3050.9, 49916.9 };
    mag.normalize();

    quaternion::Quaternion out = init::mag_to_quat(acc, mag);
    out.normalize();

    EXPECT_NEAR(out.w(), 0.99591029, MAX_ABS_ERROR);
    EXPECT_NEAR(out.x(), 0.0, MAX_ABS_ERROR);
    EXPECT_NEAR(out.y(), 0.0, MAX_ABS_ERROR);
    EXPECT_NEAR(out.z(), 0.09034758, MAX_ABS_ERROR);
}

TEST(initializers_test_suite, mag1_f){
    Eigen::Vector3f acc { 0.0f, 0.0f, 9.81f };
    acc.normalize();

    Eigen::Vector3f mag { 16676.8f, -3050.9f, 49916.9f };
    mag.normalize();

    quaternion::Quaternion out = init::mag_to_quat(acc, mag);
    out.normalize();

    EXPECT_NEAR(out.w(), 0.99591029f, MAX_ABS_ERROR);
    EXPECT_NEAR(out.x(), 0.0f, MAX_ABS_ERROR);
    EXPECT_NEAR(out.y(), 0.0f, MAX_ABS_ERROR);
    EXPECT_NEAR(out.z(), 0.09034758f, MAX_ABS_ERROR);
}

// TEST(initializers_test_suite, mag2_f){
//     Eigen::Vector3f acc { -6.382152f, -7.969839f, 0.099505f };
//     acc.normalize();

//     Eigen::Vector3f mag { 36.700461f, 22.613240f, 8.881961f };
//     mag.normalize();

//     quaternion::Quaternion out = init::mag_to_quat(acc, mag);
//     out.normalize();

//     EXPECT_NEAR(out.w(), 0.70953f, MAX_ABS_ERROR);
//     EXPECT_NEAR(out.x(), -0.57186f, MAX_ABS_ERROR);
//     EXPECT_NEAR(out.y(), 0.409993f, MAX_ABS_ERROR);
//     EXPECT_NEAR(out.z(), 0.037796f, MAX_ABS_ERROR);

// }
