#include "gtest/gtest.h"
#include "initializers.hpp"

TEST(initializers_test_suite, acc_d){
    Eigen::Vector3d acc {0.0, 0.0, 9.81};
    acc.normalize();

    quaternion::Quaternion out = init::acc_to_quat(acc);

    std::cout << out.w() << " " << out.x() << std::endl;
    out.normalize();

    EXPECT_DOUBLE_EQ(out.w(), 1.0);
    EXPECT_DOUBLE_EQ(out.x(), 0.0);
    EXPECT_DOUBLE_EQ(out.y(), 0.0);
    EXPECT_DOUBLE_EQ(out.z(), 0.0);
}

TEST(initializers_test_suite, acc_f){
    Eigen::Vector3f acc {0.0f, 0.0f, 9.81f};
    acc.normalize();

    quaternion::Quaternion out = init::acc_to_quat(acc);
    out.normalize();

    EXPECT_DOUBLE_EQ(out.w(), 1.0f);
    EXPECT_DOUBLE_EQ(out.x(), 0.0f);
    EXPECT_DOUBLE_EQ(out.y(), 0.0f);
    EXPECT_DOUBLE_EQ(out.z(), 0.0f);
}

TEST(initializers_test_suite, mag_d){
    Eigen::Vector3d acc {0.0, 0.0, 9.81};
    acc.normalize();

    Eigen::Vector3d mag {16676.8, -3050.9, 49916.9};
    mag.normalize();

    quaternion::Quaternion out = init::mag_to_quat(acc, mag);

    std::cout << out.w() << " " << out.x() << std::endl;
    out.normalize();

    // EXPECT_DOUBLE_EQ(out.w(), 1.0);
    // EXPECT_DOUBLE_EQ(out.x(), 0.0);
    // EXPECT_DOUBLE_EQ(out.y(), 0.0);
    // EXPECT_DOUBLE_EQ(out.z(), 0.0);
}

TEST(initializers_test_suite, mag_f){
    Eigen::Vector3f acc {0.0f, 0.0f, 9.81f};
    acc.normalize();

    Eigen::Vector3f mag {16676.8f, -3050.9f, 49916.9f};
    mag.normalize();

    quaternion::Quaternion out = init::acc_to_quat(acc);
    out.normalize();

    EXPECT_DOUBLE_EQ(out.w(), 1.0f);
    EXPECT_DOUBLE_EQ(out.x(), 0.0f);
    EXPECT_DOUBLE_EQ(out.y(), 0.0f);
    EXPECT_DOUBLE_EQ(out.z(), 0.0f);
}
