#include "gtest/gtest.h"
#include "quaternion.hpp"

TEST(quat_test_suite, invalid_init){
    EXPECT_THROW({
        try
        {
            quaternion::Quaternion a(0.0, 0.0, 0.0, 0.0);
        }
        catch(const std::invalid_argument& e)
        {
            EXPECT_STREQ("Magnitude of quaternion cannot be zero.", e.what() );
            throw;
        }
    }, std::invalid_argument);
}

TEST(quat_test_suite, conjugate_f){
    quaternion::Quaternion a(1.0f, 2.1f, 3.3f, 6.9f);
    quaternion::Quaternion c = a.conjugate();

    EXPECT_FLOAT_EQ(1.0f, c.w());
    EXPECT_FLOAT_EQ(-2.1f, c.x());
    EXPECT_FLOAT_EQ(-3.3f, c.y());
    EXPECT_FLOAT_EQ(-6.9f, c.z());
}

TEST(quat_test_suite, conjugate_d){
    quaternion::Quaternion a(1.0, 2.1, 3.3, 6.9);
    quaternion::Quaternion c = a.conjugate();

    EXPECT_DOUBLE_EQ(1.0, c.w());
    EXPECT_DOUBLE_EQ(-2.1, c.x());
    EXPECT_DOUBLE_EQ(-3.3, c.y());
    EXPECT_DOUBLE_EQ(-6.9, c.z());
}

TEST(quat_test_suite, product){
    quaternion::Quaternion a(19.0, 2.1, 3.3, 6.9);
    quaternion::Quaternion b(19.0, -2.1, -3.3, -6.9);
    quaternion::Quaternion c = a * b;

    EXPECT_DOUBLE_EQ(423.91, c.w());
    EXPECT_DOUBLE_EQ(0.0, c.x());
    EXPECT_DOUBLE_EQ(0.0, c.y());
    EXPECT_DOUBLE_EQ(0.0, c.z());
}

TEST(quat_test_suite, norm){
    quaternion::Quaternion a(-19.0, -2.1, 3.3, 6.9);

    a.normalize();

    auto norm = std::sqrt(
            std::pow(a.w(), 2.0) + std::pow(a.x(), 2.0) +
            std::pow(a.y(), 2.0) + std::pow(a.z(), 2.0));

    EXPECT_DOUBLE_EQ(1.0, norm);
    // EXPECT_DOUBLE_EQ(-0.92281951, a.w());
    // EXPECT_DOUBLE_EQ(-0.1019958, a.x());
    // EXPECT_DOUBLE_EQ(0.16027, a.y());
    // EXPECT_DOUBLE_EQ(0.3351, a.z());

}
