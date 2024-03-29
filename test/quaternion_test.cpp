#include "gtest/gtest.h"
#include "quaternion.hpp"

#include <cmath>

TEST(quat_test_suite, invalid_init){
    EXPECT_THROW({
        try
        {
            quaternion::Quaternion a(0.0, 0.0, 0.0, 0.0);
        }
        catch(const std::invalid_argument& e)
        {
            EXPECT_STREQ("Cannot create quaternion: Magnitude of quaternion cannot be zero.", e.what() );
            throw;
        }
    }, std::invalid_argument);

    EXPECT_THROW({
        try
        {
            quaternion::Quaternion a(0.00000001, 0.0, 0.0, 0.0);
        }
        catch(const std::invalid_argument& e)
        {
            EXPECT_STREQ("Cannot create quaternion: Magnitude of quaternion cannot be zero.", e.what() );
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

TEST(quat_test_suite, norm_d){
    quaternion::Quaternion a(-19.0, -2.1, 3.3, 6.9);

    a.normalize();

    auto norm = std::sqrt(
        std::pow(a.w(), 2.0) + std::pow(a.x(), 2.0)
        + std::pow(a.y(), 2.0) + std::pow(a.z(), 2.0));

    EXPECT_DOUBLE_EQ(1.0, norm);
    EXPECT_DOUBLE_EQ(-0.92281951518771288, a.w());
    EXPECT_DOUBLE_EQ(-0.10199584115232617, a.x());
    EXPECT_DOUBLE_EQ(0.16027917895365537, a.y());
    EXPECT_DOUBLE_EQ(0.3351291923576431, a.z());
}

TEST(quat_test_suite, norm_f){
    quaternion::Quaternion a(-19.0f, -2.1f, 3.3f, 6.9f);

    a.normalize();

    auto norm = std::sqrt(
        std::pow(a.w(), 2.0f) + std::pow(a.x(), 2.0f)
        + std::pow(a.y(), 2.0f) + std::pow(a.z(), 2.0f));

    EXPECT_FLOAT_EQ(1.0f, norm);
    EXPECT_FLOAT_EQ(-0.92281951f, a.w());
    EXPECT_FLOAT_EQ(-0.10199585f, a.x());
    EXPECT_FLOAT_EQ(0.16027917f, a.y());
    EXPECT_FLOAT_EQ(0.3351292f, a.z());
}

TEST(quat_test_suite, scalar_f){
    quaternion::Quaternion<float> a(2.0f, -2.0f, 3.0f, 6.0f);

    a = a * 0.5;

    EXPECT_FLOAT_EQ(1.0f, a.w());
    EXPECT_FLOAT_EQ(-1.0f, a.x());
    EXPECT_FLOAT_EQ(1.5f, a.y());
    EXPECT_FLOAT_EQ(3.0f, a.z());
}

TEST(quat_test_suite, add_f){
    quaternion::Quaternion<float> a(2.0f, -2.0f, 3.0f, 6.0f);

    a = a + a;

    EXPECT_FLOAT_EQ(4.0f, a.w());
    EXPECT_FLOAT_EQ(-4.0f, a.x());
    EXPECT_FLOAT_EQ(6.0f, a.y());
    EXPECT_FLOAT_EQ(12.0f, a.z());
}

TEST(quat_test_suite, subtract_f){
    quaternion::Quaternion<float> a(2.0f, -2.0f, 3.0f, 6.0f);

    a = a - a*0.5;

    EXPECT_FLOAT_EQ(1.0f, a.w());
    EXPECT_FLOAT_EQ(-1.0f, a.x());
    EXPECT_FLOAT_EQ(1.5f, a.y());
    EXPECT_FLOAT_EQ(3.0f, a.z());
}

TEST(quat_test_suite, subtract_equals_f){
    quaternion::Quaternion<float> a(2.0f, -2.0f, 3.0f, 6.0f);

    a -= a*0.5;

    EXPECT_FLOAT_EQ(1.0f, a.w());
    EXPECT_FLOAT_EQ(-1.0f, a.x());
    EXPECT_FLOAT_EQ(1.5f, a.y());
    EXPECT_FLOAT_EQ(3.0f, a.z());
}

TEST(quat_test_suite, set_f){
    quaternion::Quaternion<float> a(2.0f, -2.0f, 3.0f, 6.0f);
    a.set(-100.0f, -101.0f, -102.0f, -103.0f);

    EXPECT_FLOAT_EQ(-100.0f, a.w());
    EXPECT_FLOAT_EQ(-101.0f, a.x());
    EXPECT_FLOAT_EQ(-102.0f, a.y());
    EXPECT_FLOAT_EQ(-103.0f, a.z());
}

TEST(quat_test_suite, to_array){
    quaternion::Quaternion<float> a(2.0f, -2.0f, 3.0f, 6.0f);
    auto out = a.to_array();

    EXPECT_FLOAT_EQ(out[0], a.w());
    EXPECT_FLOAT_EQ(out[1], a.x());
    EXPECT_FLOAT_EQ(out[2], a.y());
    EXPECT_FLOAT_EQ(out[3], a.z());
}
