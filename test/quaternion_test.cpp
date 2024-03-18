#include "gtest/gtest.h"
#include "quaternion.hpp"

TEST(quat_test_suite, invalid_quat) {

    EXPECT_THROW({
        try
        {
            quaternion::Quaternion a(0.0, 0.0, 0.0, 0.0);
        }
        catch( const std::invalid_argument& e )
        {
            // and this tests that it has the correct message
            EXPECT_STREQ( "Magnitude of quaternion cannot be zero.", e.what() );
            throw;
        }
    }, std::invalid_argument);
}

TEST(quat_test_suite, conjugate_f) {
    quaternion::Quaternion a(1.0f, 2.1f, 3.3f, 6.9f);
    quaternion::Quaternion c = a.conjugate();

    EXPECT_FLOAT_EQ(1.0f, c.w());
    EXPECT_FLOAT_EQ(-2.1f, c.x());
    EXPECT_FLOAT_EQ(-3.3f, c.y());
    EXPECT_FLOAT_EQ(-6.9f, c.z());
}

TEST(quat_test_suite, conjugate_d) {
    quaternion::Quaternion a(1.0, 2.1, 3.3, 6.9);
    quaternion::Quaternion c = a.conjugate();

    EXPECT_FLOAT_EQ(1.0, c.w());
    EXPECT_FLOAT_EQ(-2.1, c.x());
    EXPECT_FLOAT_EQ(-3.3, c.y());
    EXPECT_FLOAT_EQ(-6.9, c.z());
}
