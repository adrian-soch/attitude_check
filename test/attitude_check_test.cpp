#include <iostream>

#include "gtest/gtest.h"

#include "attitude_check.cpp"
#include "attitude_check.hpp"

const float MAX_ABS_ERROR { 0.002 }; // About 0.115 of a degree

namespace ac = attitude_check;

class Attitude_Check_Test_Suite : public ::testing::Test {
protected:
    float dt; // dt = 1/Hz
    Eigen::Vector3f acc { 0.0f, 0.0f, 9.81f };
    Eigen::Vector3f mag { 16676.8f, -3050.9f, 49916.9f };
    Eigen::Vector3f gyr { 1.0f, 1.0f, 1.0f };

    void SetUp() override
    {
        dt = 1.0 / 100.0;
        acc.normalize();
        mag.normalize();
        mag.normalize();
    }
};

TEST_F(Attitude_Check_Test_Suite, invalid_init){
    EXPECT_THROW({
        try
        {
            ac::AttitudeCheck ac(-1.0, 0.5);
        }
        catch(const std::invalid_argument& e)
        {
            EXPECT_STREQ("Gain must be within [0.0, 1.0].", e.what() );
            throw;
        }
    }, std::invalid_argument);

    EXPECT_THROW({
        try
        {
            ac::AttitudeCheck ac(1.0, 2.0);
        }
        catch(const std::invalid_argument& e)
        {
            EXPECT_STREQ("Gain must be within [0.0, 1.0].", e.what() );
            throw;
        }
    }, std::invalid_argument);
}

TEST_F(Attitude_Check_Test_Suite, zero_gyro){
    gyr.setZero();

    ac::AttitudeCheck ac(0.5f, 0.5f, -69.0f, -69.0f, -69.0f, -69.0f);
    quaternion::Quaternion<float> out = ac.update(acc, gyr, mag, dt);

    EXPECT_NEAR(out.w(), -69.0f, MAX_ABS_ERROR);
    EXPECT_NEAR(out.x(), -69.0f, MAX_ABS_ERROR);
    EXPECT_NEAR(out.y(), -69.0f, MAX_ABS_ERROR);
    EXPECT_NEAR(out.z(), -69.0f, MAX_ABS_ERROR);
}

TEST_F(Attitude_Check_Test_Suite, zero_mag){
    mag.setZero();
    acc.setZero();

    quaternion::Quaternion<float> expected(-69.0f, -69.0f, -69.0f, -69.0f);
    expected = expected * (quaternion::Quaternion<float>(0.0f, gyr[0], gyr[1], gyr[2]) * 0.5);
    expected = quaternion::Quaternion<float>(-69.0f, -69.0f, -69.0f, -69.0f) + (expected * dt);
    expected.normalize();

    ac::AttitudeCheck ac(0.5f, 0.5f, -69.0f, -69.0f, -69.0f, -69.0f);
    quaternion::Quaternion<float> out = ac.update(acc, gyr, mag, dt);

    EXPECT_NEAR(out.w(), expected.w(), MAX_ABS_ERROR);
    EXPECT_NEAR(out.x(), expected.x(), MAX_ABS_ERROR);
    EXPECT_NEAR(out.y(), expected.y(), MAX_ABS_ERROR);
    EXPECT_NEAR(out.z(), expected.z(), MAX_ABS_ERROR);
}
