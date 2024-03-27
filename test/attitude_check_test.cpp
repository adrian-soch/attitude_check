#include <iostream>

#include "gtest/gtest.h"

#include "attitude_check.cpp"

const float MAX_ABS_ERROR { 0.002 }; // About 0.115 of a degree

namespace ac = attitude_check;

class ACheck_Test_Fixture : public ::testing::Test {
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

TEST_F(ACheck_Test_Fixture, invalid_init){
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

    EXPECT_THROW({
        try
        {
            ac::AttitudeCheck ac(0.2f, 0.2f, 0.0f, 0.0f, 0.0f, 0.0f);
        }
        catch(const std::invalid_argument& e)
        {
            EXPECT_STREQ("Cannot set quaternion: Magnitude of quaternion cannot be zero.", e.what() );
            throw;
        }
    }, std::invalid_argument);
}


TEST_F(ACheck_Test_Fixture, invalid_update){
    ac::AttitudeCheck ac(0.8, 0.999);

    EXPECT_THROW({
        try
        {
            ac.update(acc, gyr, mag, -0.00001f);
        }
        catch(const std::invalid_argument& e)
        {
            EXPECT_STREQ("Dt is too small.", e.what() );
            throw;
        }
    }, std::invalid_argument);
}

TEST_F(ACheck_Test_Fixture, marg_zero_gyro){
    gyr.setZero();

    ac::AttitudeCheck ac(0.5f, 0.5f, -69.0f, -69.0f, -69.0f, -69.0f);
    auto out = ac.update(acc, gyr, mag, dt);

    EXPECT_NEAR(out[0], -0.5f, MAX_ABS_ERROR);
    EXPECT_NEAR(out[1], -0.5f, MAX_ABS_ERROR);
    EXPECT_NEAR(out[2], -0.5f, MAX_ABS_ERROR);
    EXPECT_NEAR(out[3], -0.5f, MAX_ABS_ERROR);
}

TEST_F(ACheck_Test_Fixture, marg_zero_mag){
    mag.setZero();
    acc.setZero();

    quaternion::Quaternion<float> expected(-69.0f, -69.0f, -69.0f, -69.0f);
    expected = expected * (quaternion::Quaternion<float>(0.0f, gyr[0], gyr[1], gyr[2]) * 0.5);
    expected = quaternion::Quaternion<float>(-69.0f, -69.0f, -69.0f, -69.0f) + (expected * dt);
    expected.normalize();

    ac::AttitudeCheck ac(0.5f, 0.5f, -69.0f, -69.0f, -69.0f, -69.0f);
    auto out = ac.update(acc, gyr, mag, dt);

    EXPECT_NEAR(out[0], expected.w(), MAX_ABS_ERROR);
    EXPECT_NEAR(out[1], expected.x(), MAX_ABS_ERROR);
    EXPECT_NEAR(out[2], expected.y(), MAX_ABS_ERROR);
    EXPECT_NEAR(out[3], expected.z(), MAX_ABS_ERROR);
}

TEST_F(ACheck_Test_Fixture, marg_zero_acc){
    acc.setZero();

    quaternion::Quaternion<float> expected(-69.0f, -69.0f, -69.0f, -69.0f);
    expected = expected * (quaternion::Quaternion<float>(0.0f, gyr[0], gyr[1], gyr[2]) * 0.5);
    expected = quaternion::Quaternion<float>(-69.0f, -69.0f, -69.0f, -69.0f) + (expected * dt);
    expected.normalize();

    ac::AttitudeCheck ac(0.5f, 0.5f, -69.0f, -69.0f, -69.0f, -69.0f);
    auto out = ac.update(acc, gyr, mag, dt);

    EXPECT_NEAR(out[0], expected.w(), MAX_ABS_ERROR);
    EXPECT_NEAR(out[1], expected.x(), MAX_ABS_ERROR);
    EXPECT_NEAR(out[2], expected.y(), MAX_ABS_ERROR);
    EXPECT_NEAR(out[3], expected.z(), MAX_ABS_ERROR);
}

TEST_F(ACheck_Test_Fixture, imu_zero_gyro){
    gyr.setZero();

    ac::AttitudeCheck ac(0.5f, 0.5f, -69.0f, -69.0f, -69.0f, -69.0f);
    auto out = ac.update(acc, gyr, dt);

    EXPECT_NEAR(out[0], -0.5f, MAX_ABS_ERROR);
    EXPECT_NEAR(out[1], -0.5f, MAX_ABS_ERROR);
    EXPECT_NEAR(out[2], -0.5f, MAX_ABS_ERROR);
    EXPECT_NEAR(out[3], -0.5f, MAX_ABS_ERROR);
}


TEST_F(ACheck_Test_Fixture, imu_zero_acc){
    acc.setZero();

    quaternion::Quaternion<float> expected(-69.3333f, -69.0f, -69.0f, -69.0f);
    expected = expected * (quaternion::Quaternion<float>(0.0f, gyr[0], gyr[1], gyr[2]) * 0.5);
    expected = quaternion::Quaternion<float>(-69.3333f, -69.0f, -69.0f, -69.0f) + (expected * dt);
    expected.normalize();

    ac::AttitudeCheck ac(0.5f, 0.5f, -69.3333f, -69.0f, -69.0f, -69.0f);
    auto out = ac.update(acc, gyr, mag, dt);

    EXPECT_NEAR(out[0], expected.w(), MAX_ABS_ERROR);
    EXPECT_NEAR(out[1], expected.x(), MAX_ABS_ERROR);
    EXPECT_NEAR(out[2], expected.y(), MAX_ABS_ERROR);
    EXPECT_NEAR(out[3], expected.z(), MAX_ABS_ERROR);
}

TEST_F(ACheck_Test_Fixture, set_quaternion){
    ac::AttitudeCheck ac(0.5f, 0.5f, -69.0f, -69.0f, -69.0f, -69.0f);

    ac.set_quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    gyr.setZero();

    auto out = ac.update(acc, gyr, mag, dt);

    EXPECT_NEAR(out[0], 1.0f, MAX_ABS_ERROR);
    EXPECT_NEAR(out[1], 0.0f, MAX_ABS_ERROR);
    EXPECT_NEAR(out[2], 0.0f, MAX_ABS_ERROR);
    EXPECT_NEAR(out[3], 0.0f, MAX_ABS_ERROR);
}

TEST_F(ACheck_Test_Fixture, set_get_gain){
    ac::AttitudeCheck ac(0.5f, 0.5f, -69.0f, -69.0f, -69.0f, -69.0f);
    float in1 { 1.0f }, in2 { 1.0f };

    ac.set_gain(in1, in2);

    auto [imu, marg] = ac.get_gain();

    ASSERT_FLOAT_EQ(in1, imu);
    ASSERT_FLOAT_EQ(in2, marg);
}

class ACheck_Estimator_Test_Fixture : public ::testing::Test {
protected:
    // Tuple(Acc, Gyr, Mag, Expected_Quaternion)
    typedef std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector4f, float> T;
    std::vector<T> test_data;

    void SetUp() override { }
};

TEST_F(ACheck_Estimator_Test_Fixture, update_marg_with_intitial){
    ac::AttitudeCheck ac(0.033f, 0.041f, 0.49666186227f, 0.034292027603f, -0.0510015643620f, 0.86576549471f);

    /* *INDENT-OFF* */
    test_data = {
        T(Eigen::Vector3f(0.47451228800383594, 1.0183972122654374, 8.461165125378162),
            Eigen::Vector3f(-0.5006895838366212, 0.8074957770569486, 0.5528888911052677),
            Eigen::Vector3f(10.04650728048766, -6.088486551348383, -43.828022623283225),
            Eigen::Vector4f(0.49598288, 0.03266141,-0.05120194, 0.86620585),
            1.0/285.71428571),

        T(Eigen::Vector3f(0.3783742880038359, 0.5534032122654374, 8.264965125378161),
            Eigen::Vector3f(-0.5326483077698894, 0.6743323911175373, 0.5262569120490862),
            Eigen::Vector3f(11.084734479270914, -6.77198980959856, -44.35222010806493),
            Eigen::Vector4f(0.49530668, 0.03117841, -0.05155569, 0.86662634),
            1.0/285.71428571),

        T(Eigen::Vector3f(0.27340728800383585, 0.05015021226543746, 8.015791125378161),
            Eigen::Vector3f(-0.5624759846864724, 0.5560845889656695, 0.5166698184678814),
            Eigen::Vector3f(10.19482545174241, -6.923879422543039, -43.97779333322085),
            Eigen::Vector4f(0.49463786, 0.0299162, -0.05209187, 0.86702067),
            1.0/285.71428571),

        T(Eigen::Vector3f(0.3067612880038358, -0.2853517877345626, 8.159017125378163),
            Eigen::Vector3f(-0.5880433128989375, 0.39415992228264346, 0.5017542346803379),
            Eigen::Vector3f(9.30491642421391, -7.075769035487526, -43.60336655837678),
            Eigen::Vector4f(0.49463786, 0.0299162, -0.05209187, 0.86702067),
            1.0/285.71428571)
        };
    /* *INDENT-ON* */

    size_t count = 0;
    for(auto tuple : test_data) {
        auto [t_acc, t_gyr, t_mag, q_exp, t_dt] = tuple;

        auto out = ac.update(t_acc, t_gyr, t_mag, t_dt);

        SCOPED_TRACE("At iteration " + std::to_string(count++));
        EXPECT_NEAR(out[0], q_exp[0], MAX_ABS_ERROR);
        EXPECT_NEAR(out[1], q_exp[1], MAX_ABS_ERROR);
        EXPECT_NEAR(out[2], q_exp[2], MAX_ABS_ERROR);
        EXPECT_NEAR(out[3], q_exp[3], MAX_ABS_ERROR);
    }
}

TEST_F(ACheck_Estimator_Test_Fixture, update_imu_with_intitial){
    ac::AttitudeCheck ac(0.033f, 0.041f, 0.49666186227f, 0.034292027603f, -0.0510015643620f, 0.86576549471f);

    /* *INDENT-OFF* */
        test_data = {
        T(Eigen::Vector3f(0.47451228800383594, 1.0183972122654374, 8.461165125378162),
            Eigen::Vector3f(-0.5006895838366212, 0.8074957770569486, 0.5528888911052677),
            Eigen::Vector3f(10.04650728048766, -6.088486551348383, -43.828022623283225),
            Eigen::Vector4f(0.49596236, 0.03276414, -0.05113933,  0.86621742),
            1.0/285.71428571),

        T(Eigen::Vector3f(0.3783742880038359, 0.5534032122654374, 8.264965125378161),
            Eigen::Vector3f(-0.5326483077698894, 0.6743323911175373, 0.5262569120490862),
            Eigen::Vector3f(11.084734479270914, -6.77198980959856, -44.35222010806493),
            Eigen::Vector4f(0.49526518, 0.03134777, -0.05143123, 0.86665134),
            1.0/285.71428571),

        T(Eigen::Vector3f(0.27340728800383585, 0.05015021226543746, 8.015791125378161),
            Eigen::Vector3f(-0.5624759846864724, 0.5560845889656695, 0.5166698184678814),
            Eigen::Vector3f(10.19482545174241, -6.923879422543039, -43.97779333322085),
            Eigen::Vector4f(0.49457367, 0.03011946, -0.05192297, 0.86706038),
            1.0/285.71428571),

        T(Eigen::Vector3f(0.3067612880038358, -0.2853517877345626, 8.159017125378163),
            Eigen::Vector3f(-0.5880433128989375, 0.39415992228264346, 0.5017542346803379),
            Eigen::Vector3f(9.30491642421391, -7.075769035487526, -43.60336655837678),
            Eigen::Vector4f(0.49457367, 0.03011946, -0.05192297, 0.86706038),
            1.0/285.71428571)
        };
    /* *INDENT-ON* */

    size_t count = 0;
    for(auto tuple : test_data) {
        auto [t_acc, t_gyr, t_mag, q_exp, t_dt] = tuple;

        auto out = ac.update(t_acc, t_gyr, t_dt);

        SCOPED_TRACE("At iteration " + std::to_string(count++));
        EXPECT_NEAR(out[0], q_exp[0], MAX_ABS_ERROR);
        EXPECT_NEAR(out[1], q_exp[1], MAX_ABS_ERROR);
        EXPECT_NEAR(out[2], q_exp[2], MAX_ABS_ERROR);
        EXPECT_NEAR(out[3], q_exp[3], MAX_ABS_ERROR);
    }
}
