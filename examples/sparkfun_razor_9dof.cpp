/**
 * @file sparkfun_razor_9dof.cpp
 * @brief Demonstration of Attitude Estimation using 9 Degrees of Freedom (DoF)
 *  The DoF are Accelerometer (x, y, z), Gyroscope (x, y, z), and Magnetometer (x, y, z)
 *
 * @note The sparkfun Razer IMU has magnetometer axes that are opposite of the imu and gyro.
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <ArxContainer.h>
#include <ArxTypeTraits.h>

#include <Arduino.h>
#include <SparkFunMPU9250-DMP.h>

#include "attitude_check.hpp"
#include "initializers.hpp"

// using namespace attitude_check;

MPU9250_DMP imu;  // create IMU object to get the data
attitude_check::AttitudeCheck ac; // create the attitude estimator object

const float DEG2RAD { 0.017453292519943f };

void blink_trap(const unsigned long period)
{
    while(1) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(period);
        digitalWrite(LED_BUILTIN, LOW);
        delay(period);
    }
}

void get_intial_orientation()
{
    if(imu.dataReady() ) {
        imu.update(UPDATE_ACCEL | UPDATE_COMPASS);

        auto q0 = initializers::mag_to_quat(imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az),
            -1.0f * imu.calcMag(imu.mx), -1.0f * imu.calcMag(imu.my), -1.0f * imu.calcMag(imu.mz));
        ac.set_quaternion(q0.w(), q0.x(), q0.y(), q0.z());
    }
}

void setup()
{
    SerialUSB.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    if(imu.begin() != INV_SUCCESS) {
        blink_trap(1000);
    }

    delay(1200);

    imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    imu.setGyroFSR(2000);
    imu.setAccelFSR(8);
    imu.setLPF(5);
    imu.setSampleRate(100);
    imu.setCompassSampleRate(100);

    get_intial_orientation();
}

void loop()
{
    if(imu.dataReady() ) {
        imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);

        Vec3f acc = { imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az) };
        Vec3f gyr = { imu.calcGyro(imu.gx)* DEG2RAD, imu.calcGyro(imu.gy)* DEG2RAD, imu.calcGyro(imu.gz)* DEG2RAD };
        // gyr = gyr * DEG2RAD;
        // gyr = gyr - Eigen::Vector3f{ -0.03f, 0.016f, -0.01f }; // Subtract gyro readings when not moving
        gyr[0] -= -0.03f; gyr[1] -= -0.016f; gyr[2] -= -0.01f;
        Vec3f mag = { -1.0f * imu.calcMag(imu.mx), -1.0f * imu.calcMag(imu.my), -1.0f * imu.calcMag(imu.mz) };

        long int t1 = micros();

        auto q = ac.update(acc, gyr, mag, 0.01f);

        long int t2 = micros();
        SerialUSB.print("Time: "); SerialUSB.print(t2-t1); SerialUSB.println(" us.");

        SerialUSB.println("Quaternion: " + String(q[0], 4) + ", " + String(q[1], 4) + ", " + String(q[2],
          4) + ", " + String(q[3], 4));
    }
}
