/**
 * @file sparkfun_razor_6dof.cpp
 * @brief Demonstration of Attitude Estimation using 6 Degrees of Freedom (DoF)
 *  The DoF are Accelerometer (x, y, z) and Gyroscope (x, y, z);
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <SparkFunMPU9250-DMP.h>

#include "attitude_check.hpp"
#include "initializers.hpp"

MPU9250_DMP imu;
attitude_check::AttitudeCheck ac(0.031, 0.041);

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

        Eigen::Vector3f acc = { imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az) };

        ac.get_initial_orientation(acc);
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

    imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    imu.setGyroFSR(2000); // Set gyro to 2000 dps
    imu.setAccelFSR(8);
    imu.setLPF(5);          // Set LPF corner frequency to 5Hz
    imu.setSampleRate(100); // Set sample rate to 100Hz

    get_intial_orientation();
}

void loop()
{
    if(imu.dataReady() ) {
        imu.update(UPDATE_ACCEL | UPDATE_GYRO);

        Eigen::Vector3f acc = { imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az) };
        Eigen::Vector3f gyr = { imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz) };
        gyr = gyr * DEG2RAD;
        gyr = gyr - Eigen::Vector3f{ -0.03, 0.016, -0.01 }; // Subtract gyro readings when not moving

        auto q = ac.update(acc, gyr, 0.01f);
        SerialUSB.println("Quaternion: " + String(q[0], 4) + ", " + String(q[1], 4) + ", " + String(q[2],
          4) + ", "
          + String(q[3], 4));
    }
}
