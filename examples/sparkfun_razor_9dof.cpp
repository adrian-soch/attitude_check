#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <SparkFunMPU9250-DMP.h>

#include "attitude_check.hpp"
#include "initializers.hpp"

using namespace attitude_check;

MPU9250_DMP imu;
AttitudeCheck ac(0.031, 0.041);

void blink_trap(const unsigned long period) {
    while (1) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(period);
        digitalWrite(LED_BUILTIN, LOW);
        delay(period);
    }
}

void get_intial_orientation() {
    if ( imu.dataReady() ) {
        imu.update(UPDATE_ACCEL | UPDATE_COMPASS);

        Eigen::Vector3f acc = {imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az)};
        Eigen::Vector3f mag = { imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz)};
        mag *= -1.0f;

        auto q0 = initializers::mag_to_quat(acc, mag);
        ac.set_quaternion(q0.w(), q0.x(), q0.y(), q0.z());
    }
}

void setup() {
    SerialUSB.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    if (imu.begin() != INV_SUCCESS) {
        blink_trap(1000);
    }

    delay(1200);

    // imu.dmpBegin(DMP_FEATURE_SEND_CAL_GYRO);

    imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    imu.setGyroFSR(500); // Set gyro to 2000 dps
    imu.setAccelFSR(8);
    imu.setLPF(10);        // Set LPF corner frequency to 5Hz
    imu.setSampleRate(100); // Set sample rate to 100Hz
    imu.setCompassSampleRate(100);

    get_intial_orientation();
}

void loop() {
    if ( imu.dataReady() ) {
        imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);

        Eigen::Vector3f acc = {imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az)};
        Eigen::Vector3f gyr = {imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz)};
        gyr = gyr * DEG_TO_RAD;
        gyr = gyr - Eigen::Vector3f{-0.03, 0.016, -0.01};
        Eigen::Vector3f mag = { imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz)};
        mag *= -1.0f;

        // SerialUSB.println(String(gyr[0], 3) + " " + String(gyr[1], 3) + " " + String(gyr[2], 3));
        // SerialUSB.println(String(acc[0], 3) + " " + String(acc[1], 3) + " " + String(acc[2], 3));

        // unsigned long startMicros = micros();

        auto q = ac.update(acc, gyr, 0.01f);
        SerialUSB.println("Quaternion: " + String(q[0], 4)  + ", " +  String(q[1], 4) + ", " +  String(q[2], 4) + ", " +  String(q[3], 4));
        // unsigned long duration = micros() - startMicros;
        // SerialUSB.println(int(duration));
    }
}



// #include <SparkFunMPU9250-DMP.h>

// #define SerialPort SerialUSB

// MPU9250_DMP imu;

// void printIMUData(void)
// {
//   // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
//   // are all updated.
//   // Quaternion values are, by default, stored in Q30 long
//   // format. calcQuat turns them into a float between -1 and 1
//   float q0 = imu.calcQuat(imu.qw);
//   float q1 = imu.calcQuat(imu.qx);
//   float q2 = imu.calcQuat(imu.qy);
//   float q3 = imu.calcQuat(imu.qz);

//   SerialPort.println("Quaternion: " + String(q0, 4) + ", " +
//                     String(q1, 4) + ", " + String(q2, 4) +
//                     ", " + String(q3, 4));
//   // SerialPort.println("R/P/Y: " + String(imu.roll) + ", "
//             // + String(imu.pitch) + ", " + String(imu.yaw));
//   // SerialPort.println("Time: " + String(imu.time) + " ms");
//   // SerialPort.println();
// }

// void setup()
// {
//   SerialPort.begin(115200);

//   // Call imu.begin() to verify communication and initialize
//   if (imu.begin() != INV_SUCCESS)
//   {
//     while (1)
//     {
//       SerialPort.println("Unable to communicate with MPU-9250");
//       SerialPort.println("Check connections, and try again.");
//       SerialPort.println();
//       delay(5000);
//     }
//   }

//   imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
//                DMP_FEATURE_GYRO_CAL, // Use gyro calibration
//               100); // Set DMP FIFO rate to 10 Hz
//   // DMP_FEATURE_LP_QUAT can also be used. It uses the
//   // accelerometer in low-power mode to estimate quat's.
//   // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
// }

// void loop()
// {
//   // Check for new data in the FIFO
//   if ( imu.fifoAvailable() )
//   {
//     // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
//     if ( imu.dmpUpdateFifo() == INV_SUCCESS)
//     {
//       // computeEulerAngles can be used -- after updating the
//       // quaternion values -- to estimate roll, pitch, and yaw
//     //   imu.computeEulerAngles();
//       printIMUData();
//     }
//   }
// }
