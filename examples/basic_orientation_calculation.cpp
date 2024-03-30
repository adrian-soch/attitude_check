/**
 * @file basic_orientation_calculation.cpp
 * @brief Show how to use attitude check. Reads data from csv file and
 *  writes quaternion result into a csv file
 *
 * @note You can create a csv file by using the `scripts/create_sensor_csv.py`
 *
 * @copyright Copyright (c) 2024
 *
 */

// Includes for handling the file
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

// Include for using attitude check library
#include "attitude_check.hpp"

using Vec3f = Eigen::Vector3f;

std::vector<std::vector<float>> readCsv(const std::string& filename) {
    std::ifstream file(filename);
    std::vector<std::vector<float>> data;

    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return data; // Early return on failure to open the file
    }

    // Get headers before processing numbers
    std::string headers;
    std::getline(file, headers);

    std::string line;
    while (std::getline(file, line)) {
        std::vector<float> row;
        std::stringstream iss(line);
        std::string value;

        // Split the line by commas
        while (std::getline(iss, value, ',')) {
            try {
                // Convert string to float and add to the row
                row.push_back(std::stof(value));
            } catch (const std::invalid_argument& e) {
                // Handle the case where the conversion fails
                std::cerr << "Invalid number: " << value << std::endl;
                continue;
            }
        }

        data.push_back(row);
    }

    file.close();
    return data;
}


void writeToCSV(const std::string& filePath, const std::vector<std::array<float, 4>>& data) {
    std::ofstream file(filePath);

    if (!file.is_open()) {
        throw std::runtime_error("Could not open file");
    }

    for (const auto& arr : data) {
        file << arr[0] << ',' << arr[1] << ',' << arr[2] << ',' << arr[3] << '\n';
    }
    file.close();
}

std::vector<std::array<float, 4> > simulate(std::vector<std::vector<float>>& data, float dt, attitude_check::AttitudeCheck* filter)
{
    filter->set_gain(0.033, 0.041);

    std::vector<std::array<float, 4> > output;
    output.reserve(data.size());

    // Uncomment if you dont want to use default 1,0,0,0 quaternion
    // auto measurement = data.front();
    // data.erase(data.begin());
    // Vec3f a0 = Eigen::Map<Eigen::Vector3f>(&measurement[0]);
    // Vec3f m0 = Eigen::Map<Eigen::Vector3f>(&measurement[6]);
    // filter->get_initial_orientation(a0, m0);

    for (auto sense : data) {
        Vec3f acc = Eigen::Map<Eigen::Vector3f>(&sense[0]);
        Vec3f gyr = Eigen::Map<Eigen::Vector3f>(&sense[3]);
        Vec3f mag = Eigen::Map<Eigen::Vector3f>(&sense[6]);

        auto q = filter->update(acc, gyr, mag, dt);

        output.push_back(q);
    }

    return output;
}

int main()
{
    const float RATE {285.7142857142857f};
    const std::string csvFilename = "/home/adrian/dev/attitude_check/docs/sample_sensor_data_hz_285.7142857142857.csv";
    const std::string outFilename = "/home/adrian/dev/attitude_check/docs/attitude_check_marg_results.csv";

    // Read CSV data
    std::vector<std::vector<float> > csvData = readCsv(csvFilename);

    // Create filter object
    attitude_check::AttitudeCheck ac;

    // Get quaternion results
    auto results = simulate(csvData, 1.0/RATE, &ac);

    writeToCSV(outFilename, results);
    return 0;
}
