/**
 * @file basic_orientation_calculation.cpp
 * @brief Show how to use attitude check. Reads data from csv file and
 *  writes quaternion result into a csv file
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

const std::string csvFilename = "your_csv_file.csv"; // Replace with your CSV file path
const std::string txtFilename = "row_count.txt";     // Output text file

// Function to read CSV file into a 2D vector of floats
std::vector<std::vector<float> > readCSV(const std::string& filename)
{
    std::ifstream file(filename);
    std::vector<std::vector<float> > data;

    if(!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return data;
    }

    std::string line;
    while(std::getline(file, line)) {
        std::vector<float> row;
        std::istringstream iss(line);
        float value;

        while(iss >> value) {
            row.push_back(value);
            if(iss.peek() == ',')
                iss.ignore();
        }

        data.push_back(row);
    }

    file.close();
    return data;
}

int main()
{
    attitude_check::AttitudeCheck ac;

    auto [g1, g2] = ac.get_gain();
    std::cout << g1 << std::endl;


    // Read CSV data
    std::vector<std::vector<float> > csvData = readCSV(csvFilename);

    // Get the number of rows
    size_t numRows = csvData.size();

    // Write row count to a text file
    std::ofstream txtFile(txtFilename);
    if(txtFile.is_open()) {
        txtFile << "Number of rows in CSV: " << numRows << std::endl;
        txtFile.close();
        std::cout << "Row count written to " << txtFilename << std::endl;
    } else {
        std::cerr << "Error writing to file: " << txtFilename << std::endl;
    }

    return 0;
}
