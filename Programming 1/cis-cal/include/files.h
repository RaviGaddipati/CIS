//
// Created by gaddra on 10/12/16.
//

#ifndef CIS_CAL_FILES_H
#define CIS_CAL_FILES_H

#include <string>
#include <fstream>
#include <algorithm>
#include <vector>
#include <iostream>
#include "utils.h"

class CalibrationBody {

public:

    CalibrationBody(const std::string &filename) {
        open(filename);
    }

    void open(const std::string &filename) {
        std::ifstream in(filename);
        if (!in.good()) throw std::invalid_argument("Error opening file: " + filename);

        std::string line;

        // Header information line
        std::getline(in, line);
        line.erase(std::remove(line.begin(), line.end(), std::isspace), line.end());
        auto split_line = split(line, ',');
        if (split_line.size() != 4) throw std::invalid_argument("Expected line of format \"N_D, N_A, N_C, got " + line);
        try {
            _Nd = std::stoi(split_line[0]);
            _Na = std::stoi(split_line[1]);
            _Nc = std::stoi(split_line[2]);
        } catch (std::exception &e) {
            std::cerr << "Error parsing arguments in: " << line << std::endl;
            throw;
        }
        _name = split_line[3];

        int i;
        for (i = 0; i < _Nd; ++i) {
            if (!std::getline(in, line)) throw std::invalid_argument("Unexpected end of file.");

        }

        for (i = 0; i < _Na; ++i) {
            if (!std::getline(in, line)) throw std::invalid_argument("Unexpected end of file.");

        }

        for (i = 0; i < _Nc; ++i) {
            if (!std::getline(in, line)) throw std::invalid_argument("Unexpected end of file.");

        }
    }

private:
    std::string _name;
    int
            _Nd, // Number of optical markers on EM base
            _Na, // number of optical markers on calibration object
            _Nc; // number EM markers on calibration object

};


#endif //CIS_CAL_FILES_H
