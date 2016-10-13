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
#include "pointcloud.h"

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
        _parse_coordinates(in, _Nd, _d);
        _parse_coordinates(in, _Na, _a);
        _parse_coordinates(in, _Nc, _c);
    }

private:
    std::string _name;
    int
            _Nd, // Number of optical markers on EM base
            _Na, // number of optical markers on calibration object
            _Nc; // number EM markers on calibration object
    PointCloud<double> _d, _a, _c;

    void _parse_coordinates(std::istream &in, size_t n, PointCloud<double> &target) {
        std::string line;
        std::vector<std::string> split_line;
        for (suze_t i = 0; i < n; ++i) {
            if (!std::getline(in, line)) throw std::invalid_argument("Unexpected end of file.");
            line.erase(std::remove(line.begin(), line.end(), std::isspace), line.end());
            split(line, ',', split_line);
            if (split_line.size() != 3) throw std::invalid_argument ("Expected 3 values in line: " + line);
            target.add_point({std::stod(split_line[0]), std::stod(split_line[1]), std::stod(split_line[2])});
        }
    }

};


#endif //CIS_CAL_FILES_H
