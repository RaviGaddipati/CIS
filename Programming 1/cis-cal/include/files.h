/**
 * @author Ravi Gaddipati
 * @date October 13, 2016
 * rgaddip1@jhu.edu
 *
 * @brief
 * Data file readers.
 *
 * @file
 */

#ifndef CIS_CAL_FILES_H
#define CIS_CAL_FILES_H

#include <string>
#include <fstream>
#include <algorithm>
#include <vector>
#include <iostream>
#include "utils.h"
#include "pointcloud.h"

/**
 * One set of marker locations.
 */
struct CalibrationFrame {
    PointCloud<double> opt_marker_embase, opt_marker_calobj, em_marker_calobj;
};

/**
 * @brief
 * Read and parse a calibration reading/body file. A Calibration Body file is just a readings file with
 * nframes = 1.
 */
class CalibrationFile {

public:

    CalibrationFile(const std::string &filename) {
        open(filename);
    }

    /**
     * @brief
     * Load the point clouds in the file.
     * @param filename
     */
    void open(const std::string &filename) {
        std::ifstream in(filename);
        if (!in.good()) throw std::invalid_argument("Error opening file: " + filename);

        std::string line;
        size_t nd, na, nc, nframes;

        // Header information line
        std::getline(in, line);
        line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
        auto split_line = split(line, ',');

        if (split_line.size() == 4) {
            // Cal body file
            nframes = 1;
            _name = split_line[3];
        } else if (split_line.size() == 5) {
            // Cal readings file
            nframes = std::stoul(split_line[3]);
            _name = split_line[4];
        } else throw std::invalid_argument("Expected line of format \"N_D, N_A, N_C, [N_FRAMES], NAME\" got " + line);


        try {
            nd = std::stoul(split_line[0]);
            na = std::stoul(split_line[1]);
            nc = std::stoul(split_line[2]);
        } catch (std::exception &e) {
            std::cerr << "Error parsing arguments in: " << line << std::endl;
            throw;
        }

        _frames.clear();
        for (size_t i = 0; i < nframes; ++i) {
            _frames.emplace_back();
            auto &frame = _frames.back();
            _parse_coordinates(in, nd, frame.opt_marker_embase);
            _parse_coordinates(in, na, frame.opt_marker_calobj);
            _parse_coordinates(in, nc, frame.em_marker_calobj);
        }
    }

    std::string name() const {
        return _name;
    }

    const CalibrationFrame &frame(size_t i = 0) const {
        return _frames.at(i);
    }

    size_t size() const {
        return _frames.size();
    }

    typename std::vector<CalibrationFrame>::const_iterator begin() const { return _frames.begin(); }

    typename std::vector<CalibrationFrame>::const_iterator end() const { return _frames.end(); }

private:
    std::string _name;
    std::vector<CalibrationFrame> _frames;

    /**
 * @brief
 * Parses the n next lines of the stream.
 * @param in input stream
 * @param n number of lines to parse
 * @param target Target to add points to.
 */
    void _parse_coordinates(std::istream &in, size_t n, PointCloud<double> &target) {
        std::string line;
        std::vector<std::string> split_line;
        std::vector<PointCloud<double>::Point> points;
        for (size_t i = 0; i < n; ++i) {
            if (!std::getline(in, line)) throw std::invalid_argument("Unexpected end of file.");
            line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
            split(line, ',', split_line);
            if (split_line.size() != 3) throw std::invalid_argument ("Expected 3 values in line: " + line);
            points.emplace_back(std::stod(split_line[0]), std::stod(split_line[1]), std::stod(split_line[2]));
        }
        target.add_points(points);
    }

};

// Convenience aliases
using CalibrationBody = CalibrationFile;
using CalibrationReadings = CalibrationFile;


TEST_CASE ("Cal Body file") {
    const std::string tmp_file("_cis_tmp_file");
    std::ofstream t(tmp_file);
    t << "1,2,\t3, NAME\n1,1 ,1\n2,2,2\n3,3,3\n4,4,4\n5,5,5\n6,6,6";
    t.close();

    CalibrationFile cb(tmp_file);

            CHECK(cb.frame().opt_marker_embase.size() == 1);
            CHECK(cb.frame().opt_marker_calobj.size() == 2);
            CHECK(cb.frame().em_marker_calobj.size() == 3);
            CHECK(cb.name() == "NAME");

    auto d = cb.frame().opt_marker_embase;
    auto a = cb.frame().opt_marker_calobj;
    auto c = cb.frame().em_marker_calobj;

    PointCloud<double>::Point p;

            CHECK(d.size() == 1);
    p = {1, 1, 1};
            CHECK(d.at(0) == p);
            CHECK_THROWS(d.at(1));

            CHECK(a.size() == 2);
    p = {2, 2, 2};
            CHECK(a.at(0) == p);
    p = {3, 3, 3};
            CHECK(a.at(1) == p);
            CHECK_THROWS(a.at(2));

            CHECK(c.size() == 3);
    p = {4, 4, 4};
            CHECK(c.at(0) == p);
    p = {5, 5, 5};
            CHECK(c.at(1) == p);
    p = {6, 6, 6};
            CHECK(c.at(2) == p);
            CHECK_THROWS(c.at(3));


    remove(tmp_file.c_str());
}

TEST_CASE ("Cal Readings file") {
    const std::string tmp_file("_cis_tmp_file");
    std::ofstream t(tmp_file);
    t << "1,2,\t3,2, NAME\n1,1 ,1\n2,2,2\n3,3,3\n4,4,4\n5,5,5\n6,6,6\n1,1 ,1\n2,2,2\n3,3,3\n4,4,4\n5,5,5\n6,6,6";
    t.close();

    CalibrationFile cb(tmp_file);
            CHECK(cb.size() == 2);

            CHECK(cb.name() == "NAME");

    auto d = cb.frame(0).opt_marker_embase;
    auto a = cb.frame(0).opt_marker_calobj;
    auto c = cb.frame(0).em_marker_calobj;

    PointCloud<double>::Point p;

            CHECK(d.size() == 1);
    p = {1, 1, 1};
            CHECK(d.at(0) == p);
            CHECK_THROWS(d.at(1));

            CHECK(a.size() == 2);
    p = {2, 2, 2};
            CHECK(a.at(0) == p);
    p = {3, 3, 3};
            CHECK(a.at(1) == p);
            CHECK_THROWS(a.at(2));

            CHECK(c.size() == 3);
    p = {4, 4, 4};
            CHECK(c.at(0) == p);
    p = {5, 5, 5};
            CHECK(c.at(1) == p);
    p = {6, 6, 6};
            CHECK(c.at(2) == p);
            CHECK_THROWS(c.at(3));

    d = cb.frame(1).opt_marker_embase;
    a = cb.frame(1).opt_marker_calobj;
    c = cb.frame(1).em_marker_calobj;

            CHECK(d.size() == 1);
    p = {1, 1, 1};
            CHECK(d.at(0) == p);
            CHECK_THROWS(d.at(1));

            CHECK(a.size() == 2);
    p = {2, 2, 2};
            CHECK(a.at(0) == p);
    p = {3, 3, 3};
            CHECK(a.at(1) == p);
            CHECK_THROWS(a.at(2));

            CHECK(c.size() == 3);
    p = {4, 4, 4};
            CHECK(c.at(0) == p);
    p = {5, 5, 5};
            CHECK(c.at(1) == p);
    p = {6, 6, 6};
            CHECK(c.at(2) == p);
            CHECK_THROWS(c.at(3));


    remove(tmp_file.c_str());
}

#endif //CIS_CAL_FILES_H
