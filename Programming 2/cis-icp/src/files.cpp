/**
 * @author Ravi Gaddipati, Doran Walsten
 * @date October 23, 2016
 * rgaddip1@jhu.edu
 *
 * @brief
 * File parser implementations.
 *
 * @file
 */
#include <iostream>
#include "files.h"
#include "utils.h"

char cis::File::_parse_coordinates(std::istream &in, size_t n, cis::PointCloud &target, char delim) {
    static std::string line;
    static std::vector<std::string> split_line;
    std::vector<Point> points;
    for (size_t i = 0; i < n; ++i) {
        if (!std::getline(in, line)) throw std::invalid_argument("Unexpected end of file.");
        normalize_whitespace(line);
        if (delim == '\0') delim = guess_delim(line);
        split(line, delim, split_line);
        if (split_line.size() != 3) throw std::invalid_argument("Expected 3 values in line: " + line);
        points.emplace_back(std::stod(split_line[0]), std::stod(split_line[1]), std::stod(split_line[2]));
    }
    target.add_points(points);
    return delim;
}

void cis::RigidBody::open(std::istream &in) {
    // Parse meta info
    std::string line;
    std::getline(in, line);
    normalize_whitespace(line);
    const char delim = guess_delim(line);
    auto line_split = split(line, delim);

    size_t nmarkers;
    try {
        nmarkers = std::stoul(line_split[0]);
        this->_name = line_split[1];
    } catch (std::exception &e) {
        std::cerr << "Error parsing meta info line, expected 2 fields. \n" << line << std::endl;
        throw;
    }

    this->_clouds.resize(1);
    this->_clouds.at(0).resize(1);
    this->_parse_coordinates(in, nmarkers, this->_clouds[0][0], delim);

    // Tip coords
    std::getline(in, line);
    split(line, delim, line_split);
    _tip << std::stod(line_split.at(0)), std::stod(line_split.at(1)), std::stod(line_split.at(2));
}

void cis::BodySurface::open(std::istream &in) {
    // Parse meta info
    std::string line;
    std::getline(in, line);
    normalize_whitespace(line);

    size_t n = std::stoul(line);

    this->_clouds.resize(3);
    for (auto &c : this->_clouds) c.resize(1);
    char delim = this->_parse_coordinates(in, n, this->_clouds[0][0], '\0');

    // triangles and neighbors
    std::getline(in, line);
    n = std::stoul(line);

    std::vector<std::string> split_line;
    std::vector<Point> ind,neigh;
    for (size_t i = 0; i < n; ++i) {
        if (!std::getline(in, line)) throw std::invalid_argument("Unexpected end of file.");
        normalize_whitespace(line);
        split(line, delim, split_line);
        ind.emplace_back(std::stod(split_line[0]), std::stod(split_line[1]), std::stod(split_line[2]));
        neigh.emplace_back(std::stod(split_line[3]), std::stod(split_line[4]), std::stod(split_line[5]));
    }

    this->_clouds[1][0].add_points(ind);
    this->_clouds[2][0].add_points(neigh);
}