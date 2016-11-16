/**
 * @author Ravi Gaddipati, Doran Walsten
 * @date Nov 16, 2016
 * rgaddip1@jhu.edu
 *
 * @brief
 * File parser implementations.
 *
 * @file
 */
#include <iostream>
#include <numeric>
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

void cis::SurfaceFile::open(std::istream &in) {
    // Parse meta info
    std::string line;
    std::getline(in, line);
    normalize_whitespace(line);

    const size_t nvert = std::stoul(line);

    this->_clouds.resize(3);
    for (auto &c : this->_clouds) c.resize(1);
    char delim = this->_parse_coordinates(in, nvert, this->_clouds[0][0], '\0');

    // triangles and neighbors
    std::getline(in, line);
    const size_t ntri = std::stoul(line);

    std::vector<std::string> split_line;
    std::vector<Eigen::Array<long, 3, 1>> ind,neigh;
    for (size_t i = 0; i < ntri; ++i) {
        if (!std::getline(in, line)) throw std::invalid_argument("Unexpected end of file.");
        normalize_whitespace(line);
        split(line, delim, split_line);
        ind.emplace_back(std::stod(split_line[0]), std::stod(split_line[1]), std::stod(split_line[2]));
        neigh.emplace_back(std::stod(split_line[3]), std::stod(split_line[4]), std::stod(split_line[5]));

        // All triangle vertex should be positive
        assert(ind.back()(0) >= 0 && ind.back()(1) >= 0 && ind.back()(2) >= 0);
        // Make sure vertex idx is valid
        assert(ind.back()(0) < nvert && ind.back()(1) < nvert && ind.back()(2) < nvert);
        // Valid triangle index
        assert(neigh.back()(0) < (long)ntri && neigh.back()(1) < (long)ntri && neigh.back()(2) < (long)ntri);
    }

    this->_tri.resize(ind.size(), Eigen::NoChange);
    this->_neighbor.resize(neigh.size(), Eigen::NoChange);

    assert(ind.size() == neigh.size());

    for (size_t i = 0; i < ind.size(); ++i) {
        this->_tri.row(i) = ind[i];
        this->_neighbor.row(i) = neigh[i];
    }

}

Eigen::Array<double, 9, Eigen::Dynamic> cis::SurfaceFile::cat_triangles() const {
    Eigen::Array<double, 9, Eigen::Dynamic> ret;
    long ntri = _tri.rows();
    if (ntri == 0) throw std::invalid_argument("No triangles loaded.");
    ret.resize(Eigen::NoChange, ntri);
    const auto &verts = vertices();
    for (long i = 0; i < ntri; ++i) {
        ret.block<9,1>(0,i) << verts.at(_tri.row(i)(0)), verts.at(_tri.row(i)(1)), verts.at(_tri.row(i)(2));
    }
    return ret;
}

/**
 *
 * @param in File corresponding to the readings file of the sample frames
 */
void cis::SampleReadings::open(std::istream &in) {
    // Parse meta info

    std::string line;
    std::getline(in, line);
    line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
    auto line_split = split(line, ',');

    size_t nd, ns;
    try {
        ns = std::stoul(line_split[1]); //Number of frames to expect
        nd = std::stoul(line_split[0]) - this->N_a - this->N_b; //Number of dummy LEDs
        this->_name = line_split[2];
    } catch (std::exception &e) {
        std::cerr << "Error parsing meta info line, expected 4 fields. \n" << line << std::endl;
        throw;
    }

    this->_clouds.resize(2);
    for (auto &c : this->_clouds) c.resize(ns);
    PointCloud temp;
    for (size_t frame = 0; frame < ns; ++frame ) {
        this->_parse_coordinates(in, this->N_a, this->_clouds[0][frame], ',');
        this->_parse_coordinates(in, this->N_b, this->_clouds[1][frame],',');
        this->_parse_coordinates(in, nd, temp, ','); //Want this stuff to be ignored in a temporary PointCloud
    }
}
