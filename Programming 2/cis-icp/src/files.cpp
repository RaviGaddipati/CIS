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

    Eigen::Array<double, 3, 1>
            min = {std::numeric_limits<double>::max(),
                   std::numeric_limits<double>::max(),
                   std::numeric_limits<double>::max()},
            max = {std::numeric_limits<double>::min(),
                   std::numeric_limits<double>::min(),
                   std::numeric_limits<double>::min()};
    for (size_t i = 0; i < ind.size(); ++i) {
        this->_tri.row(i) = ind[i];
        _spheres.emplace_back(_bounding_sphere(i));
        min = min.min(_spheres.back().centroid.array());
        max = max.max(_spheres.back().centroid.array());
        this->_neighbor.row(i) = neigh[i];
    }

    _root.included_pts.resize(_tri.rows());
    std::iota(_root.included_pts.begin(), _root.included_pts.end(), 0);
    _root.max_dim = max;
    _root.min_dim = min;

}

void cis::SurfaceFile::_reorder_longest_edge(cis::Point &v1, cis::Point &v2, cis::Point &v3) {
    static double e1n,e2n,e3n;
    static Point swp;

    e1n = (v2 - v1).norm();
    e2n = (v3 - v1).norm();
    e3n = (v3 - v2).norm();

    if (e3n > e2n && e3n > e1n) {
        // v3-v2 is longest
        swp = v1;
        v1 = v3;
        v3 = swp;
    }

    else if (e2n > e1n && e2n > e3n) {
        // v2-v3 is longest
        swp = v2;
        v2 = v3;
        v3 = swp;
    }

    // Nothing to be done if v1-v2 is longest
}

cis::SurfaceFile::BoundingSphere cis::SurfaceFile::_bounding_sphere(size_t triangle_idx) {
    static Point u,v,d;
    static double g;
    static BoundingSphere ret;

    const auto &v_idx = _tri.row(triangle_idx);
    Point a = vertices().at(v_idx(0)), b = vertices().at(v_idx(1)), c = vertices().at(v_idx(2));

    _reorder_longest_edge(a, b, c);
    ret.centroid = (a + b) / 2;
    u = a - ret.centroid;
    v = c - ret.centroid;
    d = (u.cross(v)).cross(u);
    g = (v.norm() - u.norm()) / (d * 2).dot(v - u);

    if (g > 0) ret.centroid += g * d;
    ret.radius = (ret.centroid - a).norm();
    ret.triangle = triangle_idx;
    return ret;
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
