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
    sur = Surface(cat_triangles(), neighbor_triangles());
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

/*************** TEST CASES ***************/

TEST_CASE("Rigid Body File") {
    const std::string tmpfile = "cis-icp-doctest.tmp";
    {
        std::ofstream o(tmpfile);
        o << "6 Problem4-BodyB.txt\n"
          <<    "    -8.161    -30.321    -80.776\n"
          <<    "    13.074      6.228    -44.869\n"
          <<    "    24.232      1.885   -103.562\n"
          <<    "    10.774     -5.880    -60.917\n"
          <<    "    28.722     18.354    -91.628\n"
          <<    "   -31.677     16.526    -87.024\n"
          <<    "     0.000      0.000      0.000\n";
    }

    SUBCASE("File wrapper") {
        cis::RigidBody rb(tmpfile);
        const std::vector<cis::Point> pts = {{-8.161, -30.321, -80.776}, {-31.677, 16.526, -87.024}, {0,0,0}};
        CHECK(rb.markers().at(0).isApprox(pts.at(0)));
        CHECK(rb.markers().at(rb.markers().size() - 1).isApprox(pts.at(1)));
        CHECK(rb.tip().isApprox(pts.at(2)));
    }

    remove(tmpfile.c_str());
}

TEST_CASE("Surface File") {
    const std::string tmpfile = "cis-icp-doctest.tmp";
    {
        std::ofstream o(tmpfile);
        o << "2\n"
          << "-1 0 1\n"
          << "-2 0 2\n"
          << "2\n"
          << "0 0 1 -1 -1 -1\n"
          << "0 0 0 -2 -2 -2\n";
    }

    SUBCASE("File wrapper") {
        cis::SurfaceFile rb(tmpfile);
        const cis::Point p = {-1, 0, 1};
        const Eigen::Array<long,1,3> a = {0,0,0}, b = {-2,-2,-2}, c = ((rb.neighbor_triangles().row(1)));
        CHECK(rb.vertices().at(0) == p);
        CHECK(((rb.triangles().row(rb.triangles().rows() - 1).matrix()) == a.matrix()) == true);
        bool h = (c.matrix() == b.matrix());
        CHECK(h);
    }

    remove(tmpfile.c_str());
}

TEST_CASE("SampleReadings File") {
    const std::string tmpfile = "cis-icp-doctest.tmp";
    {
        std::ofstream o(tmpfile);
        o << "3, 2, PA3-A-Debug-SampleReadingsTest.txt 0\n"
          << "-81.34,   -43.04,   117.06\n"
          << "-35.48,    -2.92,    63.39\n"
          << "-76.64,    11.69,   109.42\n"
          << "12.12,   -17.85,    93.44\n"
          << "1.75,   -52.74,    46.84\n"
          << "-28.38,    10.92,    42.26\n";
    }

    SUBCASE("File wrapper") {
        cis::SampleReadings sr(tmpfile, 1, 1);
        const std::vector<cis::Point> pts = {{-81.34, -43.04, 117.06},
                                             {-35.48, -2.92,  63.39},
                                             {-76.64, 11.69,  109.42},
                                             {12.12,  -17.85, 93.44},
                                             {1.75,   -52.74, 46.84},
                                             {-28.38, 10.92,  42.26}};

        CHECK( sr.pointer_rigid_body().size() == 2);
        CHECK(sr.fixed_rigid_body().size() == 2);

        auto test_pointer = sr.pointer_rigid_body().at(0);
        CHECK(test_pointer.at(0) == pts.at(0));
        CHECK_THROWS(test_pointer.at(1));
        test_pointer = sr.pointer_rigid_body().at(1);
        CHECK(test_pointer.at(0) == pts.at(3));
        CHECK_THROWS(test_pointer.at(1));

        auto test_fixed = sr.fixed_rigid_body().at(0);
        CHECK(test_fixed.at(0) == pts.at(1));
        CHECK_THROWS(test_fixed.at(1));
        test_fixed = sr.fixed_rigid_body().at(1);
        CHECK(test_fixed.at(0) == pts.at(4));
        CHECK_THROWS(test_fixed.at(1));


    }

    remove(tmpfile.c_str());
}

TEST_CASE("Surface tree") {
    /**
     * The test case defines a cube, where each face is split into two triangles. This allows for the
     * testing of bounding sphere centroids that lie on the median split plane.
     */
    const std::string tmpfile = "cis-icp-doctest.tmp";
    {
        std::ofstream o(tmpfile);
        o << "8\n"
          << "0 0 0\n"
          << "1 1 0\n"
          << "0 1 0\n"
          << "0 0 2\n"
          << "0 1 2\n"
          << "1 1 2\n"
          << "1 0 2\n"
          << "1 0 0\n"
          << "12\n"
          << "7 5 6 -1 -1 -1\n"
          << "7 1 5 -1 -1 -1\n"
          << "4 3 6 -1 -1 -1\n"
          << "4 5 6 -1 -1 -1\n"
          << "0 3 6 -1 -1 -1\n"
          << "7 0 6 -1 -1 -1\n"
          << "7 0 2 -1 -1 -1\n"
          << "7 2 1 -1 -1 -1\n"
          << "1 2 5 -1 -1 -1\n"
          << "5 2 4 -1 -1 -1\n"
          << "0 2 4 -1 -1 -1\n"
          << "0 4 3 -1 -1 -1\n";
    }

    cis::SurfaceFile f(tmpfile);
    cis::Surface s(f.cat_triangles(), f.neighbor_triangles());

    SUBCASE("Centroids") {
        const std::vector<Eigen::Vector3d> pts = {{1,0.5,1}, {0.5,0.5,2}, {0.5,0,1}, {0.5,0.5,0}, {0.5,1,1},{0, 0.5,1}};
        for (size_t i = 0; i < s.root()->size(); ++i) {
            CHECK(s.sphere_centroid(i) == pts.at(i/2));
        }

    }

    SUBCASE("Tree node sizes") {
        std::vector<std::shared_ptr<cis::Surface::Division>> q = { s.root() };
        while (q.size()) {
            std::shared_ptr<cis::Surface::Division> curr = q.back();
            q.pop_back();
            if (curr->size() > 1) {
                int size = 0;
                if (curr->left()) {
                    size += curr->left()->size();
                    q.push_back(curr->left());
                }
                if (curr->right()) {
                    size += curr->right()->size();
                    q.push_back(curr->right());
                }
                CHECK(size == curr->size() - 1);
            }
        }
    }

    SUBCASE("Closest Point") {
        //Generate random points with x >= 1, 1 >= y > 0, 2 >= z >= 0 (project onto face of prism)
        std::default_random_engine generator;

        std::uniform_real_distribution<double> range_distribution(1.0,20.0);
        std::uniform_real_distribution<double> step_distribution(0.0,1.0);
        std::uniform_real_distribution<double> z_distribution(0.0,2.0);
        cis::PointCloud pc1;

        for (int j = 0; j < 5; j++) {
            cis::Point toAdd = {range_distribution(generator), step_distribution(generator), z_distribution(generator)};
            pc1.add_point(toAdd);
        }

        //Generate random points with 1 >= x >= 0 , y >= 1, 2 >= z >= 0 (project onto another face of prism
        cis::PointCloud pc2;
        for (int j = 0; j < 5; j++) {
            cis::Point toAdd = {step_distribution(generator), range_distribution(generator), z_distribution(generator)};
            pc2.add_point(toAdd);
        }

        //Generate random point x >=1, y >=1, 2 >= z >= 0 (project onto an edge of the prism
        cis::PointCloud pc3;
        for (int j = 0; j < 5; j++) {
            cis::Point toAdd = {range_distribution(generator), range_distribution(generator), z_distribution(generator)};
            pc3.add_point(toAdd);
        }

        //For each of the test point clouds, check to confirm that the expected closest point is determined
        //pc1 : in the form (1,y,z)
        for (size_t i = 0; i < pc1.size(); i++ ) {
            auto c = s.root()->find_closest_point(pc1.at(i));
            CHECK(c(0) == 1);
            c(0) = pc1.at(i)(0);
            CHECK(c.isApprox(pc1.at(i)));
        }

        //pc2: in the form (x,1,z)
        for (size_t i = 0; i < pc2.size(); i++ ) {
            auto c = s.root()->find_closest_point(pc2.at(i));
            CHECK(c(1) == 1);
            c(1) = pc2.at(i)(1);
            CHECK(c.isApprox(pc2.at(i)));
        }

        //pc3: in the form (1,1,z)
        for (size_t i = 0; i < pc3.size(); i++ ) {
            auto c = s.root()->find_closest_point(pc3.at(i));
            CHECK(c(0) == 1);
            CHECK(c(1) == 1);
            c(0) = pc3.at(i)(0);
            c(1) = pc3.at(i)(1);
            CHECK(c.isApprox(pc3.at(i)));
        }


    }

    SUBCASE("Ordering") {
        std::vector<std::shared_ptr<cis::Surface::Division>> q = {s.root()};
        while (q.size()) {
            auto c = q.back(); q.pop_back();
            if (c->left()) {
                CHECK(c->left()->value()(c->split_plane()) <= c->value()(c->split_plane()));
                q.push_back(c->left());
            }
            if (c->right()) {
                CHECK(c->right()->value()(c->split_plane()) > c->value()(c->split_plane()));
                q.push_back(c->right());
            }
        }

    }


    remove(tmpfile.c_str());
}