/**
 * @author Ravi Gaddipati, Doran Walsten
 * @date Nov 16, 2016
 * rgaddip1@jhu.edu
 *
 * @brief
 * Data file readers.
 *
 * @file
 */

#ifndef CIS_ICP_FILES_H
#define CIS_ICP_FILES_H

#include <string>
#include <fstream>
#include "pointcloud.h"


namespace cis {

    /**
     * @brief
     * Abstract class that provides base data structure and coord parser.
     */
    class File {
    public:

        virtual void open(std::string file) {
            std::ifstream in(file);
            if (!in.good()) throw std::invalid_argument("Invalid file: " + file);
            open(in);
        }

        virtual void open(std::istream &) = 0;

        /**
         * @return Number of frames if data exists, else 0
         */
        virtual size_t size() const  {
            return _clouds.size() > 0 ? _clouds.at(0).size() : 0;
        };

        virtual std::string name() const {
            return _name;
        };

        virtual const std::vector<PointCloud> &get(const size_t i) const {
            return _clouds.at(i);
        }

    protected:
        std::string _name;
        std::vector<std::vector<PointCloud>>
                _clouds;

        /**
        * @brief
        * Parses the n next lines of the stream.
        * @param in input stream
        * @param n number of lines to parse
        * @param target Target to add points to.
        */
        char _parse_coordinates(std::istream &in, size_t n, PointCloud &target, const char delim);

    };

    /**
     * @brief
     * Represents a Calibration body.
     */
    class RigidBody : public File {
    public:
        using File::open;

        RigidBody() {}

        /**
         * @param file Open the given file
         */
        RigidBody(const std::string file) {
            this->open(file);
        }

        /**
         * @param in Parse data from the stream
         */
        void open(std::istream &in) override;

        /**
         * @return LED markers on the rigid body
         */
        const PointCloud &markers() const {
            return this->_clouds.at(0).at(0);
        }

        cis::Point tip() const {
            return _tip;
        }

    private:
        cis::Point _tip;

    };

    /**
     * Loads a file defining a surgace.
     */
    class SurfaceFile : public File {
    public:
        using File::open;

        SurfaceFile() {}
        SurfaceFile(const std::string file) {
            this->open(file);
        }

        void open(std::istream &in) override;

        /**
         * @return PointCloud of all the vertices.
         */
        const PointCloud &vertices() const {
            return this->_clouds.at(0).at(0);
        }

        /**
         * @return Array providing the indicies of each vertex for each triangle.
         */
        const Eigen::Array<long, Eigen::Dynamic, 3> &triangles() const {
            return this->_tri;
        }

        /**
         * @return All the triangles, where each column is XYZ of each vertex.
         */
        Eigen::Array<double, 9, Eigen::Dynamic> cat_triangles() const;

        /**
         * @return Indicies of neighboring triangles.
         */
        const Eigen::Array<long, Eigen::Dynamic, 3> &neighbor_triangles() const {
            return this->_neighbor;
        }


    private:
        Eigen::Array<long, Eigen::Dynamic, 3> _tri, _neighbor;

    };

    class SampleReadings : public File {
    public:
        using File::open;

        SampleReadings() {}

        /**
         * @param file Open the given sample readings with N_a points in body A and N_b in body B
         */
        SampleReadings(std::string file, size_t N_a, size_t N_b) {
            this->N_a = N_a;
            this->N_b = N_b;
            open(file);
        }

        /**
         * @param in Parse the input stream
         */
        void open(std::istream &in);

        /**
         * @return Frames of LED markers on the pointer
         */
        const std::vector<PointCloud> &pointer_rigid_body() const {
            return this->_clouds[0];
        }

        /**
         * @return Frams of LED markers on the fixed rigid body in the bone
         */
        const std::vector<PointCloud> &fixed_rigid_body() const {
            return this->_clouds[1];
        }

    private:
        size_t N_a;
        size_t N_b;
    };
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


#endif //CIS_ICP_FILES_H