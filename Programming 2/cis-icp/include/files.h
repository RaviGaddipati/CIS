/**
 * @author Ravi Gaddipati
 * @date Nov 2, 2016
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
         * @return Optical markers on the EM base
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

    class BodySurface : public File {
    public:
        using File::open;
        BodySurface() {}
        BodySurface(const std::string file) {
            this->open(file);
        }

        void open(std::istream &in) override;

        const PointCloud &vertices() const {
            return this->_clouds.at(0).at(0);
        }

        const PointCloud &triangles() const {
            return this->_clouds.at(1).at(0);
        }

        const PointCloud &neighbor_triangles() const {
            return this->_clouds.at(2).at(0);
        }
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
          << "-1 0 1 1 1 1\n"
          << "-2 0 2 2 2 2\n";
    }

            SUBCASE("File wrapper") {
        cis::BodySurface rb(tmpfile);
        const std::vector<cis::Point> pts = {{-1, 0, 1}, {-2, 0, 2}, {2,2,2}};
                CHECK(rb.vertices().at(0) == pts.at(0));
                CHECK(rb.triangles().at(rb.triangles().size() - 1) == pts.at(1));
                CHECK(rb.neighbor_triangles().at(1) == pts.at(2));
    }

    remove(tmpfile.c_str());
}

#endif //CIS_ICP_FILES_H