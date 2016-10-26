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
#include <doctest.h>
#include "utils.h"
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
        void _parse_coordinates(std::istream &in, size_t n, PointCloud &target);

    };

    /**
     * @brief
     * Represents a Calibration body.
     */
    class CalBody : public File {
    public:
        using File::open;

        CalBody() {}

        /**
         * @param file Open the given file
         */
        CalBody(std::string file) {
            this->open(file);
        }

        /**
         * @param in Parse data from the stream
         */
        void open(std::istream &in);

        /**
         * @return Optical markers on the EM base
         */
        const PointCloud &opt_marker_embase() const {
            return this->_clouds[0][0];
        }

        /**
         * @return Optical markers on the calibration object
         */
        const PointCloud &opt_marker_calobj() const {
            return this->_clouds[1][0];
        }

        /**
         * @return EM markers on the calibration object
         */
        const PointCloud &em_marker_calobj() const {
            return this->_clouds[2][0];
        }

    };

    /**
     * @brief
     * Represents point clouds through multiple frames
     */
    class CalReadings : public File {
    public:
        using File::open;

        CalReadings() {}

        /**
         * @param file Open the given file.
         */
        CalReadings(std::string file) {
            open(file);
        }

        /**
         * @param in Parse the data stream
         */
        void open(std::istream &in);

        /**
         * @return  Frames of Optical markers on the EM base
         */
        const std::vector<PointCloud> &opt_marker_embase() const {
            return this->_clouds[0];
        }

        /**
         * @return  Frames of Optical markers on the Calibration Object
         */
        const std::vector<PointCloud> &opt_marker_calobj() const {
            return this->_clouds[1];
        }

        /**
         * @return  Frames of EM markers on the calibration object
         */
        const std::vector<PointCloud> &em_marker_calobj() const {
            return this->_clouds[2];
        }
    };

    /**
     * @brief
     * Represents data frames from a pivot calibration procedure, EM markers.
     */

    class EMPivot : public File {
    public:
        using File::open;

        EMPivot() {}

        /**
         * @param file Open the given file
         */
        EMPivot(std::string file) {
            open(file);
        }

        /**
         * @param in Parse the input stream
         */
        void open(std::istream &in);

        /**
         * @return Frames of EM markers on the probe.
         */
        const std::vector<PointCloud> &em_marker_probe() const {
            return this->_clouds[0];
        }
    };

    /**
     * @brief
     * Represents data frames from a pivot calibration procedure, Optical markers.
     */
    class OptPivot : public File {
        using File::open;
    public:
        OptPivot() {}

        /**
         * @param file Open the givne file
         */
        OptPivot(std::string file) {
            open(file);
        }

        /**
         * @param in Parse the input stream.
         */
        void open(std::istream &in);

        /**
         * @return vector of frames of optical markers on the EM base.
         */
        const std::vector<PointCloud> &opt_marker_embase() const {
            return this->_clouds.at(0);
        }

        /**
         * @return vector of frames of optical markers on the probe.
         */
        const std::vector<PointCloud> &opt_marker_probe() const {
            return this->_clouds.at(1);
        }
    };

    /**
     * PA2 CT Fiducial file parser.
     */
    class CTFiducials : public File {
        using File::open;
    public:
        CTFiducials() {}

        CTFiducials(std::string filename) {
            open(filename);
        }

        /**
         * @param in Parse the input stream.
         */
        void open(std::istream &in);

        const PointCloud &CT_fiducials() const {
            return this->_clouds.at(0).at(0);
        }

    };

    /**
     * PA 2 EM Fiducials.
     */
    class EMFiducials : public File {
        using File::open;
    public:
        EMFiducials() {}

        EMFiducials(std::string file) {
            open(file);
        }

        /**
         * @param in Parse the input stream.
         */
        void open(std::istream &in);

        const std::vector<PointCloud> &EM_fiducials() const {
            return this->_clouds.at(0);
        }
    };


    /**
     * PA2 EM markers on the probe.
     */
    class EMNav : public File {
        using File::open;
    public:
        EMNav() {}
        EMNav(std::string file) {
            open(file);
        }

        /**
         * @param in Parse the input stream.
         */
        void open(std::istream &in);

        const std::vector<PointCloud> &em_markers_probe() const {
            return this->_clouds[0];
        }
    };

    /**
     * @brief
     * Parses an output file, primarily to compute error.
     */
    class OutputPraser : public File {
    public:
        using File::open;
        OutputPraser() {}
        OutputPraser(std::string file) {open(file);}

        void open(std::istream &in);

        /**
         * @return Expected points
         */
        const std::vector<PointCloud> &expected() const {
            return this->_clouds[0];
        }

        /**
         * @return Reported Post position from EM Tracker
         */
        const Point &em_post() const {
            return _em_post;
        }

        /**
         * @return Reported Post position from Optical
         */
        const Point &opt_post() const {
            return _opt_post;
        }

    private:
        Point _em_post, _opt_post;
    };

    /**
     * @brief
     * Parses the output file for PA 2.
     */
    class OutputParser2 {
    public:
        OutputParser2 () {}

        OutputParser2 (std::string file) {
            open(file);
        }

        void open(std::string file) {
            std::ifstream in(file);
            if (!in.good()) throw std::invalid_argument("Error opening file: " + file);
            open(in);
        }

        void open(std::istream &in);

        /**
         * Probe tip in each frame.
         * @return vector of points
         */
        const std::vector<Point> &probe_tip() const {
            return _points;
        }

        /**
         * @return Return file name.
         */
        std::string name() const {
            return _name;
        }

    private:
        std::vector<Point> _points;
        std::string _name;

    };

    /**
     * @param out Output stream to write to
     * @param file_name label of output file
     * @param data Vector of frames of PointClouds to write
     * @param em_post position of the post found by EM Pivot
     * @param opt_post position of the post found by Optical pivot
     */
    void output_writer(std::ostream &out,
                       const std::string file_name,
                       const std::vector<PointCloud> &data,
                       const Eigen::Matrix<double, 3, 1> &em_post,
                       const Eigen::Matrix<double, 3, 1> &opt_post);

    /**
 * @param file_name Write the data to this file
 * @param data Vector of frames of PointClouds to write
 * @param em_post position of the post found by EM Pivot
 * @param opt_post position of the post found by Optical pivot
 */
    void output_writer(const std::string file_name,
                       const std::vector<PointCloud> &data,
                       const Eigen::Matrix<double, 3, 1> &em_post,
                       const Eigen::Matrix<double, 3, 1> &opt_post);

    /**
     * Output writer for PA 2.
     * @param os output stream
     * @param name filename
     * @param frames of data
     */
    void output_writer(std::ostream &os, std::string name, const std::vector<cis::Point> &frames);
    void ouput_writer(std::string filename, const std::vector<cis::Point> &frames);
}

/*************** TEST CASES ***************/

TEST_CASE ("Calibration Body file") {
    using namespace cis;
    const std::string tmp_file("_cis_tmp_file");
    std::ofstream t(tmp_file);
    t << "1,2,\t3, NAME\n1,1 ,1\n2,2,2\n3,3,3\n4,4,4\n5,5,5\n6,6,6";
    t.close();

    cis::CalBody cb{tmp_file};
            CHECK(cb.opt_marker_embase().size() == 1);
            CHECK(cb.opt_marker_calobj().size() == 2);
            CHECK(cb.em_marker_calobj().size() == 3);
            CHECK(cb.name() == "NAME");

    const auto &d = cb.opt_marker_embase();
    const auto &a = cb.opt_marker_calobj();
    const auto &c = cb.em_marker_calobj();

    Point p;

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

TEST_CASE ("Calibration Readings file") {
    using namespace cis;
    const std::string tmp_file("_cis_tmp_file");
    std::ofstream t(tmp_file);
    t << "1,2,\t3,2, NAME\n1,1 ,1\n2,2,2\n3,3,3\n4,4,4\n5,5,5\n6,6,6\n1,1 ,1\n2,2,2\n3,3,3\n4,4,4\n5,5,5\n6,6,6";
    t.close();

    CalReadings cb(tmp_file);
            CHECK(cb.size() == 2);

            CHECK(cb.name() == "NAME");

    auto d = cb.opt_marker_embase().at(0);
    auto a = cb.opt_marker_calobj().at(0);
    auto c = cb.em_marker_calobj().at(0);

    Point p;

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

    d = cb.opt_marker_embase().at(1);
    a = cb.opt_marker_calobj().at(1);
    c = cb.em_marker_calobj().at(1);
            CHECK_THROWS(cb.opt_marker_embase().at(2));
            CHECK_THROWS(cb.opt_marker_calobj().at(2));
            CHECK_THROWS(cb.em_marker_calobj().at(2));

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

TEST_CASE ("EM Pivot File") {
    using namespace cis;
    const std::string tmp_file("_cis_tmp_file");
    std::ofstream t(tmp_file);
    t << "2,\t2, NAME\n1,1 ,1\n2,2,2\n3,3,3\n4,4,4\n";
    t.close();

    EMPivot emp(tmp_file);
            CHECK(emp.name() == "NAME");

            REQUIRE(emp.em_marker_probe().size() == 2);
    auto ng = emp.em_marker_probe()[0];
    Point p;
            CHECK(ng.size() == 2);
    p = {1, 1, 1};
            CHECK(ng.at(0) == p);
    p = {2, 2, 2};
            CHECK(ng.at(1) == p);
            CHECK_THROWS(ng.at(2));

    ng = emp.em_marker_probe()[1];
            CHECK(ng.size() == 2);
    p = {3, 3, 3};
            CHECK(ng.at(0) == p);
    p = {4, 4, 4};
            CHECK(ng.at(1) == p);
            CHECK_THROWS(ng.at(2));

    remove(tmp_file.c_str());
}

TEST_CASE ("Optical Pivot File") {
    using namespace cis;
    const std::string tmp_file("_cis_tmp_file");
    std::ofstream t(tmp_file);
    t << "2, 1,\t2, NAME\n1,1 ,1\n2,2,2\n3,3,3\n4,4,4\n1,1 ,1\n2,2,2\n";
    t.close();

    OptPivot op(tmp_file);
            CHECK(op.name() == "NAME");

            REQUIRE(op.opt_marker_embase().size() == 2);
            REQUIRE(op.opt_marker_probe().size() == 2);

            REQUIRE(op.opt_marker_embase()[0].size() == 2);
            REQUIRE(op.opt_marker_probe()[0].size() == 1);

    Point p;

    auto ng = op.opt_marker_embase()[0];

            CHECK(ng.size() == 2);
    p = {1, 1, 1};
            CHECK(ng.at(0) == p);
    p = {2, 2, 2};
            CHECK(ng.at(1) == p);
            CHECK_THROWS(ng.at(2));

    ng = op.opt_marker_embase()[1];

            CHECK(ng.size() == 2);
    p = {4, 4, 4};
            CHECK(ng.at(0) == p);
    p = {1, 1, 1};
            CHECK(ng.at(1) == p);
            CHECK_THROWS(ng.at(2));

    ng = op.opt_marker_probe()[0];

            CHECK(ng.size() == 1);
    p = {3, 3, 3};
            CHECK(ng.at(0) == p);
            CHECK_THROWS(ng.at(1));

    ng = op.opt_marker_probe()[1];

            CHECK(ng.size() == 1);
    p = {2, 2, 2};
            CHECK(ng.at(0) == p);
            CHECK_THROWS(ng.at(1));

    remove(tmp_file.c_str());
}

TEST_CASE ("Output Writer") {
    std::stringstream dest;
    cis::PointCloud pc;
    pc.add_point({0,0,0});
    pc.add_point({1,1,1});
    pc.add_point({2,2,2});
    std::vector<cis::PointCloud> frames = {pc, pc};
    Eigen::Matrix<double,3,1> pt = {1,2,3};
    cis::output_writer(dest, "out.txt", frames, pt, pt);

    CHECK((dest.str()) == "3,2,out.txt\n1,2,3\n1,2,3\n0,0,0\n1,1,1\n2,2,2\n0,0,0\n1,1,1\n2,2,2\n");
}

#endif //CIS_CAL_FILES_H
