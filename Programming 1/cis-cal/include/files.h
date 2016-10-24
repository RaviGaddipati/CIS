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


namespace cis {

    /**
     * @brief
     * Abstract class that provides base data structure and coord parser.
     */
    template<typename T>
    class File {
    public:

        virtual void open(std::string file) {
            std::ifstream in(file);
            if (!in.good()) throw std::invalid_argument("Invalid file: " + file);
            open(in);
        }

        virtual void open(std::istream &) = 0;

        virtual size_t size() {
            return _clouds.size() > 0 ? _clouds[0].size() : 0;
        };

        virtual std::string name() const {
            return _name;
        };
        virtual const std::vector<PointCloud < T>> &

        get(const size_t i) const {
            return _clouds.at(i);
        }

    protected:
        std::string _name;
        std::vector<std::vector<PointCloud < T>>>
        _clouds;

        /**
        * @brief
        * Parses the n next lines of the stream.
        * @param in input stream
        * @param n number of lines to parse
        * @param target Target to add points to.
        */
        void _parse_coordinates(std::istream &in, size_t n, PointCloud <T> &target) {
            std::string line;
            std::vector<std::string> split_line;
            std::vector<typename PointCloud<T>::Point> points;
            for (size_t i = 0; i < n; ++i) {
                if (!std::getline(in, line)) throw std::invalid_argument("Unexpected end of file.");
                line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
                split(line, ',', split_line);
                if (split_line.size() != 3) throw std::invalid_argument("Expected 3 values in line: " + line);
                points.emplace_back(std::stod(split_line[0]), std::stod(split_line[1]), std::stod(split_line[2]));
            }
            target.add_points(points);
        }

    };

    /**
     * @brief
     * Represents a Calibration body.
     */
    template<typename T>
    class CalBody : public File<T> {
    public:
        using File<T>::open;

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
        void open(std::istream &in) {
            // Parse meta info
            std::string line;
            std::getline(in, line);
            line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
            auto line_split = split(line, ',');

            size_t nd, na, nc;
            try {
                nd = std::stoul(line_split[0]);
                na = std::stoul(line_split[1]);
                nc = std::stoul(line_split[2]);
                this->_name = line_split[3];
            } catch (std::exception &e) {
                std::cerr << "Error parsing meta info line, expected 4 fields. \n" << line << std::endl;
                throw;
            }

            this->_clouds.resize(3);
            for (auto &c : this->_clouds) c.resize(1);
            this->_parse_coordinates(in, nd, this->_clouds[0][0]);
            this->_parse_coordinates(in, na, this->_clouds[1][0]);
            this->_parse_coordinates(in, nc, this->_clouds[2][0]);
        }

        /**
         * @return Optical markers on the EM base
         */
        const PointCloud <T> &opt_marker_embase() const {
            return this->_clouds[0][0];
        }

        /**
         * @return Optical markers on the calibration object
         */
        const PointCloud <T> &opt_marker_calobj() const {
            return this->_clouds[1][0];
        }

        /**
         * @return EM markers on the calibration object
         */
        const PointCloud <T> &em_marker_calobj() const {
            return this->_clouds[2][0];
        }

    };

    /**
     * @brief
     * Represents point clouds through multiple frames
     */
    template<typename T>
    class CalReadings : public File<T> {
    public:
        using File<T>::open;

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
        void open(std::istream &in) {
            // Parse meta info
            std::string line;
            std::getline(in, line);
            line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
            auto line_split = split(line, ',');

            size_t nd, na, nc, nf;
            try {
                nd = std::stoul(line_split[0]);
                na = std::stoul(line_split[1]);
                nc = std::stoul(line_split[2]);
                nf = std::stoul(line_split[3]);
                this->_name = line_split[4];
            } catch (std::exception &e) {
                std::cerr << "Error parsing meta info line, expected 5 fields. \n" << line << std::endl;
                throw;
            }

            this->_clouds.resize(3);
            for (auto &c : this->_clouds) c.resize(nf);
            for (size_t frame = 0; frame < nf; ++frame) {
                this->_parse_coordinates(in, nd, this->_clouds[0][frame]);
                this->_parse_coordinates(in, na, this->_clouds[1][frame]);
                this->_parse_coordinates(in, nc, this->_clouds[2][frame]);
            }
        }

        /**
         * @return  Frames of Optical markers on the EM base
         */
        const std::vector<PointCloud < T>> &

        opt_marker_embase() const {
            return this->_clouds[0];
        }

        /**
         * @return  Frames of Optical markers on the Calibration Object
         */
        const std::vector<PointCloud < T>> &

        opt_marker_calobj() const {
            return this->_clouds[1];
        }

        /**
         * @return  Frames of EM markers on the calibration object
         */
        const std::vector<PointCloud < T>> &

        em_marker_calobj() const {
            return this->_clouds[2];
        }
    };

    /**
     * @brief
     * Represents data frames from a pivot calibration procedure, EM markers.
     */
    template<typename T>
    class EMPivot : public File<T> {
    public:
        using File<T>::open;

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
        void open(std::istream &in) {
            // Parse meta info
            std::string line;
            std::getline(in, line);
            line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
            auto line_split = split(line, ',');

            size_t ng, nf;
            try {
                ng = std::stoul(line_split[0]);
                nf = std::stoul(line_split[1]);
                this->_name = line_split[2];
            } catch (std::exception &e) {
                std::cerr << "Error parsing meta info line, expected 3 fields. \n" << line << std::endl;
                throw;
            }

            this->_clouds.resize(1);
            for (auto &c : this->_clouds) c.resize(nf);
            for (size_t frame = 0; frame < nf; ++frame) {
                this->_parse_coordinates(in, ng, this->_clouds[0][frame]);
            }
        }

        /**
         * @return Frames of EM markers on the probe.
         */
        const std::vector<PointCloud < T>> &

        em_marker_probe() const {
            return this->_clouds[0];
        }
    };

    /**
     * @brief
     * Represents data frames from a pivot calibration procedure, Optical markers.
     */
    template<typename T>
    class OptPivot : public File<T> {
        using File<T>::open;
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
        void open(std::istream &in) {
            // Parse meta info
            std::string line;
            std::getline(in, line);
            line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
            auto line_split = split(line, ',');

            size_t nd, nh, nf;
            try {
                nd = std::stoul(line_split[0]);
                nh = std::stoul(line_split[1]);
                nf = std::stoul(line_split[2]);
                this->_name = line_split[3];
            } catch (std::exception &e) {
                std::cerr << "Error parsing meta info line, expected 4 fields. \n" << line << std::endl;
                throw;
            }

            this->_clouds.resize(2);
            for (auto &c : this->_clouds) c.resize(nf);
            for (size_t frame = 0; frame < nf; ++frame) {
                this->_parse_coordinates(in, nd, this->_clouds[0][frame]);
                this->_parse_coordinates(in, nh, this->_clouds[1][frame]);
            }
        }

        /**
         * @return vector of frames of optical markers on the EM base.
         */
        const std::vector<PointCloud < T>> &

        opt_marker_embase() const {
            return this->_clouds[0];
        }

        /**
         * @return vector of frames of optical markers on the probe.
         */
        const std::vector<PointCloud < T>> &

        opt_marker_probe() const {
            return this->_clouds[1];
        }
    };

    /**
     * @param file_name Write the data to this file
     * @param data Vector of frames of PointClouds to write
     * @param em_post position of the post found by EM Pivot
     * @param opt_post position of the post found by Optical pivot
     */
    template<typename T>
    void output_writer(const std::string file_name,
                       const std::vector<PointCloud < T>>

    &data,
    const Eigen::Matrix<T, 3, 1> &em_post,
    const Eigen::Matrix<T, 3, 1> &opt_post
    ) {
    std::ofstream out(file_name);
    if (!out.

    good()

    ) throw std::invalid_argument("Invalid output file: " + file_name);
    output_writer(out, file_name, data, em_post, opt_post
    );
}

/**
 * @param out Output stream to write to
 * @param file_name label of output file
 * @param data Vector of frames of PointClouds to write
 * @param em_post position of the post found by EM Pivot
 * @param opt_post position of the post found by Optical pivot
 */
template<typename T>
void output_writer(std::ostream &out,
                   const std::string file_name,
                   const std::vector<PointCloud < T>>

&data,
const Eigen::Matrix<T, 3, 1> &em_post,
const Eigen::Matrix<T, 3, 1> &opt_post
) {
if (data.

size()

< 1) throw std::invalid_argument("Invalid data vector.");

out << data.at(0).

size()

<< ',' << data.

size()

<< ',' << file_name << '\n'
<< em_post(0) << ',' << em_post(1) << ',' << em_post(2) << '\n'
<< opt_post(0) << ',' << opt_post(1) << ',' << opt_post(2) << '\n';

for (
const auto &frame :
data) {
for (
size_t i = 0;
i<frame.

size();

++i) {
out << frame.
at(i)(0)
<< ',' << frame.
at(i)(1)
<< ',' << frame.
at(i)(2)
<< '\n';
}
}
out <<
std::flush;
}
}

/*************** TEST CASES ***************/

TEST_CASE ("Calibration Body file") {
    using namespace cis;
    const std::string tmp_file("_cis_tmp_file");
    std::ofstream t(tmp_file);
    t << "1,2,\t3, NAME\n1,1 ,1\n2,2,2\n3,3,3\n4,4,4\n5,5,5\n6,6,6";
    t.close();

    cis::CalBody<double> cb{tmp_file};
            CHECK(cb.opt_marker_embase().size() == 1);
            CHECK(cb.opt_marker_calobj().size() == 2);
            CHECK(cb.em_marker_calobj().size() == 3);
            CHECK(cb.name() == "NAME");

    const auto &d = cb.opt_marker_embase();
    const auto &a = cb.opt_marker_calobj();
    const auto &c = cb.em_marker_calobj();

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

TEST_CASE ("Calibration Readings file") {
    using namespace cis;
    const std::string tmp_file("_cis_tmp_file");
    std::ofstream t(tmp_file);
    t << "1,2,\t3,2, NAME\n1,1 ,1\n2,2,2\n3,3,3\n4,4,4\n5,5,5\n6,6,6\n1,1 ,1\n2,2,2\n3,3,3\n4,4,4\n5,5,5\n6,6,6";
    t.close();

    CalReadings<double> cb(tmp_file);
            CHECK(cb.size() == 2);

            CHECK(cb.name() == "NAME");

    auto d = cb.opt_marker_embase().at(0);
    auto a = cb.opt_marker_calobj().at(0);
    auto c = cb.em_marker_calobj().at(0);

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

    EMPivot<double> emp(tmp_file);
            CHECK(emp.name() == "NAME");

            REQUIRE(emp.em_marker_probe().size() == 2);
    auto ng = emp.em_marker_probe()[0];
    PointCloud<double>::Point p;
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

    OptPivot<double> op(tmp_file);
            CHECK(op.name() == "NAME");

            REQUIRE(op.opt_marker_embase().size() == 2);
            REQUIRE(op.opt_marker_probe().size() == 2);

            REQUIRE(op.opt_marker_embase()[0].size() == 2);
            REQUIRE(op.opt_marker_probe()[0].size() == 1);

    PointCloud<double>::Point p;

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
    cis::PointCloud<double> pc;
    pc.add_point({0, 0, 0});
    pc.add_point({1, 1, 1});
    pc.add_point({2, 2, 2});
    std::vector<cis::PointCloud<double>> frames = {pc, pc};
    Eigen::Matrix<double, 3, 1> pt = {1, 2, 3};
    cis::output_writer(dest, "out.txt", frames, pt, pt);

            CHECK((dest.str()) == "3,2,out.txt\n1,2,3\n1,2,3\n0,0,0\n1,1,1\n2,2,2\n0,0,0\n1,1,1\n2,2,2\n");
}

#endif //CIS_CAL_FILES_H
