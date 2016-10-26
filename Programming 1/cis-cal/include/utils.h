//
// Created by gaddra on 10/12/16.
//

#ifndef CIS_CAL_UTILS_H
#define CIS_CAL_UTILS_H

#ifdef __GNUC__
#define LIKELY(x) __builtin_expect((x),1)
#define UNLIKELY(x) __builtin_expect((x),0)
#define __UNROLL__ __attribute__((optimize("unroll-loops")))
#define __INLINE__ __attribute__((always_inline)) inline
#else
#define LIKELY(x) (x)
#define UNLIKELY(x) (x)
#define __UNROLL__
#define __INLINE__ inline
#endif

#include <vector>
#include <fstream>
#include <algorithm>
#include <sstream>
#include "doctest.h"
#include "pointcloud.h"
#include "Eigen"


/**
 * @brief
 * Splits a string into a vector given some character delimiter.
 * @param s string to split
 * @param delim split string at delim, discarding the delim
 * @return vector to store results in
 */
std::vector<std::string> split(const std::string &s, char delim);

/**
 * @brief
 * Splits a string into a vector given some character delimiter.
 * @param s string to split
 * @param delim split string at delim, discarding the delim
 * @param vec vector to store results in
 */
void split(const std::string &s,
           char delim, std::vector<std::string> &vec);


/**
 * @brief
 * Opens a file and checks if its valid.
 * @param filename File to check if valid.
 */
inline bool file_exists(std::string filename) {
    std::ifstream f(filename);
    return f.good();
}

/**
 * Compute n choose k to get the number of possible combinations. Inspired by work completed on Stack Overflow:
 * http://stackoverflow.com/questions/9330915/number-of-combinations-n-choose-r-in-c
 * @param n - size of collection
 * @param k - number of entries to select
 * @return The number of potential combinations
 */
int nChoosek(int n, int k);

/**
 * @brief
 * Prints a point as a CSV line.
 * @param os Stream to print to
 * @param p Point
 * @return os
 */
template <typename T>
std::ostream &print_point(std::ostream &os, const Eigen::Matrix<T, 3, 1> &p) {
    os << p(0) << ',' << p(1) << ',' << p(2);
    return os;
}

/**
 * @return Min coeffs of all frames.
 */
template <typename T>
const typename cis::PointCloud<T>::Point min(const std::vector<cis::PointCloud<T>> &cloud);

/**
 * @return Max coeffs of all frames.
 */
template <typename T>
const typename cis::PointCloud<T>::Point max(const std::vector<cis::PointCloud<T>> &cloud);

TEST_CASE("Min and Max points") {
    cis::PointCloud<double> pc1{{{1, -2, 3}, {0, 1, .5}, {0, 0, 1}, {-1, 0, 0}, {3, 8, 7}}};
    cis::PointCloud<double> pc2{{{0, 2, 3}, {0, 1, .5}, {0, 0, 1}, {-100, 0, 0}, {3, 8, 7}}};
    std::vector<cis::PointCloud<double>> frames = {pc1, pc2};
    const auto _min = min(frames);
    const auto _max = max(frames);
    typename cis::PointCloud<double>::Point p{-100,-2,0};
    CHECK(_min == p);
    p = {3,8,7};
    CHECK(_max == p);
}

#endif //CIS_CAL_UTILS_H
