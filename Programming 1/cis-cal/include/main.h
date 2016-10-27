//
// Created by gaddra on 10/26/16.
//

#ifndef CIS_CAL_MAIN_H
#define CIS_CAL_MAIN_H

/**
 * CLI Usage information.
 */
void printusage();

/**
 * @brief
 * Prints the error between the post positions and the expected values
 * of two output files.
 * @param f1 File 1
 * @param f2 File 2
 */
void error_report(const std::string &f1, const std::string &f2, bool frame_error=false);

/**
 * @brief
 * Prints the error for PA 2
 * @param f1 File 1
 * @param f2 File 2
 */
void error_report_2(const std::string &f1, const std::string &f2);


#endif //CIS_CAL_MAIN_H
