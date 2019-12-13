//
// Created by font_al on 10/8/18.
//

#ifndef TEST_FUNCTIONS_H
#define TEST_FUNCTIONS_H

#include "utils/SystemSettings.h"

namespace fv {

#ifdef LOGFILES
    void setLogFiles();
    void closeLogFiles();
#endif

#ifdef PROFILING
    struct functionProfiling {

        std::string Name{};
        std::chrono::high_resolution_clock::time_point t1{};
        std::chrono::high_resolution_clock::time_point t2{};
        std::chrono::duration<double, std::milli> timeSpan{0};

        double totalTime{};
        std::vector<double> times{};

        size_t numberOfExecutions{0};

        explicit functionProfiling(const std::string functionName) : Name{functionName} {};

        void functionBegin() {
            t1 = std::chrono::high_resolution_clock::now();
            ++numberOfExecutions;
        }

        void functionEnd() {
            t2 = std::chrono::high_resolution_clock::now();
            typedef std::chrono::duration<int, std::milli> milliseconds_type;
            timeSpan = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            times.push_back(timeSpan.count());
            totalTime += timeSpan.count();
        }

        void functionProfile() {

            double meanTime = totalTime / numberOfExecutions;
            double stdev = 0;
            double max = times[0];
            double min = times[0];

            for (size_t i{}; i < numberOfExecutions; i++) {
                stdev += ((times[i] - meanTime) * (times[i] - meanTime));
                if (times[i] > max) {
                    max = times[i];
                }
                if (times[i] < min) {
                    min = times[i];
                }
            }
            stdev /= (numberOfExecutions - 1);
            stdev = sqrt(stdev);

            std::cout << "\nProfiling of function = " << Name << std::endl;
            std::cout << "Total Time              = " << totalTime << " ms" << std::endl;
            std::cout << "Number of Executions    = " << numberOfExecutions << std::endl;
            std::cout << "Mean Time               = " << meanTime << " ms" << std::endl;
            std::cout << "Standard Deviation Time = " << stdev << " ms" << std::endl;
            std::cout << "Min Time                = " << min << " ms" << std::endl;
            std::cout << "Max Time                = " << max << " ms\n" << std::endl;

        }
    };
#endif

    void debugPoint(int i);

//    void variableAnalysis(std::vector<DATA_TYPE> var, std::string Name, std::string Unit);

    //void extract_points_thresh_debug(int value, void *userdata);
    //void extract_points_d_debug(int value, void *userdata);
    //void extract_points_trackbar(fv::Frame frame);

    void waitFunction(const std::string& expectedInput);

}

#endif //TEST_FUNCTIONS_H
