//
// Created by font_al on 10/8/18.
//

#include "test_functions.h"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//LOG FILES
#ifdef LOGFILES
    void fv::setLogFiles(){
        gtLog = new std::ofstream();
//#ifdef naive
        //gtLog->open("/home_local/wang_sn/workspace/dvo/logs/gtLog_naive.txt", std::ios::trunc | std::ios::out);
//#endif
        gtLog->open("/home_local/wang_sn/workspace/dvo/logs/gtLog_g2o.txt", std::ios::trunc | std::ios::out);
        gtLog->precision(12);
        std::cout<<"created logfiles"<<std::endl;

        #ifdef ENTROPY
        entropyLog = new std::ofstream();
        entropyLog->open("logs/entropyLog.txt", std::ios::trunc | std::ios::out);
        entropyLog->precision(12);
        #endif

        #ifdef SYNC_DEPTH_BLOCK_DEBUG
        syncDepthBlockLog = new std::ofstream();
        syncDepthBlockLog->open("logs/syncDepthBlockLog.txt", std::ios::trunc | std::ios::out);
        syncDepthBlockLog->precision(12);
        #endif

        #ifdef CINEMATIC_MODEL_BLOCK_DEBUG
        cinematicModelBlockLog = new std::ofstream();
        cinematicModelBlockLog->open("logs/cinematicModelBlockLog.txt", std::ios::trunc | std::ios::out);
        cinematicModelBlockLog->precision(12);
        #endif

        #ifdef EXCEPTIONS_BLOCK_DEBUG
        exceptionsBlockLog = new std::ofstream();
        exceptionsBlockLog->open("logs/exceptionsBlockLog.txt", std::ios::trunc | std::ios::out);
        exceptionsBlockLog->precision(12);
        #endif
    }

    void fv::closeLogFiles(){
        gtLog->close();
        delete gtLog;

        #ifdef ENTROPY
        entropyLog->close();
        delete entropyLog;
        #endif

        #ifdef SYNC_DEPTH_BLOCK_DEBUG
        syncDepthBlockLog->close();
        delete syncDepthBlockLog;
        #endif

        #ifdef CINEMATIC_MODEL_BLOCK_DEBUG
        cinematicModelBlockLog->close();
        delete cinematicModelBlockLog;
        #endif

        #ifdef EXCEPTIONS_BLOCK_DEBUG
        exceptionsBlockLog->close();
        delete exceptionsBlockLog;
        #endif
    }

#endif
void fv::debugPoint(int i){
    std::cout << "DEBUG POINT: " << i << std::endl;
};

void fv::waitFunction(const std::string& expectedInput){

    std::string input;

    std::cout << "Continue("<< expectedInput << ")?: " << std::endl;
    std::cin >> input;

    while (input != expectedInput) {
        std::cout << "Continue("<< expectedInput << ")?: " << std::endl;
        std::cin >> input;
    }
}
