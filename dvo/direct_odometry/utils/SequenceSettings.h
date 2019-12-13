//
// Created by font_al on 4/6/19.
//

#ifndef SEQUENCESETTINGS_H
#define SEQUENCESETTINGS_H

#include "definesAndIncludes.h"

namespace fv {

    cv::FileStorage openFileStorage(const std::string& pathYamlFile);

    #ifdef GT
    struct gt_unit{

        long long timestamp{0};
        size_t id{0};
        size_t idDepth{0};
        bool validDepth{false};
        bool validPose{false};
        DATA_TYPE t_cw[3]{};
        DATA_TYPE R_cw[9]{};
        DATA_TYPE t_wc[3]{};
        DATA_TYPE R_wc[9]{};
    };

    #endif

    class SequenceSettings {

    public:

        std::string name{};
        std::string path{};
        DATA_TYPE RGB_frequency{};
        DATA_TYPE depth_frequency{};
        std::string camera_yamlFile{};

        DATA_TYPE depthConstant{};
        DATA_TYPE timeUnitToSeconds{1};

        SequenceSettings() = default;
        void initialize(std::string& pathDataset);


        #ifdef DATASET
        size_t numRGB{};
        size_t numDepth{};
        size_t firstRGB{};
        size_t lastRGB{};

        std::vector<std::string> imageList;
        std::vector<long long> imageList_timestamp;

        std::vector<std::string> depthList;
        std::vector<long long> depthList_timestamp;



        void createFileList(const std::string &listPath, std::vector<std::string> &fileList);
        void createImageListRGBD(std::vector<std::string> filelist,std::vector<std::string> &imageList,std::vector<long long> &imageList_timestamp); // Create one list for the image names and other for their timestamps
        bool isImage(const std::string &fname_);
        std::vector<std::string> read_csv(std::string path_, int col);
        #ifdef GT
        std::vector<gt_unit> gt;
        std::vector<std::string> gtList;
        std::vector<gt_unit> gt_read_RGBD(std::vector<std::string> gt_list, std::vector<long long> imageList_timestamp);
        #endif
        #endif

        #ifdef ROS
        std::string RGB_topic;
        std::string depth_topic;
        #endif
    };
}

extern fv::SequenceSettings seqSet;

#endif //SEQUENCESETTINGS_H
