//
// Created by font_al on 4/6/19.
//

#include "SequenceSettings.h"
#include <boost/algorithm/string.hpp>
fv::SequenceSettings seqSet{};

void fv::SequenceSettings::initialize(std::string& sequenceData_yamlFile) {


    cv::FileStorage fs_sequenceData =  fv::openFileStorage(sequenceData_yamlFile);
    cv::FileNode n;

    #ifdef DATASET
    n = fs_sequenceData["monoRGBDdataset"];
    #endif
    #ifdef ROS
    n = fs_sequenceData["monoRGBDrosbag"];
    #endif

    if(not(n.empty())){
        name  = (std::string)(n["name"]);
        path  = (std::string)(n["path"]);
        depthConstant  = (DATA_TYPE)(n["depthConstant"]); // Depth image conversion (m/depthUnit)
        timeUnitToSeconds = (DATA_TYPE)(n["timeUnitToSeconds"]); // Timestamps conversion to seconds (s/timeUnit)

        RGB_frequency = 1/(((DATA_TYPE)(n["RGB_hz"])*timeUnitToSeconds));
        depth_frequency = 1/(((DATA_TYPE)(n["depth_hz"])*timeUnitToSeconds));

        #ifdef DATASET
            numRGB  = (int) n["numRGB"];
            numDepth = (int) n["numDepth"];
            firstRGB = 0;
            lastRGB  = numRGB - 1;

            #ifdef GT
            firstRGB = (int) n["firstRGB"];
            lastRGB  = (int) n["lastRGB"];
            #endif

            std::vector<std::string> fileList;
            // Reading image addresses and timestamps
            createFileList(path + "rgb", fileList);
            createImageListRGBD(fileList, imageList, imageList_timestamp);
            std::cout << "num RGB found = " << imageList.size() <<" / "<< numRGB << std::endl;
            fileList.clear();

            // Reading depth addresses and timestamps
            createFileList(path + "depth", fileList);
            createImageListRGBD(fileList, depthList, depthList_timestamp);
            std::cout << "num Depth found = " << depthList.size() <<" / "<< numDepth << std::endl;
            fileList.clear();

            #ifdef GT
            // Creating position groundtruth
            gtList = read_csv(path + "gt.txt", 15);
            gt = gt_read_RGBD(gtList, imageList_timestamp);
            #endif
        #endif

        #ifdef ROS
            RGB_topic  = (std::string)(n["RGB_topic"]);
            depth_topic  = (std::string)(n["depth_topic"]);
        #endif

        camera_yamlFile = (std::string) n["camera0YAML"];
    }
}

cv::FileStorage fv::openFileStorage(const std::string& pathYamlFile){

    cv::FileStorage tempFile(pathYamlFile, cv::FileStorage::READ);
    if (!tempFile.isOpened()) {
        std::cerr << "Failed to open " << pathYamlFile << std::endl;
    }
    else{
        std::cout << pathYamlFile << " : FOUND" << std::endl;
    }
    return tempFile;
}

#ifdef DATASET

#ifdef GT
std::vector< fv::gt_unit> fv::SequenceSettings::gt_read_RGBD(std::vector<std::string> gt_list, std::vector<long long> imageList_timestamp){

    std::cout << "\nComputing gt... " << gt_list.size()<< std::endl;

    std::vector< fv::gt_unit> gt;
    std::string s;

    DATA_TYPE var{};
    fv::gt_unit gt_i{};

    for(size_t i{0}; i < imageList_timestamp.size(); ++i) {
        std::stringstream stream0(gt_list[i]);

        std::getline(stream0, s, ' ');
        gt_i.id = size_t(std::stoi(s));

        std::getline(stream0, s, ' ');
        gt_i.validDepth = bool(std::stoi(s));

        std::getline(stream0, s, ' ');
        gt_i.idDepth = size_t(std::stoi(s));

        std::getline(stream0, s, ' ');
        gt_i.validPose = size_t(std::stoi(s));

        std::getline(stream0, s, ' ');
        gt_i.timestamp = imageList_timestamp[i];

        #ifdef GT_DEBUG
        std::cout << "\n\nID = " << gt_i.id << std::endl;
        std::cout << "timestamp = " << gt_i.timestamp << std::endl;;
        std::cout << "t = " ;
        #endif
        for (size_t translationXYZ{}; translationXYZ < 3; ++translationXYZ){
            std::getline(stream0, s, ' ');
            var =  METRIC_UNIT_CONV*(CONVERSION_STRING_TO_DATA_TYPE(s));
            gt_i.t_wc[translationXYZ] = var;
            #ifdef GT_DEBUG
            std::cout << gt_i.t_wc[translationXYZ]<<" ";
            #endif

        }
        #ifdef GT_DEBUG
        std::cout << "\nR = " ;
        #endif

        for (size_t rotation{}; rotation < 9; ++rotation){
            std::getline(stream0, s, ' ');
            var =  METRIC_UNIT_CONV*(CONVERSION_STRING_TO_DATA_TYPE(s));
            gt_i.R_wc[rotation] = var;
            #ifdef GT_DEBUG
            std::cout << gt_i.R_wc[rotation]<<" ";
            #endif

        }

        gt.push_back(gt_i);

    }

    return gt;
}
#endif

void fv::SequenceSettings::createFileList(const std::string &listPath, std::vector<std::string> &fileList) {

    bool wildcard = listPath.find("*") != listPath.find("?");

    std::string listBase = listPath;
    std::string listMatch = "";
    if (wildcard) {
        size_t pos = listPath.find_last_of("/");
        if (pos != std::string::npos) {
            listBase = listPath.substr(0, pos);
            listMatch = listPath.substr(pos + 1);
        } else {
            listBase = ".";
            listMatch = listPath;
        }
    }

    std::string _name = listBase;

    struct stat filestat;
    if (stat(listBase.c_str(), &filestat)) {
    } else if (S_ISDIR(filestat.st_mode)) {
        DIR *dpath = opendir(listBase.c_str());
        if (!dpath) {
        } else {
            while (struct dirent *dentry = readdir(dpath)) {
                if ((dentry->d_name[0] != '.')&& (dentry->d_name[0] != '..')){

                    fileList.push_back(listBase + '/' + dentry->d_name);
                }
            }
        }
    } else {
        std::string line;
        std::ifstream f(listBase.c_str());
    }

    std::sort(fileList.begin(), fileList.end());

}

/*void fv::SequenceSettings::createImageListNUIM(std::vector<std::string> filelist, std::vector<std::string> &_imageList,std::vector<long long> &imageList_timestamp){

    std::string image_timestamp;
    int _numFrames;
    size_t idImage{0};

    for(auto& imgPath : filelist){
        if(isImage(imgPath)){
            //if((idImage >= firstImage)&&(idImage <= lastImage)) {
            _imageList.push_back(imgPath);
            size_t pos = imgPath.find_last_of("/");
            image_timestamp = imgPath.substr(pos + 1);

            pos = image_timestamp.find_last_of(".");
            image_timestamp = image_timestamp.substr(0, pos);

            //////////////////////
            //pos = image_timestamp.find_last_of(".");
            //image_timestamp = image_timestamp.substr(0,pos)+image_timestamp.substr(pos+1);
            //////////////////////////
            imageList_timestamp.push_back(std::stoll(image_timestamp));

            _numFrames++;

            //std::cout << "Image:  " << imgPath << std::endl;
            //std::cout << "Timestamp: " << std::stoll(image_timestamp) << std::endl;
            // }
            ++idImage;
        }
    }
}*/

void fv::SequenceSettings::createImageListRGBD(std::vector<std::string> filelist, std::vector<std::string> &_imageList,std::vector<long long> &imageList_timestamp){

    std::string image_timestamp;

    int _numFrames{};
    size_t idImage{0};

    for(auto& imgPath : filelist){
        if(isImage(imgPath)){
            _imageList.push_back(imgPath);

            // Getting timestamp from the image name for TU-RGBD dataset
            size_t pos = imgPath.find_last_of("/");
            image_timestamp = imgPath.substr(pos+1);

            pos = image_timestamp.find_last_of(".");
            image_timestamp = image_timestamp.substr(0,pos);
            boost::erase_all(image_timestamp, ".");
            //////////////////////////
            imageList_timestamp.push_back(std::stoll(image_timestamp));
            _numFrames++;
            ++idImage;

        }
    }
}

bool fv::SequenceSettings::isImage(const std::string &fname_) {
    std::string fext = fname_.substr(fname_.find_last_of(".") + 1);
    return !(strcasecmp(fext.c_str(), "JPG") &&
             strcasecmp(fext.c_str(), "JPEG") &&
             strcasecmp(fext.c_str(), "PNG"));
}

std::vector<std::string> fv::SequenceSettings::read_csv(std::string path_, int col){
    std::string line;
    std::string s;
    std::vector<std::string> read_csv_return;

    std::ifstream file_(path_);

    if(file_.good()){
        file_.clear();
        file_.seekg(0, std::ios::beg);
        //std::getline(file_, line);
    }
    else{
        std::cout << "\n FILE NOT FOUND:  "<<std::endl;
        std::cout << path_<<"\n"<<std::endl;
    }

    while (std::getline(file_, line)){
        std::stringstream stream(line);
        for (int j = 0; j < col; ++j) {
            std::getline(stream, s, ',');
        }
        //std::cout << s << std::endl;
        read_csv_return.push_back(s);

    }

    return read_csv_return;
}

#endif