#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>


struct DataFrame { // represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // camera image
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
};

struct ExperimentData {
    std::string detector;
    std::string descriptor;

    int FrameKps[10];
    int ROIKps[10];
    int numMatches[10];

    double NeighSizes[10];
    double totalRuntime[10];
};

#endif /* dataStructures_h */
