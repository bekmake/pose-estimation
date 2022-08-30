#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <utils.hpp>
using namespace cv;
using namespace std;
void LoadDetectAprilTag(String ImagePath, Mat &image,Ptr<aruco::Dictionary> &dictionary,vector<vector<Point2f>> &corners,vector<int> &ids )
{

    image = imread(ImagePath);
    // use all aruco types
    // when corners size is greater than 1, start using this type
    dictionary = aruco::getPredefinedDictionary(aruco::DICT_APRILTAG_36h11);

    aruco::detectMarkers(image, dictionary, corners, ids);
}