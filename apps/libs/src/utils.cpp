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

void crossProduct(Mat& r1, Mat& r2, Mat& r3) {
    double a1, a2, a3, b1, b2, b3;
    a1 = r1.at<double>(0, 0);
    a2 = r1.at<double>(1, 0);
    a3 = r1.at<double>(2, 0);
    b1 = r2.at<double>(0, 0);
    b2 = r2.at<double>(1, 0);
    b3 = r2.at<double>(2, 0);

    double c1, c2, c3;
    c1 = a2 * b3 - a3 * b2;
    c2 = a3 * b1 - a1 * b3;
    c3 = a1 * b2 - a2 * b1;
    r3.push_back(c1);
    r3.push_back(c2);
    r3.push_back(c3);
}