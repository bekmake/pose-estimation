#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <utils.hpp>
using namespace cv;
using namespace std;

void detectAprilTag(Mat &image, vector<vector<Point2f>> &corners, vector<int> &ids)
{
    Ptr<aruco::Dictionary> dictionary;
    // add all aruco types in dictionary and use them 
    // when corners size is greater than 1, start using this type of aruco
    dictionary = aruco::getPredefinedDictionary(aruco::DICT_APRILTAG_36h11);
    aruco::detectMarkers(image, dictionary, corners, ids);
}

void crossProduct(Mat &a, Mat &b, Mat &c)
{
    c = (Mat1d(3, 1) << a.at<double>(1) * b.at<double>(2) - a.at<double>(2) * b.at<double>(1),
         a.at<double>(2) * b.at<double>(0) - a.at<double>(0) * b.at<double>(2),
         a.at<double>(0) * b.at<double>(1) - a.at<double>(1) * b.at<double>(0));
}

void iterateThroughFolder(String folderPath, vector<String> &imagePaths)
{
    String path = folderPath;
    vector<String> fileNames;
    glob(path, fileNames, false);
    for (size_t i = 0; i < fileNames.size(); i++)
    {
        imagePaths.push_back(fileNames[i]);
    }
}

void horizontallyConcatThreeMat(Mat &A, Mat &B, Mat &C, Mat &D, Mat &E)
{
    hconcat(A, B, E);
    hconcat(E, C, E);
    hconcat(E, D, E);
}

void drawCube(Mat &image, Mat &pt1Cube, Mat &pt2Cube, Mat &pt3Cube, Mat &pt4Cube)
{
    Point p1 = pt1Cube, p2 = pt2Cube, p3 = pt3Cube, p4 = pt4Cube;
    line(image, p1, p2, Scalar(0, 255, 0), 2);
    line(image, p2, p3, Scalar(0, 255, 0), 2);
    line(image, p3, p4, Scalar(0, 255, 0), 2);
    line(image, p4, p1, Scalar(0, 255, 0), 2);
}