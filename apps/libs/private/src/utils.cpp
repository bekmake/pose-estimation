/**
 * @file utils.cpp
 * @author Malik Bekmurat (bekmake@gmail.com)
 * @brief Utility functions for the pose estimation and cube drawing project.
 * @version 0.1
 * @date 31-08-2022
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <utils.hpp>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
using namespace cv;
using namespace std;

Utils::Utils()
{
}
Utils::~Utils()
{
}
void Utils::detectAprilTag(Mat &image, vector<vector<Point2f>> &corners, vector<int> &ids)
{
    Ptr<aruco::Dictionary> dictionary;
    // add all aruco types in dictionary and use them
    // when corners size is greater than 1, start using this type of aruco
    dictionary = aruco::getPredefinedDictionary(aruco::DICT_APRILTAG_36h11);
    aruco::detectMarkers(image, dictionary, corners, ids);
}

void Utils::crossProduct(Mat &a, Mat &b, Mat &c)
{
    c = (Mat1d(3, 1) << a.at<double>(1) * b.at<double>(2) - a.at<double>(2) * b.at<double>(1),
         a.at<double>(2) * b.at<double>(0) - a.at<double>(0) * b.at<double>(2),
         a.at<double>(0) * b.at<double>(1) - a.at<double>(1) * b.at<double>(0));
}

void Utils::horizontallyConcatThreeMat(Mat &A, Mat &B, Mat &C, Mat &D, Mat &E)
{
    hconcat(A, B, E);
    hconcat(E, C, E);
    hconcat(E, D, E);
}

void Utils::drawCube(Mat &image, Mat &pt1Cube, Mat &pt2Cube, Mat &pt3Cube, Mat &pt4Cube, Point &p5, Point &p6, Point &p7, Point &p8)
{
    Point p1 = pt1Cube, p2 = pt2Cube, p3 = pt3Cube, p4 = pt4Cube;

    line(image, p1, p2, Scalar(0, 255, 0), 2);
    line(image, p2, p3, Scalar(0, 255, 0), 2);
    line(image, p3, p4, Scalar(0, 255, 0), 2);
    line(image, p4, p1, Scalar(0, 255, 0), 2);
    line(image, p1, p6, Scalar(0, 255, 0), 2);
    line(image, p2, p5, Scalar(0, 255, 0), 2);
    line(image, p3, p8, Scalar(0, 255, 0), 2);
    line(image, p4, p7, Scalar(0, 255, 0), 2);
}

void Utils::rectangleToPoints(vector<Point2f> corners ,Point &p5, Point &p6, Point &p7, Point &p8)
{
    p5 = Point(corners[0].x, corners[0].y);
    p6 = Point(corners[1].x, corners[1].y);
    p7 = Point(corners[2].x, corners[2].y);
    p8 = Point(corners[3].x, corners[3].y);
}