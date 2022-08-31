/**
 * @file geometry.cpp
 * @author Malik Bekmurat (bekmake@gmail.com)
 * @brief Project geometry functions for the pose estimation and cube drawing project.
 * @version 0.1
 * @date 31-08-2022
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <geometry.hpp>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <utils.hpp>

using namespace cv;
using namespace std;

Geometry::Geometry()
{
}

Geometry::~Geometry()
{
}
void Geometry::findProjectionMatrix(Mat &camMatrix, Mat &H, Mat &projectionMatrix)
{
    Mat camMatrixInv, B_tilde, r1, r2, r3, t, B;
    camMatrixInv = camMatrix.inv();
    B_tilde = camMatrixInv * H;
    double lambda;
    lambda = 2 / (norm(camMatrixInv * H.col(0)) + norm(camMatrixInv * H.col(1)));
    r1 = lambda * B_tilde.col(0);
    r2 = lambda * B_tilde.col(1);
    UtilsInstance.crossProduct(r1, r2, r3);
    t = lambda * B_tilde.col(2);
    UtilsInstance.horizontallyConcatThreeMat(r1, r2, r3, t, B);
    projectionMatrix = camMatrix * B;
}

void Geometry::projectCubePoints(Mat &projectionMatrix, Mat &pt1Cube, Mat &pt2Cube, Mat &pt3Cube, Mat &pt4Cube)
{

    Mat pt1 = (Mat1d(4, 1) << 0, 0, -1, 1);
    Mat pt2 = (Mat1d(4, 1) << 0, 1, -1, 1);
    Mat pt3 = (Mat1d(4, 1) << 1, 1, -1, 1);
    Mat pt4 = (Mat1d(4, 1) << 1, 0, -1, 1);

    pt1Cube = projectionMatrix * pt1;
    pt1Cube = pt1Cube / pt1Cube.at<double>(2);
    pt2Cube = projectionMatrix * pt2;
    pt2Cube = pt2Cube / pt2Cube.at<double>(2);
    pt3Cube = projectionMatrix * pt3;
    pt3Cube = pt3Cube / pt3Cube.at<double>(2);
    pt4Cube = projectionMatrix * pt4;
    pt4Cube = pt4Cube / pt4Cube.at<double>(2);
    pt1Cube.pop_back(1);
    pt2Cube.pop_back(1);
    pt3Cube.pop_back(1);
    pt4Cube.pop_back(1);
}

void Geometry::projectCube(Mat &image, vector<vector<Point2f>> &corners, vector<int> &ids, Mat &camMatrix, Mat &distCoeffs, Mat &squareCoords)
{

    vector<Point2d> aprilTag;
    for (size_t i = 0; i < squareCoords.rows; i++)
    {
        aprilTag.push_back(cv::Point2d((double)squareCoords.at<double>(i, 0), (double)squareCoords.at<double>(i, 1)));
    }
    Mat H = findHomography(aprilTag, corners[0], RANSAC);
    Mat projectionMatrix;
    findProjectionMatrix(camMatrix, H, projectionMatrix);
    Mat pt1Cube, pt2Cube, pt3Cube, pt4Cube;
    Mat pt5Cube, pt6Cube, pt7Cube, pt8Cube;
    projectCubePoints(projectionMatrix, pt1Cube, pt2Cube, pt3Cube, pt4Cube);
    aruco::drawDetectedMarkers(image, corners, ids);
    Point p5, p6, p7, p8;
    UtilsInstance.rectangleToPoints(corners[0],p5, p6, p7, p8);
    UtilsInstance.drawCube(image, pt1Cube, pt2Cube, pt3Cube, pt4Cube, p5, p6, p7, p8);

    namedWindow("Display Image", WINDOW_AUTOSIZE);
    imshow("Display Image", image);
    waitKey(1000./24);
}

void Geometry::processImage(Mat &image, Mat &camMatrix, Mat &distCoeffs, Mat &squareCoords)
{

    vector<vector<Point2f>> corners;
    vector<int> ids;

    UtilsInstance.detectAprilTag(image, corners, ids);
    if (ids.size() > 0)
    {
        projectCube(image, corners, ids, camMatrix, distCoeffs, squareCoords);
    }
}

void Geometry::iterateThroughFolder(String folderPath, Mat &camMatrix, Mat &distCoeffs, Mat &squareCoords)
{

    vector<String> fileNames;
    glob(folderPath, fileNames, false);
    for (size_t i = 0; i < fileNames.size(); i++)
    {
        Mat image = imread(fileNames[i]);
        processImage(image, camMatrix, distCoeffs, squareCoords);
    }
}