#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <utils.hpp>

using namespace cv;
using namespace std;

void findProjectionMatrix(Mat &camMatrix, Mat &H, Mat &projectionMatrix)
{
    Mat camMatrixInv, B_tilde, r1, r2, r3, t, B;
    camMatrixInv = camMatrix.inv();
    B_tilde = camMatrixInv * H;
    double lambda;
    lambda = 2 / (norm(camMatrixInv * H.col(0)) + norm(camMatrixInv * H.col(1)));
    r1 = lambda * B_tilde.col(0);
    r2 = lambda * B_tilde.col(1);
    crossProduct(r1, r2, r3);
    t = lambda * B_tilde.col(2);
    horizontallyConcatThreeMat(r1, r2, r3, t, B);
    projectionMatrix = camMatrix * B;
}

void projectCubePoints(Mat &projectionMatrix, Mat &pt1Cube, Mat &pt2Cube, Mat &pt3Cube, Mat &pt4Cube)
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