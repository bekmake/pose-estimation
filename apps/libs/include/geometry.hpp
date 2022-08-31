#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <utils.hpp>

using namespace cv;
using namespace std;

void findProjectionMatrix(Mat &camMatrix, Mat &H, Mat &projectionMatrix);
void projectCubePoints(Mat &projectionMatrix, Mat &pt1Cube, Mat &pt2Cube, Mat &pt3Cube, Mat &pt4Cube);
