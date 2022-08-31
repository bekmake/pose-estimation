#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
using namespace cv;
using namespace std;

void iterateThroughFolder(String folderPath, vector<String> &imagePaths);
void loadDetectAprilTag(String ImagePath, Mat &image, Ptr<aruco::Dictionary> &dictionary, vector<vector<Point2f>> &corners, vector<int> &ids);
void crossProduct(Mat &a, Mat &b, Mat &c);
void drawCube(Mat &image, Mat &pt1Cube, Mat &pt2Cube, Mat &pt3Cube, Mat &pt4Cube);
void horizontallyConcatThreeMat(Mat &A, Mat &B, Mat &C, Mat &D, Mat &E);
