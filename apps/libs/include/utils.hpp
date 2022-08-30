#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
using namespace cv;
using namespace std;

void LoadDetectAprilTag(String ImagePath, Mat &image,Ptr<aruco::Dictionary> &dictionary,vector<vector<Point2f>> &corners,vector<int> &ids );