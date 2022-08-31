#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <utils.hpp>
#include <geometry.hpp>

using namespace cv;
using namespace std;

void projectCube(Mat &image, vector<vector<Point2f>> &corners, vector<int> &ids, Mat &camMatrix, Mat &distCoeffs, Mat &squareCoords)
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
    projectCubePoints(projectionMatrix, pt1Cube, pt2Cube, pt3Cube, pt4Cube);
    aruco::drawDetectedMarkers(image, corners, ids);
    drawCube(image, pt1Cube, pt2Cube, pt3Cube, pt4Cube);

    namedWindow("Display Image", WINDOW_AUTOSIZE);
    imshow("Display Image", image);
    waitKey(0);
}

void processImage(Mat &image, Mat &camMatrix, Mat &distCoeffs, Mat &squareCoords)
{

    vector<vector<Point2f>> corners;
    vector<int> ids;

    detectAprilTag(image, corners, ids);
    if (ids.size() > 0)
    {
        projectCube(image, corners, ids, camMatrix, distCoeffs, squareCoords);
    }
}

void iterateThroughFolder(String folderPath, Mat &camMatrix, Mat &distCoeffs, Mat &squareCoords)
{

    vector<String> fileNames;
    glob(folderPath, fileNames, false);
    for (size_t i = 0; i < fileNames.size(); i++)
    {
        Mat image = imread(fileNames[i]);
        processImage(image, camMatrix, distCoeffs, squareCoords);
    }
}

void readConfig(String &configPath, Mat &camMatrix, Mat &distCoeffs, Mat &squareCoords)
{
    FileStorage fs(configPath, FileStorage::READ);
    fs["Camera_Matrix"] >> camMatrix;
    fs["Distortion_Coefficients"] >> distCoeffs;
    fs["Square_Coordinates"] >> squareCoords;
}

int main(int argc, char **argv)
{

    String configPath = "../../apps/config.yml";
    String imagePath = "../../data/";

    Mat camMatrix, distCoeffs, squareCoords;
    readConfig(configPath, camMatrix, distCoeffs, squareCoords);
    iterateThroughFolder(imagePath, camMatrix, distCoeffs, squareCoords);
    return 0;
}