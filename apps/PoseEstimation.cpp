#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <utils.hpp>
#include <geometry.hpp>

using namespace cv;
using namespace std;

void readConfig(String &configPath, Mat &camMatrix, Mat &distCoeffs, Mat &squareCoords)
{
    FileStorage fs(configPath, FileStorage::READ);
    fs["Camera_Matrix"] >> camMatrix;
    fs["Distortion_Coefficients"] >> distCoeffs;
    fs["Square_Coordinates"] >> squareCoords;
}

int main(int argc, char **argv)
{

    Mat image;
    double markerLength = 0.13;
    Ptr<aruco::Dictionary> dictionary;
    vector<vector<Point2f>> corners;
    vector<int> ids;
    String imagePath = "../../data/image001.jpg";
    String configPath = "../../apps/config.yml";

    Mat camMatrix, distCoeffs, squareCoords;
    loadDetectAprilTag(imagePath, image, dictionary, corners, ids);
    readConfig(configPath, camMatrix, distCoeffs, squareCoords);

    if (ids.size() > 0)
    {

        vector<Point2d> aprilTag;
        for (size_t i = 0; i < squareCoords.rows; i++)
        {
            aprilTag.push_back(cv::Point2d((double)squareCoords.at<double>(i, 0), (double)squareCoords.at<double>(i, 1)));
        }
        Mat H = findHomography(aprilTag, corners[0], RANSAC);

        vector<Mat> rotations, translations, normals;

        aruco::drawDetectedMarkers(image, corners, ids);

        Mat projectionMatrix;
        findProjectionMatrix(camMatrix, H, projectionMatrix);
        Mat pt1Cube, pt2Cube, pt3Cube, pt4Cube;
        projectCubePoints(projectionMatrix, pt1Cube, pt2Cube, pt3Cube, pt4Cube);
        drawCube(image, pt1Cube, pt2Cube, pt3Cube, pt4Cube);

    }

    namedWindow("Display Image", WINDOW_AUTOSIZE);
    imshow("Display Image", image);
    waitKey(0);
    return 0;
}