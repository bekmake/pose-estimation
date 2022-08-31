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
    Geometry GeometryInstance;
    
    String configPath = "../../config/config.yml";
    String imagePath = "../../data/";

    Mat camMatrix, distCoeffs, squareCoords;
    readConfig(configPath, camMatrix, distCoeffs, squareCoords);
    GeometryInstance.iterateThroughFolder(imagePath, camMatrix, distCoeffs, squareCoords);
    return 0;
}