#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;
int main(int argc, char** argv )
{

    Mat image;
    image = imread("C:/Users/kimil/interviews/TII/AprilTagSeq/image001.jpg");
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", image);
    waitKey(0);
    return 0;
}