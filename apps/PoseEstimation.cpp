#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <utils.hpp>

using namespace cv;
using namespace std;



int main(int argc, char **argv)
{


    Mat image;
    double markerLength = 0.13;
    Ptr<aruco::Dictionary> dictionary ;
    vector<vector<Point2f>> corners;
    vector<int> ids;
    String ImagePath="../../data/image001.jpg";

    Mat camMatrix=(Mat1d(3, 3) << 766.1088867187500, 0, 313.9585628047498, 0, 769.9354248046875, 250.3607131410900, 0, 0, 1);
    Mat distCoeffs= (Mat1d(1, 5) << 0., 0., 0., 0., 0.);
    
    LoadDetectAprilTag(ImagePath,image, dictionary,corners,ids );

    // image = imread("../../data/image001.jpg");
    // camMatrix = (Mat1d(3, 3) << 766.1088867187500, 0, 313.9585628047498, 0, 769.9354248046875, 250.3607131410900, 0, 0, 1);
    // distCoeffs = (Mat1d(1, 5) << 0., 0., 0., 0., 0.);
    // // use all aruco types
    // // when corners size is greater than 1, start using this type
    // Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_APRILTAG_36h11);
    // vector<vector<Point2f>> corners;
    // vector<int> ids;

    // aruco::detectMarkers(image, dictionary, corners, ids);
    


    if (ids.size() > 0)
    {

        vector<Vec3d> rvecs, tvecs;

        aruco::estimatePoseSingleMarkers(corners, 0.05, camMatrix, distCoeffs, rvecs,
                                         tvecs);

        vector<vector<double>> squareCorners = {{0, 0}, {0, 1}, {1, 1}, {1, 0}};
        vector<Point2d> aprilTag;
        for (size_t i = 0; i < squareCorners.size(); i++)
        {
            aprilTag.push_back(cv::Point2d((double)squareCorners[i][0], (double)squareCorners[i][1]));
        }
        Mat H = findHomography(aprilTag, corners[0], RANSAC);
        cout << "H:\n"
             << H << endl;
        vector<Mat> rotations, translations, normals;
        decomposeHomographyMat(H, camMatrix, rotations, translations, normals);
        // I concatenate rotations and translations matrices
        // multiply intrinsics to concatenated vector
        // obtain projection matrix
        aruco::drawDetectedMarkers(image, corners, ids);

        for (int j = 0; j < rvecs.size(); ++j)
        {
            auto rvec = rvecs[j];
            auto tvec = tvecs[j];
            cv::drawFrameAxes(image, camMatrix, distCoeffs, rvec, tvec, 0.02);
        }
    }

    namedWindow("Display Image", WINDOW_AUTOSIZE);
    imshow("Display Image", image);
    waitKey(0);
    return 0;
}
