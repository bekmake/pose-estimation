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
    hconcat(r1, r2, B);
    hconcat(B, r3, B);
    hconcat(B, t, B);
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

int main(int argc, char **argv)
{

    Mat image;
    double markerLength = 0.13;
    Ptr<aruco::Dictionary> dictionary;
    vector<vector<Point2f>> corners;
    vector<int> ids;
    String ImagePath = "../../data/image001.jpg";

    Mat camMatrix = (Mat1d(3, 3) << 766.1088867187500, 0, 313.9585628047498, 0, 769.9354248046875, 250.3607131410900, 0, 0, 1);
    Mat distCoeffs = (Mat1d(1, 5) << 0., 0., 0., 0., 0.);

    LoadDetectAprilTag(ImagePath, image, dictionary, corners, ids);

    if (ids.size() > 0)
    {

        vector<Vec3d> rvecs, tvecs;

        aruco::estimatePoseSingleMarkers(corners, 0.05, camMatrix, distCoeffs, rvecs,
                                         tvecs);

        vector<vector<double>> squareCorners = {{0, 1}, {0, 0}, {1, 0}, {1, 1}};
        vector<Point2d> aprilTag;
        for (size_t i = 0; i < squareCorners.size(); i++)
        {
            aprilTag.push_back(cv::Point2d((double)squareCorners[i][0], (double)squareCorners[i][1]));
        }
        Mat H = findHomography(aprilTag, corners[0], RANSAC);

        vector<Mat> rotations, translations, normals;

        aruco::drawDetectedMarkers(image, corners, ids);

        Mat projectionMatrix;
        findProjectionMatrix(camMatrix, H, projectionMatrix);
        Mat pt1Cube, pt2Cube, pt3Cube, pt4Cube;
        projectCubePoints(projectionMatrix, pt1Cube, pt2Cube, pt3Cube, pt4Cube);
        Point p1 = pt1Cube, p2 = pt2Cube, p3 = pt3Cube, p4 = pt4Cube;
        line(image, p1, p2, Scalar(0, 255, 0), 2, LINE_8);
        line(image, p2, p3, Scalar(0, 255, 0), 2, LINE_8);
        line(image, p3, p4, Scalar(0, 255, 0), 2, LINE_8);
        line(image, p4, p1, Scalar(0, 255, 0), 2, LINE_8);


        for (int j = 0; j < rvecs.size(); ++j)
        {
            auto rvec = rvecs[j];
            auto tvec = tvecs[j];
            cv::drawFrameAxes(image, camMatrix, distCoeffs, rvec, tvec, 0.02);
            cout << "rvec 3:\n"
                 << rvec << endl;
        }
    }

    namedWindow("Display Image", WINDOW_AUTOSIZE);
    imshow("Display Image", image);
    waitKey(0);
    return 0;
}
