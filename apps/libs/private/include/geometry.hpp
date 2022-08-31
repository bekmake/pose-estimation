/**
 * @file geometry.hpp
 * @author Malik Bekmurat (bekmake@gmail.com)
 * @brief Project geometry functions for the pose estimation and cube drawing project.
 * @version 0.1
 * @date 31-08-2022
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <utils.hpp>

using namespace cv;
using namespace std;

class Geometry
{
private:
    Utils UtilsInstance;

public:
    /**
     * @brief
     * Constructor
     */
    Geometry();
    /**
     * @brief
     * Destructor
     */
    ~Geometry();

    /**
     * @brief
     * Function to calculate the projection matrix from the camera matrix and the homography matrix.
     */
    void findProjectionMatrix(Mat &camMatrix, Mat &H, Mat &projectionMatrix);

    /**
     * @brief
     * Function to project the cube points using the projection matrix.
     */
    void projectCubePoints(Mat &projectionMatrix, Mat &pt1Cube, Mat &pt2Cube, Mat &pt3Cube, Mat &pt4Cube);

    /**
     * @brief
     * Fucniton to go through the folder and process the images.
     */
    void iterateThroughFolder(String folderPath, Mat &camMatrix, Mat &distCoeffs, Mat &squareCoords);

    /**
     * @brief
     * Function to detect apriltag and project its coordinates on the image.
     */
    void processImage(Mat &image, Mat &camMatrix, Mat &distCoeffs, Mat &squareCoords);

    /**
     * @brief
     * Function that does projective geometry calculations and draws cube over the apriltag.
     */
    void projectCube(Mat &image, vector<vector<Point2f>> &corners, vector<int> &ids, Mat &camMatrix, Mat &distCoeffs, Mat &squareCoords);
};