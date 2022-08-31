/**
 * @file utils.hpp
 * @author Malik Bekmurat (bekmake@gmail.com)
 * @brief Utility functions for the pose estimation and cube drawing project.
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
using namespace cv;
using namespace std;

class Utils
{

public:
    /**
     * @brief
     * Constructor
     */
    Utils();

    /**
     * @brief
     * Destructor
     */
    ~Utils();

    /**
     * @brief
     * Function to detect aruco tag in an image.
     */
    void detectAprilTag(Mat &image, vector<vector<Point2f>> &corners, vector<int> &ids);

    /**
     * @brief
     * Function to calculate the cross product of two vectors.
     */
    void crossProduct(Mat &a, Mat &b, Mat &c);

    /**
     * @brief
     * Function to draw a cube on an image.
     */
    void drawCube(Mat &image, Mat &pt1Cube, Mat &pt2Cube, Mat &pt3Cube, Mat &pt4Cube, Point &p5, Point &p6, Point &p7, Point &p8);

    /**
     * @brief
     * Function to horizontally concat four matrices.
     */
    void horizontallyConcatThreeMat(Mat &A, Mat &B, Mat &C, Mat &D, Mat &E);

    /**
     * @brief
     * Function which splits vecotr of points into four points.
     */

    void rectangleToPoints(vector<Point2f> corners ,Point &p5, Point &p6, Point &p7, Point &p8);
};