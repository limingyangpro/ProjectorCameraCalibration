/*
 * skeleton.hpp
 *
 *  Created on: Apr 10, 2015
 *      Author: liming
 */

#ifndef DETECTIONLIB_SKELETON_SKELETON_HPP_
#define DETECTIONLIB_SKELETON_SKELETON_HPP_

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

int thinning(cv::Mat &roadnetwork, int iter);
void skeletonize(cv::Mat& inputoutput);



#endif /* DETECTIONLIB_SKELETON_SKELETON_HPP_ */
