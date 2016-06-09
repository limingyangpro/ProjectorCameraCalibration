/*
 * ScaledDetector.hpp
 *
 *  Created on: Jun 7, 2015
 *      Author: liming
 */

#ifndef IOLIB_SCALEDDETECTOR_HPP_
#define IOLIB_SCALEDDETECTOR_HPP_

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "../skeleton/skeleton.hpp"
using namespace std;

class ImgPatch
{
public:
	cv::Mat_<uchar> cleanedPatch;

	/**
	 * The half of the patch size = (imgPatch.rows-1)/2
	 * cleanedPatch.size = (half_patch_size*2+1)**2 = imgPatch.rows**2
	 * the middel point is
	 * cleanedPatch[half_patch_size][half_patch_size]
	 */
	int half_patch_size;

private:
	/**
	 * Get singled connected skeleton in the image patch
	 */
	void getCleanedPatch(cv::Mat_<uchar> patch);

public:
	ImgPatch() {};
	ImgPatch(cv::Mat_<uchar> patch);

	void setPatch(cv::Mat_<uchar> patch);
	float cornerValue();
	float intersectionValue();
	float cornerOrIntersectionValue();
	float onlyCornerValue();

	bool selfTest();
};

class ScaledDetector
{
private:
int halfPatchSize;
cv::Mat_<uchar> currentSkeleton;
cv::Mat_<float> reponseMap;
ScaledDetector* nextScaleDetector;


public:
ScaledDetector(cv::Mat binaryImg, int half_patch_size, int num_of_scale);
~ScaledDetector();

void detectCornerInCurrentScale();
void detectCorner();
void detectIntersectionInCurrentScale();
void detectIntersection();
void detectAllInCurrentScale();
void detectAll();

void findKeypointsInCurrentScale(vector<cv::Point2f> &keypoints);
void drawKeypoints();

};


#endif /* IOLIB_SCALEDDETECTOR_HPP_ */
