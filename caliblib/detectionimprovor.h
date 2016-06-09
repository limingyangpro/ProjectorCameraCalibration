/*
 * improvedetection.h
 *
 *  Created on: Jan 22, 2016
 *      Author: liming
 */

#ifndef CALIBLIB_DETECTIONIMPROVOR_H_
#define CALIBLIB_DETECTIONIMPROVOR_H_

#include "pointdetection.h"

class RawData;

class DetectionImprovor : public PointDetection
{
protected:
	vector<cv::Point2f> regionPolygone;
	RawData* rawDataPtr;

	void initialize(const RawData* rwDataPtr);
	virtual void intializeDetector();
	virtual void detect(const cv::Mat_<uchar> &iImg, vector<cv::Point2d> &oDetectedPts);

public:
	DetectionImprovor();
	RawData* improveDetection(const RawData* rwDataPtr);
};




#endif /* CALIBLIB_DETECTIONIMPROVOR_H_ */
