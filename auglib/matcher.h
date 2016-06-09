/*
 * matcher.h
 *
 *  Created on: Feb 28, 2016
 *      Author: liming
 */

#ifndef AUGLIB_MATCHER_H_
#define AUGLIB_MATCHER_H_

#include <opencv2/core.hpp>

#include "../lgc/include/lgcmethod.hpp"
#include "../detectionlib/detector.hpp"
#include <iostream>
using namespace std;

namespace augmentation
{

void distortPoint(const cv::Point2f & src, cv::Point2f & dst, const cv::Mat & cameraMatrix, const cv::Mat & distorsionMatrix);
void distortPoints(const vector< cv::Point2f > & src, vector< cv::Point2f > & dst, const cv::Mat & cameraMatrix, const cv::Mat & distorsionMatrix);


class AbstractMatcher
{
protected:
	vector<cv::Point2d> referencePattern;

	virtual bool readRefPointsFromFile(string filename) { return false; };
public:
	AbstractMatcher() {};
	AbstractMatcher(string filename) { readRefPointsFromFile(filename); };

	virtual cv::Mat calHomography(const cv::Mat &img, cv::Mat_<double> &homo) { return cv::Mat(); };
};

class LGCMatcher : public AbstractMatcher
{
private:
	AbstractKPDetector* detector;
	lgc2::LGCmethod<pcl::PointXY>* lgcalgo;   /**<Matching algorithm>*/

	virtual bool readRefPointsFromFile(string filename);
	void initialize();

public:
	LGCMatcher(string filename);
	~LGCMatcher();
	virtual cv::Mat calHomography(const cv::Mat &img, cv::Mat_<double> &homo);
};

class ChessBoardMatcher : public AbstractMatcher
{
private:
	int colNum, rowNum;
	virtual bool readRefPointsFromFile(string filename);

public:
	ChessBoardMatcher(string filename);
	virtual cv::Mat calHomography(const cv::Mat &img, cv::Mat_<double> &homo);
};


class AcircleGridMatcher : public AbstractMatcher
{
private:
	cv::Size patternSz;
	int colNum, rowNum;
	virtual bool readRefPointsFromFile(string filename);

public:
	AcircleGridMatcher(string filename);
	virtual cv::Mat calHomography(const cv::Mat &img, cv::Mat_<double> &homo);
	cv::Mat_<double> findErrs(const cv::Mat &srcImg,
	const cv::Mat_<double> &trueHomo,
	const cv::Mat_<double> &projHomo,
	const cv::Mat_<double> &projIntrinsic,
	const cv::Mat_<double> &projDistorsion);
};
}

#endif /* AUGLIB_MATCHER_H_ */
