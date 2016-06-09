/*
 * boardtracker.h
 *
 *  Created on: Dec 11, 2015
 *      Author: liming
 */

#ifndef POINTDETECTION_H_
#define POINTDETECTION_H_

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "../detectionlib/detector.hpp"
#include "../lgc/include/lgcmethod.hpp"
#include "../lgc/include/affine2D.hpp"
#include "../lgc/include/perspective2D.hpp"

class CalibrationData;

template <typename TypePoint2D>
vector<TypePoint2D> convertPointType(const vector<cv::Point2d> &iPoints)
{
	vector<TypePoint2D> result(iPoints.size());
	for (int i = 0 ; i < iPoints.size() ; i ++)
	{
		result[i].x = (float)iPoints[i].x;
		result[i].y = (float)iPoints[i].y;
	}

	return result;
}

/**
 * This class is used to track the calibration board with LGC algorithm
 * It will retrieve both physical point set and projected point sets
 */
class PointDetection
{
public:
	PointDetection();
	virtual ~PointDetection();


	void setOrignalPointPatterns(const vector<cv::Point2d>* phsycialPts, const vector<cv::Point2d>* unwrapedPrjPts);
	void setProjectedPatterns(const vector<cv::Point2d>* prjPts);

	void initialize();

	virtual cv::Mat matchPointSet(const cv::Mat &srcImg) {return cv::Mat(); };

protected:
	/**
	 * Draw matched points in the img with homography found
	 *
	 * @param img: [InputOutput] img to draw on
	 * @param color: [Input] Color to use
	 * @param modelPtsVector: [Input] A vector contains model points, (bottom-left origin)
	 * @param matchedPairs: [Input] Point correspondences between the model points (ref) and the scene points (img),
	 *                      <d, <refID, imgID> >, where d is not used here.
	 * @param homo: [Input] Homography between models points and scene points, scenePt = Homo*modelPt
	 */
	static void drawMatchedPoints(cv::Mat &img, const cv::Scalar &color,
			const vector<cv::Point2d> &modelPtsVector,
			const vector< pair<int, pair<int,int> > > &matchedPairs,
			const cv::Mat_<double> homo);

	/**
	 * Initialize the matching algorithm and detectors
	 */
	virtual void intializeDetector() {return; };
	void initializeMatchingAlgo();

	/**
	 * Detect points in specified image
	 * @param iImg: [Input] image to be detected
	 * @param oDetectedPts: [Output] Detected points, with origne at bottom left
	 */
	virtual void detect(const cv::Mat_<uchar> &iImg, vector<cv::Point2d> &oDetectedPts) {return;};

	/**
	 * Convert size to boundingbox
	 */
	static vector< cv::Point2f > size2Boundingbox(const cv::Size &sz, int offset);

	cv::Mat_<uchar> getBoardAreaMask(cv::Mat camImg, cv::Size boardSize, cv::Mat homo);
	void matchPhysicalPointSet();
	void matchProjectedPointSet();

	cv::Mat img_gray;
	cv::Mat img_to_show;

	const vector<cv::Point2d>* unwrapedProjectedPatternPtr;   /**< Pointer to the unwarped projected pattern, used for model registration >*/
	const vector<cv::Point2d>* projectedPatternPtr;           /**< Pointer to Vector which contains projected pattern >*/
	const vector<cv::Point2d>* physicalPatternPtr;            /**< Pointer to Vector which contains board pattern >*/

	vector<cv::Point2d> boardPtsInScene;
	vector< pair<int, pair<int,int> > > correspPhysicalPoints;
	vector<cv::Point2d> projectedPtsInScene;
	vector< pair<int, pair<int,int> > > correspProjectedPoints;


	AbstractKPDetector* detector;
	lgc2::LGCmethod<pcl::PointXY>* lgcalgo;   /**<Matching algorithm>*/

	cv::Mat_<double> latestHomoCam;
	cv::Mat_<double> latestHomoProjectorUnwraped;

};

#endif /* POINTDETECTION_H_ */
