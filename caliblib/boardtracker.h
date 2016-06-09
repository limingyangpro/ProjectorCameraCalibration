/*
 * boardtracker.h
 *
 *  Created on: Dec 11, 2015
 *      Author: liming
 */

#ifndef BOARDTRACKER_H_
#define BOARDTRACKER_H_

#include "pointdetection.h"


class PointSetSteadyChecker
{
private:
	int num;                    //Number of points on the calibration board
	double threshold;            //define under how many pixels that the set are considered as steady
	vector<cv::Point2d> refPtVector;


	/**
	 * This function makes the iPtVector in the right order. The result is a vector of points arranged in a form that
	 * their corresponding model points ID is in ascendent order
	 * @param ptVector [InputOutput] Input/Output point vector, containing point coordinates
	 * @param corresp [Input] Point correspondences between the points on the calibration board (ref) and
	 *        their images in the camera (img), stored under form of <d, <refID, imgID> >, where d is not used here.
	 */
	void rearrangeInModelOrder(vector<cv::Point2d> &ptVector, const vector< pair<int, pair<int,int> > > &corresp);
public:
	PointSetSteadyChecker(int setSize);
	PointSetSteadyChecker();
	void setPtNum(int n);

	/**
	 * Clear recorded ref pt set; used to get new positions
	 */
	void clearRefPtSet();

	/**
	 * This function set the reference point set for PointSetSteady checker
	 * @param ptVector [Input] Input point vector, containing point coordinates
	 * @param corresp [Input] Point correspondences between the points on the calibration board (ref) and
	 *        their images in the camera (img), stored under form of <d, <refID, imgID> >, where d is not used here.
	 */
	void setRefPtSet(const vector<cv::Point2d> &ptVector, const vector< pair<int, pair<int,int> > > &corresp);

	/**
	 * This function checks if the new point set is steady compared to the registered point set
	 * @param ptVector [Input] Input point vector, containing point coordinates
	 * @param corresp [Input] Point correspondences between the points on the calibration board (ref) and
	 *        their images in the camera (img), stored under form of <d, <refID, imgID> >, where d is not used here.
	 */
	bool isSteady(const vector<cv::Point2d> &newptVector, const vector< pair<int, pair<int,int> > > &corresp);

	bool isChecking();
};

class RawData;
class CalibrationEnvironment;
/**
 * This class is used to track the calibration board with LGC algorithm
 * It will retrieve both physical point set and projected point sets
 */
class BoardTracker : public PointDetection
{
public:
	BoardTracker();
	BoardTracker(float ratio);
	~BoardTracker();

	void initialize(int numView,
			CalibrationEnvironment *calibenv,
			RawData* rd,
			const vector<cv::Point2d>* prjPtr,
			vector<bool>* maskPtr);

	float getElapsedTime() const;
	float getCompleteRatio() const;
	bool arePointsAligned() const;

	virtual cv::Mat matchPointSet(const cv::Mat &srcImg);

	cv::Mat_<double> latestPrewrapProjector;          /**Prewrap of projector */
	CalibrationEnvironment* calibEnvironment;

private:

	virtual void intializeDetector();

	/**
	 * Detect points in specified image
	 * @param iImg: [Input] image to be detected
	 * @param oDetectedPts: [Output] Detected points, with origne at bottom left
	 */
	virtual void detect(const cv::Mat_<uchar> &iImg, vector<cv::Point2d> &oDetectedPts);

	RawData* rawDataPtr;
	PointSetSteadyChecker steadyChecker;
	Chronometer tm;
	const double hold_on_ms;
	int numAcquisition;
	float intialPatternRatio;          /**The first projected pattern should be resized by this value */
};





#endif /* BOARDTRACKER_H_ */
