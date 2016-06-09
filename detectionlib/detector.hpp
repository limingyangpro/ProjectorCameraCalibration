/*
 * keypointDetector.hpp
 *
 *  Created on: Mar 1, 2015
 *      Author: liming
 */

#ifndef DETECTIONLIB_DETECTOR_HPP_
#define DETECTIONLIB_DETECTOR_HPP_

#include <numeric>
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "scaleddetector/scaleddetector.hpp"
using namespace std;

bool cleanup(vector<cv::Point2d>& inputoutputpoints, float thres);    //Clean up interest points
void revert_y(vector<cv::Point2d>& ioKPoints, int rownum);

class AbstractKPDetector
{
public:
	/**
	 * Constructor with default parameters
	 */
	AbstractKPDetector() {};
	virtual ~AbstractKPDetector() {};

	virtual void runAlgo(const cv::Mat_<uchar> &iSrc, vector<cv::Point2d>& oKPoints, vector<float> &oDiameters) const {} ;

	/**
	 * Main method for detecting
	 * @param iSrc [Input] Image for detecting
	 * @param oKPoints [Output] An array of detected key points
	 */
	virtual bool detect(const cv::Mat_<uchar> &iSrc, vector<cv::Point2d>& oKPoints, vector<float> &oDiameters) const;
};

class SimplePointDetector : public AbstractKPDetector
{
private:
	cv::SimpleBlobDetector::Params detectorparams;
	cv::Ptr< cv::SimpleBlobDetector > blobDetector;

	virtual void runAlgo(const cv::Mat_<uchar> &iSrc, vector<cv::Point2d>& oKPoints, vector<float> &oDiameters) const;

public:
	SimplePointDetector();

	/**
	 * Parameterized constructor
	 * @param blobcolor [Input] Color of the blob to be detected (0 means to extract black blobs, 1 means to extract white blobs)
	 * @param minArea [Input] Each extracted blob's area should be larger than minArea.
	 * @param maxArea [Input] Each extracted blob's area should be smaller than maxArea.
	 */
	SimplePointDetector(int blobcolor, int minArea, int maxArea);
	~SimplePointDetector();
};

class ContoursPointDetector : public AbstractKPDetector
{
private:
	float ratio;              /**Enclosing circle area / blob area */
	virtual void runAlgo(const cv::Mat_<uchar> &iSrc, vector<cv::Point2d>& oKPoints, vector<float> &oDiameters) const;

public:
	ContoursPointDetector();

	/**
	 * Parameterized constructor
	 * @param blobcolor [Input] Color of the blob to be detected (0 means to extract black blobs, 1 means to extract white blobs)
	 * @param minArea [Input] Each extracted blob's area should be larger than minArea.
	 * @param maxArea [Input] Each extracted blob's area should be smaller than maxArea.
	 */
	ContoursPointDetector(float r);
	~ContoursPointDetector();
};

//From Yuji's code
class MSERDetector : public AbstractKPDetector
{
private:
    float m_mserMaxRadiusRatio;
    float m_mserMinRadius;
    float m_mserMearAreaFac;
    float area_sigma;
    bool detectBlackBlob;
	virtual void runAlgo(const cv::Mat_<uchar> &iSrc, vector<cv::Point2d>& oKPoints, vector<float> &oDiameters) const;
	std::vector<cv::RotatedRect> filterEllipses( const std::vector<cv::RotatedRect>& ellipses ) const;

	/**
	 * Use knn approximate search
	 */
	std::vector<cv::RotatedRect> removeIntersectingEllipses( const std::vector<cv::RotatedRect>& ellipses) const;

	/**
	 * Change to exact linear search
	 */
	std::vector<cv::RotatedRect> removeIntersectingEllipses2( const std::vector<cv::RotatedRect>& ellipses ) const;
	std::vector<cv::RotatedRect> filterColors( const cv::Mat_<uchar> &grayImg, const std::vector<cv::RotatedRect>& ellipses ) const;
	std::vector<cv::RotatedRect> filterAreas( const cv::Mat_<uchar> &grayImg, const std::vector<cv::RotatedRect>& ellipses ) const;
	void draw( const cv::Mat_<uchar> &grayImg, const std::vector<cv::RotatedRect>& ellipses) const;
	void printEllipses(const std::vector<cv::RotatedRect>& ellipses) const;
public:
	bool detect(const cv::Mat_<uchar> &iSrc, vector<cv::Point2d>& oKPoints, vector<float> &oDiameters) const;
	MSERDetector();
	~MSERDetector() {};
};


#endif /* DETECTIONLIB_DETECTOR_HPP_ */
