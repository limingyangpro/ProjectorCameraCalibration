/*
 * detectionimprovor.cpp
 *
 *  Created on: Jan 22, 2016
 *      Author: liming
 */

#include "detectionimprovor.h"
#include "calibdata.h"

DetectionImprovor::DetectionImprovor() : rawDataPtr(0)
{

}

void DetectionImprovor::intializeDetector()
{
	detector = new MSERDetector();
}

void DetectionImprovor::initialize(const RawData* oldDataPtr)
{
	rawDataPtr = new RawData();
	rawDataPtr->setAllSizes(oldDataPtr->physicalBoardSize,
			oldDataPtr->cameraImageSize,
			oldDataPtr->projectorImageSize );
	rawDataPtr->setModelPatterns(*oldDataPtr->physicalPatternPtr, *oldDataPtr->unwrapedprojectedPatternPtr);

	setOrignalPointPatterns(rawDataPtr->physicalPatternPtr, rawDataPtr->unwrapedprojectedPatternPtr);
	PointDetection::initialize();
}


void DetectionImprovor::detect(const cv::Mat_<uchar> &iImg,vector<cv::Point2d> &oDetectedPts)
{
	vector<cv::Point2d> rawDetectedPoints;
	vector<float> pointDiameter;
	detector->detect(iImg, rawDetectedPoints, pointDiameter);

	oDetectedPts.clear();
	for (unsigned int i = 0 ; i < rawDetectedPoints.size() ; i++)
	{
		if (cv::pointPolygonTest(regionPolygone, rawDetectedPoints[i], false) > 0)
		{
			oDetectedPts.push_back(rawDetectedPoints[i]);
		}
	}
}

//cv::Mat findHomographyPoint2d(const vector<cv::Point2d> &from, const vector<cv::Point2d> &to)
//{
//	assert(from.size() == to.size());
//	vector<cv::Point2f> from_float(from.size()), to_float(to.size());
//	for (unsigned int j = 0 ; j < from.size() ; j++)
//	{
//		from_float[j] = cv::Point2f((float)from[j].x, (float)from[j].y);
//		to_float[j] = cv::Point2f((float)to[j].x, (float)to[j].y);
//	}
//	return cv::findHomography(from_float, to_float);
//}

RawData* DetectionImprovor::improveDetection(const RawData* oldDataPtr)
{
	initialize(oldDataPtr);

	cv::Mat img_gray_orignal;
	vector<cv::Point2f> refPts, detectedPts;
	cv::Mat_<double> homobc;

	for (int i = 0 ; i < oldDataPtr->recordedImage.size() ; i++)
	{
		//Set the real projected pattern
		setProjectedPatterns(&oldDataPtr->allProjectedPts[i]);
		//Get gray image
		cv::cvtColor( oldDataPtr->recordedImage[i], img_gray_orignal, CV_BGR2GRAY );

		/**
		 * Improve the physical point detection
		 */
		homobc = findHomography(oldDataPtr->physicalPattern2D[i], oldDataPtr->physicalPointsScene[i]);

		//set image to match
		regionPolygone = size2Boundingbox(rawDataPtr->getBoardSize(), 10);
		warpPerspective(img_gray_orignal, img_gray, homobc.inv(), oldDataPtr->physicalBoardSize, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 255);  //Empty area is white
		matchPhysicalPointSet();

//		cv::cvtColor(img_gray, img_to_show, CV_GRAY2RGB);
//		imshow("physical", img_to_show);
//		cv::waitKey(-1);
		perspectiveTransform(boardPtsInScene, boardPtsInScene, homobc);


		/**
		 * Improve the projected point detection
		 */
		if (!latestHomoCam.empty())
		{
			cv::perspectiveTransform( regionPolygone, regionPolygone, homobc);

//			cv::Mat mask = getBoardAreaMask(img_gray_orignal, homobc);
//			cv::Mat boarderArea(img_gray_orignal.rows, img_gray_orignal.cols, CV_8UC1, cv::Scalar(0));
//			img_gray_orignal.copyTo(boarderArea, mask);
//			img_gray_orignal = boarderArea;

			cv::Mat_<double> homopc = findHomography(oldDataPtr->projectedPatternModel[i], oldDataPtr->projectedPointsScene[i]);

			cv::perspectiveTransform( regionPolygone, regionPolygone,  homopc.inv());
			warpPerspective(img_gray_orignal, img_gray, homopc.inv(), oldDataPtr->projectorImageSize, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 255);  //Empty area is white
			matchProjectedPointSet();

//			cv::cvtColor(img_gray, img_to_show, CV_GRAY2RGB);
//			imshow("projected", img_to_show);
//			cv::waitKey(-1);
			perspectiveTransform(projectedPtsInScene, projectedPtsInScene, homopc);
		}

		if ((!correspPhysicalPoints.empty())&&(!correspProjectedPoints.empty()))
		{
			cout<<"Image "<<i<<" processed , left "<<correspPhysicalPoints.size()<<" physical points and "<<correspProjectedPoints.size()<<" points"<<endl;
			rawDataPtr->addOneObservation(
					oldDataPtr->recordedImage[i],
					latestHomoCam,
					latestHomoProjectorUnwraped,
					boardPtsInScene,
					correspPhysicalPoints,
					projectedPatternPtr,
					projectedPtsInScene,
					correspProjectedPoints);
		}
	}

	return rawDataPtr;

};


