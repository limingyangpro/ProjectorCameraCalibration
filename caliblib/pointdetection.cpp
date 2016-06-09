/*
 * PointDetection.cpp
 *
 *  Created on: Dec 11, 2015
 *      Author: liming
 */

#include "pointdetection.h"

PointDetection::PointDetection()
:	lgcalgo(0),
	detector(0),
	unwrapedProjectedPatternPtr(0),
	projectedPatternPtr(0),
	physicalPatternPtr(0)
{
}

PointDetection::~PointDetection()
{
	if (lgcalgo)
	{
		delete lgcalgo;
	}

	if (detector)
	{
		delete detector;
	}
}

void PointDetection::setOrignalPointPatterns(const vector<cv::Point2d>* phsycialPts, const vector<cv::Point2d>* unwrapedPrjPts)
{
	physicalPatternPtr = phsycialPts;
	unwrapedProjectedPatternPtr = unwrapedPrjPts;
}

void PointDetection::setProjectedPatterns(const vector<cv::Point2d>* prjPts)
{
	projectedPatternPtr = prjPts;
}

void PointDetection::initialize()
{
	intializeDetector();
	initializeMatchingAlgo();
}

void PointDetection::initializeMatchingAlgo()
{
	lgc2::Affine2D*  affinetrans = new lgc2::Affine2D(1.3f,10.0f);
	lgc2::Perspective2D* persptrans = new lgc2::Perspective2D();
	lgc2::Parameter* lgcparam = new lgc2::Parameter(
			6, 6,   //kr, ki
			0.03,   //eta
			10, 1000,
			1.3, 100,
			2, 0.1,
			45, 20,
			8, 4);
	lgc2::Generator2Daff* generator2Daff = new lgc2::Generator2Daff(lgcparam,affinetrans);
	lgcalgo = new lgc2::LGCmethod<pcl::PointXY>(lgcparam, generator2Daff, affinetrans, persptrans);

	{
		//Register board pattern
		vector<pcl::PointXY> convertedPoints = convertPointType<pcl::PointXY>(*(physicalPatternPtr));
		lgcalgo->setRefPoints(&convertedPoints);
	}
	{
		//Register projected pattern
		//We use unwrapedprojectedPatternPtr for point matching, not to find homgraphy, the resulting homography is not true
		vector<pcl::PointXY> convertedPoints = convertPointType<pcl::PointXY>(*(unwrapedProjectedPatternPtr));
		lgcalgo->setRefPoints(&convertedPoints);
	}
}



void PointDetection::drawMatchedPoints(cv::Mat &img, const cv::Scalar &color,
		const vector<cv::Point2d> &modelPtsVector,
		const vector< pair<int, pair<int,int> > > &matchedPairs,
		const cv::Mat_<double> homo)
{
	//Draw matched points, with estimated homography
	vector<cv::Point2d> modelPt2D(matchedPairs.size()), projectedScenePt2D;
	for (unsigned int i = 0 ; i < matchedPairs.size() ; i++)
	{
		modelPt2D[i] = ( modelPtsVector[matchedPairs[i].second.first] );
	}
	cv::perspectiveTransform(modelPt2D, projectedScenePt2D, homo);

	//projectedScenePt2D contains scene point coordinates, bottom-left origin.
	for (unsigned int i = 0 ; i < projectedScenePt2D.size() ; i++)
	{
//		circle(img, cv::Point2d(projectedScenePt2D[i].x, img.rows - projectedScenePt2D[i].y), 5,  color, 3, 8, 0 );
		circle(img, cv::Point2d(projectedScenePt2D[i].x, projectedScenePt2D[i].y), 5,  color, 3, 8, 0 );
	}
}

vector< cv::Point2f > PointDetection::size2Boundingbox(const cv::Size &sz, int offset)
{
	vector< cv::Point2f > boundingbox;

	boundingbox.resize(4);
	boundingbox[0].x = 0 + offset;
	boundingbox[0].y = 0 + offset;
	//max corner
	boundingbox[2].x = sz.width - 2*offset;
	boundingbox[2].y = sz.height - 2*offset;
	//	//other corner
	boundingbox[1].x = boundingbox[0].x; boundingbox[1].y = boundingbox[2].y;
	boundingbox[3].x = boundingbox[2].x; boundingbox[3].y = boundingbox[0].y;

	return boundingbox;
}

cv::Mat_<uchar> PointDetection::getBoardAreaMask(cv::Mat camImg, cv::Size boardSize, cv::Mat homo)
{
	assert(!homo.empty());
	vector<cv::Point2f> warpedBoardPolygone;
	cv::Mat_<uchar> mask = cv::Mat::zeros(camImg.size(), CV_8U);
	cv::perspectiveTransform( size2Boundingbox(boardSize, 0), warpedBoardPolygone, homo);  //results in canonical coordinate

	vector<cv::Point2i> drawPolygone(4);
	for (int i = 0 ; i < warpedBoardPolygone.size() ; i++)
	{
		//change to opencv's coordinate
		drawPolygone[i].x = warpedBoardPolygone[i].x;
		drawPolygone[i].y = warpedBoardPolygone[i].y;
//		drawPolygone[i].y = img_gray.rows - warpedBoardPolygone[i].y;
	}

	fillConvexPoly(mask, drawPolygone, 255);

	return mask.clone();
}

void PointDetection::matchPhysicalPointSet()
{
	detect(img_gray, boardPtsInScene);

	/**
	 * Draw detected point in the background image
	 */
	for (unsigned int i = 0 ; i < boardPtsInScene.size() ; i++)
	{
		circle(img_to_show,
//				cv::Point2d(boardPtsInScene[i].x, img_to_show.rows - boardPtsInScene[i].y),
				boardPtsInScene[i],
				5,
				cv::Scalar(1, 210, 250),
				2,
				8,
				0 );
	}

	Chronometer timer;
	timer.tic();
	int modelID;
	vector<pcl::PointXY> convertedPoints = convertPointType<pcl::PointXY>(boardPtsInScene);
	correspPhysicalPoints.clear();
	latestHomoCam = lgcalgo->setImgPointsAndFindOneModel(&convertedPoints, NULL, modelID, &correspPhysicalPoints);

	cout<<"First matching used "<<timer.tac()<<" ms; Found "<<correspPhysicalPoints.size()<<" correspondances in model "<<modelID<<endl;

	if (!latestHomoCam.empty())
	{
		drawMatchedPoints(img_to_show, cv::Scalar(255, 0, 0),
				*physicalPatternPtr,
				correspPhysicalPoints,
				latestHomoCam);
	}
}

void PointDetection::matchProjectedPointSet()
{
	detect(255 - img_gray, projectedPtsInScene);

	/**
	 * Draw detected point in the background image
	 */
	for (unsigned int i = 0 ; i < projectedPtsInScene.size() ; i++)
	{
		circle(img_to_show,
//				cv::Point2d(projectedPtsInScene[i].x, img_to_show.rows - projectedPtsInScene[i].y),
				projectedPtsInScene[i],
				5,
				cv::Scalar(47,18,251),
				2,
				8,
				0 );
	}

	/**
	 * Matching projected pattern
	 */
	Chronometer timer;
	timer.tic();
	int modelID;
	vector<pcl::PointXY> convertedPoints = convertPointType<pcl::PointXY>(projectedPtsInScene);
	correspProjectedPoints.clear();
	latestHomoProjectorUnwraped = lgcalgo->setImgPointsAndFindOneModel(&convertedPoints, NULL, modelID, &correspProjectedPoints);
	cout<<"Second matching used "<<timer.tac()<<" ms; Found "<<correspProjectedPoints.size()<<" correspondances in model "<<modelID<<endl;

	if (!latestHomoProjectorUnwraped.empty())
	{
		//Draw matched points, with estimated homography
		drawMatchedPoints(img_to_show, cv::Scalar(0, 50, 0),
				*unwrapedProjectedPatternPtr,
				correspProjectedPoints,
				latestHomoProjectorUnwraped);
	}

}
