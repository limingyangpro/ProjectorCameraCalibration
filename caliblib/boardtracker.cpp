/*
 * boardtracker.cpp
 *
 *  Created on: Dec 11, 2015
 *      Author: liming
 */

#include "boardtracker.h"
#include "calibdata.h"
#include "calibenvironment.h"

BoardTracker::BoardTracker()
: 	PointDetection(),
	rawDataPtr(0),
	hold_on_ms(1000),
	intialPatternRatio(1.0)
{

	latestPrewrapProjector = cv::Mat::eye(3,3,CV_64F);   //default is no prewrap
}

BoardTracker::BoardTracker(float ratio)
: 	PointDetection(),
	rawDataPtr(0),
	hold_on_ms(1000),
	intialPatternRatio(ratio)
{

	latestPrewrapProjector = cv::Mat::eye(3,3,CV_64F);   //default is no prewrap
	latestPrewrapProjector[0][0] = intialPatternRatio;
	latestPrewrapProjector[1][1] = intialPatternRatio;
}

BoardTracker::~BoardTracker()
{
}

void BoardTracker::intializeDetector()
{
	detector = new SimplePointDetector();
}

void BoardTracker::initialize(int numView,
				CalibrationEnvironment *calibenv,
				RawData* rd,
				const vector<cv::Point2d>* prjPtr,
				vector<bool>* maskPtr)
{
	numAcquisition = numView;
	rawDataPtr = rd;
	calibEnvironment = calibenv;
	setOrignalPointPatterns(rawDataPtr->physicalPatternPtr, rawDataPtr->unwrapedprojectedPatternPtr);
	setProjectedPatterns(prjPtr);

	steadyChecker.setPtNum(physicalPatternPtr->size());

	PointDetection::initialize();
}

void BoardTracker::detect(const cv::Mat_<uchar> &iImg, vector<cv::Point2d> &oDetectedPts)
{
	vector<float> oPointDiameters;
	if (latestHomoCam.empty())
	{
		detector->detect(iImg, oDetectedPts, oPointDiameters);          //Contains detected points with top-left origin
	}
	else  //detect only in board area
	{
		cv::Mat boarderArea(iImg.rows, iImg.cols, CV_8UC1, cv::Scalar(255));
		cv::Mat mask = getBoardAreaMask(img_gray, rawDataPtr->getBoardSize(), latestHomoCam);

		cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 5, 5 ));
		cv::morphologyEx(mask, mask, cv::MORPH_ERODE, element);

		iImg.copyTo(boarderArea, mask);
		detector->detect(boarderArea, oDetectedPts, oPointDiameters);
	}

	//Invert point coordinates
//	for (unsigned int i = 0 ; i < oDetectedPts.size() ; i++)
//	{
//		oDetectedPts[i].y = iImg.rows - oDetectedPts[i].y;
//	}
}

float BoardTracker::getElapsedTime() const
{
	double diff_in_ms = tm.tac();
	return (diff_in_ms/hold_on_ms);
}

float BoardTracker::getCompleteRatio() const
{
	float registeredNum = (float)rawDataPtr->getSize();
	return (registeredNum/numAcquisition);
}

bool BoardTracker::arePointsAligned() const
{
	//Check if latestHomoCam and latestHomoProjectorUnwraped are similar
	//Use two homos to transform physical points
	vector<cv::Point2d> transformedPt1, transformedPt2;
	cv::perspectiveTransform(*physicalPatternPtr, transformedPt1, latestHomoCam);
	cv::perspectiveTransform(*physicalPatternPtr, transformedPt2, latestHomoProjectorUnwraped);

	double sumDist = 0.0f;
	for (unsigned int i = 0 ; i < physicalPatternPtr->size() ; i++)
	{
		sumDist += cv::norm(transformedPt1[i] - transformedPt2[i]);
	}

	double meanErr = sumDist/physicalPatternPtr->size();
	cout<<"Error of Homographies = "<<meanErr<<endl;
	return (meanErr < 2.0f);
}

cv::Mat BoardTracker::matchPointSet(const cv::Mat &srcImg)
{
	/**
	 * Set the background texture
	 */
	img_to_show = srcImg.clone();
	cv::cvtColor( srcImg, img_gray, CV_BGR2GRAY );

	//Matching
	matchPhysicalPointSet();
	if (!latestHomoCam.empty())
	{
		matchProjectedPointSet();
	}

	/**
	 * Record result
	 */
//	int down_counting = 3;
//	static Chronometer time_counter;
//	static double hold_on_time = 2000;   //ms
	cv::Mat instruction(img_to_show.rows, img_to_show.cols, CV_8UC3);
	instruction.setTo(0);
	std::stringstream ss;
	float instructionWeight = 0.35;

    ss << "Got "<<rawDataPtr->getSize()<<" (B:"<<correspPhysicalPoints.size()<<"; P:"<<correspProjectedPoints.size()<<")";
	putText(instruction, ss.str(), cvPoint(10,50), cv::FONT_HERSHEY_PLAIN, 3, cvScalar(255,255,255), 3, CV_AA);
	ss.str("");

	if ((!correspPhysicalPoints.empty())&&(!correspProjectedPoints.empty()))
	{
		if (calibEnvironment->getProjectionStatus() == CalibrationEnvironment::NEW_PATTERN_PROJECTED)
		{
			if (calibEnvironment->synchroniser.tac() > 700)
			{
				if (!steadyChecker.isChecking())
				{
					steadyChecker.setRefPtSet(boardPtsInScene, correspPhysicalPoints);
					tm.tic();
				}
				else
				{
					if (steadyChecker.isSteady(boardPtsInScene, correspPhysicalPoints)&&arePointsAligned())
					{
						if (getElapsedTime() > 1.0f)
						{
							cout<<"Image captured !"<<endl;
							//renew state
							steadyChecker.clearRefPtSet();
							latestPrewrapProjector = latestPrewrapProjector*latestHomoProjectorUnwraped.inv()*latestHomoCam;
							calibEnvironment->changeProjectionStatus();
							tm.tic();
							rawDataPtr->addOneObservation(
									srcImg,
									latestHomoCam,
									latestHomoProjectorUnwraped,
									boardPtsInScene,
									correspPhysicalPoints,
									projectedPatternPtr,
									projectedPtsInScene,
									correspProjectedPoints);

							instructionWeight = 0.8;
						}
					}
					else
					{
						tm.tic();
						steadyChecker.clearRefPtSet();
						latestPrewrapProjector = latestPrewrapProjector*latestHomoProjectorUnwraped.inv()*latestHomoCam;
						calibEnvironment->changeProjectionStatus();
					}
				}
			}
			else
			{
				tm.tic();
			}
		}
		else
		{
			tm.tic();
		}
	}
	else
	{
		//If nothing found, reset the projection pattern
		latestPrewrapProjector = cv::Mat::eye(3,3, CV_64F);
		latestPrewrapProjector[0][0] = intialPatternRatio;
		latestPrewrapProjector[1][1] = intialPatternRatio;
		steadyChecker.clearRefPtSet();
		calibEnvironment->changeProjectionStatus();
		tm.tic();
	}


	addWeighted( img_to_show, 1-instructionWeight, instruction, instructionWeight, 0.0, img_to_show);
	return img_to_show;
}


/**
 * =================================PointSetSteadyChecker==============================
 */

PointSetSteadyChecker::PointSetSteadyChecker(int setSize) : num(setSize), threshold(3)
{

}

PointSetSteadyChecker::PointSetSteadyChecker() : num(0), threshold(3)
{

}

void PointSetSteadyChecker::setPtNum(int n)
{
	num = n;
}

void PointSetSteadyChecker::clearRefPtSet()
{
	refPtVector.clear();
}

void PointSetSteadyChecker::rearrangeInModelOrder(vector<cv::Point2d> &ptVector, const vector< pair<int, pair<int,int> > > &corresp)
{
	vector<cv::Point2d> newOrderVector(num, cv::Point2d(-9999,-9999));  //initialize with negative number, since image points should be always positive
	for (int i = 0 ; i < corresp.size() ; i++)
	{
		assert(corresp[i].second.first < num);
		newOrderVector[corresp[i].second.first] = ptVector[corresp[i].second.second];
	}
	ptVector.swap(newOrderVector);
}

void PointSetSteadyChecker::setRefPtSet(const vector<cv::Point2d> &ptVector, const vector< pair<int, pair<int,int> > > &corresp)
{
	refPtVector = ptVector;
	rearrangeInModelOrder(refPtVector, corresp);
}

bool PointSetSteadyChecker::isChecking()
{
	return (!refPtVector.empty());
}

bool PointSetSteadyChecker::isSteady(const vector<cv::Point2d> &newptVector, const vector< pair<int, pair<int,int> > > &corresp)
{
	if (isChecking())
	{
		vector<cv::Point2d> newpts = newptVector;
		rearrangeInModelOrder(newpts, corresp);
		double maxDist = 0;
		for (unsigned int i = 0 ; i < num ; i++)
		{
			if ((newpts[i].x < 0)||refPtVector[i].x < 0)
				break;
			if (cv::norm(newpts[i] - refPtVector[i]) > maxDist)
			{
				maxDist = cv::norm(newpts[i] - refPtVector[i]);
			}
		}

		cout<<"Max point movement = "<<maxDist<<" pixels."<<endl;
		if (maxDist > threshold)
		{
			return false;
		}
		else
		{
			// If the movement of all detected points are under
			return true;
		}
	}
	else
	{
		return false;
	}
}

