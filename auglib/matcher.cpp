/*
 * matcher.cpp
 *
 *  Created on: Feb 28, 2016
 *      Author: liming
 */

#include "matcher.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include "../lgc/include/affine2D.hpp"
#include "../lgc/include/perspective2D.hpp"
#include "../detectionlib/detector.hpp"

namespace augmentation {


void distortPoint(const cv::Point2d & src, cv::Point2d & dst,
                     const cv::Mat & cameraMatrix, const cv::Mat & distorsionMatrix)
{
  double fx = cameraMatrix.at<double>(0,0);
  double fy = cameraMatrix.at<double>(1,1);
  double ux = cameraMatrix.at<double>(0,2);
  double uy = cameraMatrix.at<double>(1,2);

  double k1 = distorsionMatrix.at<double>(0, 0);
  double k2 = distorsionMatrix.at<double>(0, 1);
  double p1 = distorsionMatrix.at<double>(0, 2);
  double p2 = distorsionMatrix.at<double>(0, 3);
  double k3 = distorsionMatrix.at<double>(0, 4);

  double x = (src.x - ux)/fx;
  double y = (src.y - uy)/fy;
  double xCorrected, yCorrected;
  //Step 1 : correct distorsion
  {
    double r2 = x*x + y*y;
    //radial distorsion
    xCorrected = x * (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
    yCorrected = y * (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

    //tangential distorsion
    //The "Learning OpenCV" book is wrong here !!!
    //False equations from the "Learning OpenCv" book below :
    //xCorrected = xCorrected + (2. * p1 * y + p2 * (r2 + 2. * x * x));
    //yCorrected = yCorrected + (p1 * (r2 + 2. * y * y) + 2. * p2 * x);
    //Correct formulae found at : http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
    xCorrected = xCorrected + (2. * p1 * x * y + p2 * (r2 + 2. * x * x));
    yCorrected = yCorrected + (p1 * (r2 + 2. * y * y) + 2. * p2 * x * y);
  }
  //Step 2 : ideal coordinates => actual coordinates
  {
    xCorrected = xCorrected * fx + ux;
    yCorrected = yCorrected * fy + uy;
  }
  dst = cv::Point2d(xCorrected, yCorrected);

}


void distortPoints(const vector< cv::Point2d > & src, vector< cv::Point2d > & dst, const cv::Mat & cameraMatrix, const cv::Mat & distorsionMatrix)
{
	dst.resize(src.size());
	for (unsigned int i = 0 ; i < src.size() ; i++)
	{
		distortPoint(src[i], dst[i], cameraMatrix, distorsionMatrix);
	}
}

template<typename TypePoint2D>
vector<TypePoint2D> convertPointType(const vector<cv::Point2d> &iPoints) {
	vector<TypePoint2D> result(iPoints.size());
	for (int i = 0; i < iPoints.size(); i++) {
		result[i].x = (float) iPoints[i].x;
		result[i].y = (float) iPoints[i].y;
	}

	return result;
}

bool LGCMatcher::readRefPointsFromFile(string filename) {
	ifstream file(filename.c_str());
	if (!file.is_open())         //Check if the corresponding file exists
	{
		std::cout << "Fail to open file " << filename << std::endl;
		return false;
	}

	int x_min, y_min, x_max, y_max;
	file >> x_min >> y_min >> x_max >> y_max;
	if ((x_min != 0) || (y_min != 0)) {
		cout << "Warning: the min of point pattern is not zero !" << endl;
	}

	int totalpoints;
	file >> totalpoints;

	referencePattern.resize(totalpoints);
	//Always top left as origin
	for (int i = 0; i < totalpoints; i++) {
		file >> referencePattern[i].x >> referencePattern[i].y;
//		oPoints[i].y = sz.height - oPoints[i].y;     //coordinates conversion
	}
	file.close();

	return true;
}

void LGCMatcher::initialize() {
	detector = new SimplePointDetector();
	lgc2::Affine2D* affinetrans = new lgc2::Affine2D(1.3f, 10.0f);
	lgc2::Perspective2D* persptrans = new lgc2::Perspective2D();
	lgc2::Parameter* lgcparam = new lgc2::Parameter(6, 6,   //kr, ki
			0.03,   //eta
			10, 1000, 1.3, 100, 2, 0.1, 45, 20, 8, 4);
	lgc2::Generator2Daff* generator2Daff = new lgc2::Generator2Daff(lgcparam,
			affinetrans);
	lgcalgo = new lgc2::LGCmethod<pcl::PointXY>(lgcparam, generator2Daff,
			affinetrans, persptrans);

	{
		//Register board pattern
		vector<pcl::PointXY> convertedPoints = convertPointType<pcl::PointXY>(referencePattern);
		lgcalgo->setRefPoints(&convertedPoints);
	}
}

LGCMatcher::LGCMatcher(string filename)
: detector(0),
  lgcalgo(0)
{
	readRefPointsFromFile(filename);
	initialize();
}

LGCMatcher::~LGCMatcher()
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

cv::Mat LGCMatcher::calHomography(const cv::Mat &srcImg, cv::Mat_<double> &homo)
{
	cv::Mat img_to_show = srcImg.clone();
	cv::Mat img_gray;
	cv::cvtColor(srcImg, img_gray, CV_BGR2GRAY);

	vector<float> boardPtsDiameters;
	vector<cv::Point2d> oDetectedPts;
	vector<float> oPointDiameters;
	detector->detect(img_gray, oDetectedPts, oPointDiameters);

	/**
	 * Draw detected point in the background image
	 */
	for (unsigned int i = 0; i < oDetectedPts.size(); i++) {
		cv::circle(img_to_show,
//				cv::Point2d(boardPtsInScene[i].x, img_to_show.rows - boardPtsInScene[i].y),
				oDetectedPts[i],
				5,
				cv::Scalar(1, 210, 250),
				2,
				8,
				0 );
	}

	Chronometer timer;
	timer.tic();
	int modelID;
	vector<pcl::PointXY> convertedPoints = convertPointType<pcl::PointXY>(
			oDetectedPts);
	vector<pair<int, pair<int, int> > > correspPhysicalPoints(0);
	homo = lgcalgo->setImgPointsAndFindOneModel(&convertedPoints, NULL,
			modelID, &correspPhysicalPoints);
	if (!homo.empty())
	{
		homo = homo / homo.at<double>(2, 2);
		cout << "First matching used " << timer.tac() << " ms; Found "
				<< correspPhysicalPoints.size() << " correspondances in model "
				<< modelID << endl;
	}

	return img_to_show;
}

ChessBoardMatcher::ChessBoardMatcher(string filename)
{
	if (!readRefPointsFromFile(filename))
	{
		exit(1);
	}
}


bool ChessBoardMatcher::readRefPointsFromFile(string filename) {
	ifstream file(filename.c_str());
	if (!file.is_open())         //Check if the corresponding file exists
	{
		std::cout << "Fail to open file " << filename << std::endl;
		return false;
	}

	int x_min, y_min, x_max, y_max;
	file >> x_min >> y_min >> x_max >> y_max;
	if ((x_min != 0) || (y_min != 0)) {
		cout << "Warning: the min of point pattern is not zero !" << endl;
	}

	file >> colNum >> rowNum;
	int totalpoints = rowNum*colNum;

	referencePattern.resize(totalpoints);
	//Always top left as origin
	for (int i = 0; i < totalpoints; i++) {
		file >> referencePattern[i].x >> referencePattern[i].y;
//		oPoints[i].y = sz.height - oPoints[i].y;     //coordinates conversion
	}
	file.close();

	return true;
}

cv::Mat ChessBoardMatcher::calHomography(const cv::Mat &srcImg, cv::Mat_<double> &homo) {

	cv::Mat img_to_show = srcImg.clone();
	cv::Mat img_gray;
	cv::cvtColor( srcImg, img_gray, CV_BGR2GRAY );

	vector<cv::Point2f> detectedcorners;
	bool found = cv::findChessboardCorners( img_gray, cv::Size(colNum, rowNum), detectedcorners,
	                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

	if (found)
	{
		for (unsigned int i = 0 ; i < detectedcorners.size() ; i++)
		{
			cout<<detectedcorners[i]<<endl;
		}
		homo = cv::findHomography(referencePattern, detectedcorners);
		cout<<homo<<endl;
	}

	return img_to_show;

}

AcircleGridMatcher::AcircleGridMatcher(string filename)
{
	readRefPointsFromFile(filename);
}

bool AcircleGridMatcher::readRefPointsFromFile(string filename) {
	ifstream file(filename.c_str());
	if (!file.is_open())         //Check if the corresponding file exists
	{
		std::cout << "Fail to open file " << filename << std::endl;
		return false;
	}

	int x_min, y_min, x_max, y_max;
	file >> x_min >> y_min >> x_max >> y_max;
	patternSz.width = x_max;
	patternSz.height = y_max;
	if ((x_min != 0) || (y_min != 0)) {
		cout << "Warning: the min of point pattern is not zero !" << endl;
	}

	int totalpoints;
	file >> colNum >> rowNum;
	file >> totalpoints;
	referencePattern.resize(totalpoints);

	//Always top left as origin
	for (int i = 0; i < totalpoints; i++) {
		file >> referencePattern[i].x >> referencePattern[i].y;
//		oPoints[i].y = sz.height - oPoints[i].y;     //coordinates conversion
	}
	file.close();

	return true;
}

///**
// * OpenCV blob detection works poorly, we need to rewrite this fuction :
// */
//bool findCirclesGrid( cv::InputArray _image, cv::Size patternSize,
//                          cv::OutputArray _centers, int flags, const vector<cv::Point2f> points )
//{
//    bool isAsymmetricGrid = (flags & cv::CALIB_CB_ASYMMETRIC_GRID) ? true : false;
//    bool isSymmetricGrid  = (flags & cv::CALIB_CB_SYMMETRIC_GRID ) ? true : false;
//    CV_Assert(isAsymmetricGrid ^ isSymmetricGrid);
//
//    cv::Mat image = _image.getMat();
//    std::vector<cv::Point2f> centers;
//
//    if(flags & cv::CALIB_CB_CLUSTERING)
//    {
//    	//The utility is not implemented
//    	return false;
//    }
//
//    cv::CirclesGridFinderParameters parameters;
//    parameters.vertexPenalty = -0.6f;
//    parameters.vertexGain = 1;
//    parameters.existingVertexGain = 10000;
//    parameters.edgeGain = 1;
//    parameters.edgePenalty = -0.6f;
//
//    if(flags & cv::CALIB_CB_ASYMMETRIC_GRID)
//      parameters.gridType = cv::CirclesGridFinderParameters::ASYMMETRIC_GRID;
//    if(flags & cv::CALIB_CB_SYMMETRIC_GRID)
//      parameters.gridType = cv::CirclesGridFinderParameters::SYMMETRIC_GRID;
//
//    const int attempts = 2;
//    const size_t minHomographyPoints = 4;
//    cv::Mat H;
//    for (int i = 0; i < attempts; i++)
//    {
//      centers.clear();
//      cv::CirclesGridFinder boxFinder(patternSize, points, parameters);
//      bool isFound = false;
//#define BE_QUIET 1
//      try
//      {
//        isFound = boxFinder.findHoles();
//      }
//      catch (const cv::Exception &)
//      {
//
//      }
//      if (isFound)
//      {
//        switch(parameters.gridType)
//        {
//          case CirclesGridFinderParameters::SYMMETRIC_GRID:
//            boxFinder.getHoles(centers);
//            break;
//          case CirclesGridFinderParameters::ASYMMETRIC_GRID:
//			boxFinder.getAsymmetricHoles(centers);
//			break;
//          default:
//            CV_Error(CV_StsBadArg, "Unkown pattern type");
//        }
//
//        if (i != 0)
//        {
//          Mat orgPointsMat;
//          transform(centers, orgPointsMat, H.inv());
//          convertPointsFromHomogeneous(orgPointsMat, centers);
//        }
//        Mat(centers).copyTo(_centers);
//        return true;
//      }
//
//      boxFinder.getHoles(centers);
//      if (i != attempts - 1)
//      {
//        if (centers.size() < minHomographyPoints)
//          break;
//        H = CirclesGridFinder::rectifyGrid(boxFinder.getDetectedGridSize(), centers, points, points);
//      }
//    }
//    Mat(centers).copyTo(_centers);
//    return false;
//}

cv::Mat AcircleGridMatcher::calHomography(const cv::Mat &srcImg, cv::Mat_<double> &homo) {

	cv::Mat intermidiateImg = cv::Mat::zeros( srcImg.rows, srcImg.cols, CV_8UC3);
	intermidiateImg.setTo(255);
	cv::Mat img_to_show = srcImg.clone();
	cv::Mat img_gray;
	cv::cvtColor( srcImg, img_gray, CV_BGR2GRAY );

	//Use MSER detector, which is much more stable than simple point detector
	vector<cv::Point2d> candidates;
	vector<float> dias;
	MSERDetector* detector = new MSERDetector();
	detector->detect(img_gray, candidates, dias);

	for (unsigned int i = 0 ; i < candidates.size() ; i++)
	{
		circle(intermidiateImg,
//				cv::Point2d(projectedPtsInScene[i].x, img_to_show.rows - projectedPtsInScene[i].y),
				candidates[i],
				5,
				cv::Scalar(0,0,0),
				-1,
				8,
				0 );
	}
//
//	imshow("temp", intermidiateImg);
//	cv::waitKey(-1);


	cv::SimpleBlobDetector::Params detectorparams;
	detectorparams.filterByColor = true;        //Find black points
	detectorparams.blobColor = 0;

	detectorparams.minThreshold = 5;          //Detect black points, if maxThreshold is too big, it may fail to detect points
	detectorparams.maxThreshold = 100;
	detectorparams.thresholdStep = 10;

	detectorparams.minDistBetweenBlobs = 0.0;  // minimum 10 pixels between blobs

	detectorparams.filterByArea = true;
	detectorparams.minArea = 5.0;              				// min 5 pixels squared
	detectorparams.maxArea = 200*200*4;             // max 200 pixels squared

	//Without specifying "minConvexity, some floued points can be missed
	detectorparams.filterByCircularity = false;
	detectorparams.filterByInertia = false;
	detectorparams.minInertiaRatio = 0.01;
	detectorparams.filterByConvexity = false;         // filter  by area of blob
	detectorparams.minConvexity = 0.1;              // min 5 pixels squared

	cv::Ptr< cv::SimpleBlobDetector > blobDetector = cv::SimpleBlobDetector::create(detectorparams);

//	vector<cv::KeyPoint> kps;
//	blobDetector->detect(intermidiateImg, kps);
//	drawKeypoints(srcImg, kps, img_to_show);

//	vector<int> compression_params;
//	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
//	compression_params.push_back(0);
//
//	imwrite("example.png", srcImg, compression_params);
//	imshow("detection", img_to_show);
//	cv::waitKey(-1);

	vector<cv::Point2f> detectedCircles;
	bool found = cv::findCirclesGrid( intermidiateImg, cv::Size(rowNum/2, colNum), detectedCircles, cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetector  );

	if (found)
	{
//		for (unsigned int i = 0 ; i < detectedCircles.size() ; i++)
//		{
//			cout<<detectedCircles[i]<<endl;
//		}
		homo = cv::findHomography(referencePattern, detectedCircles);
		cout<<homo<<endl;

		//Improve the detection
		cv::Mat frontView, frontView_gray;
		cv::warpPerspective(srcImg, frontView, homo.inv(), patternSz, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 255);

		//No problem with pyrDown, since big picure is only an interpolation
		int scale = 1;
		while (patternSz.width/scale > srcImg.cols)
		{
			cv::pyrDown(frontView, frontView);
			scale = scale*2;
		}

		vector<cv::Point2d> improvedDetectedCircles_raw;
		vector<float> pointDiameter;
		MSERDetector* detector = new MSERDetector();
		cv::cvtColor(frontView, frontView_gray, CV_BGR2GRAY);
		detector->detect(frontView_gray, improvedDetectedCircles_raw, pointDiameter);
		cout<<"Find homography for phsycal patterns ... found "<<improvedDetectedCircles_raw.size()<<" raw circles."<<endl;
//		for (unsigned int i = 0 ; i < improvedDetectedCircles_raw.size() ; i++)
//		{
//			circle(frontView,
//	//				cv::Point2d(projectedPtsInScene[i].x, img_to_show.rows - projectedPtsInScene[i].y),
//					improvedDetectedCircles_raw[i],
//					5,
//					cv::Scalar(47,18,251),
//					2,
//					8,
//					0 );
//		}
//		imshow("temp", frontView);
//		cv::waitKey(-1);

		for (unsigned int i = 0 ; i < improvedDetectedCircles_raw.size() ; i++)
		{
			improvedDetectedCircles_raw[i] = improvedDetectedCircles_raw[i]*scale;
			pointDiameter[i] = pointDiameter[i]*scale;
		}

		vector<cv::Point2d> improvedDetectedCircles(0);
		for (unsigned int i = 0 ; i < referencePattern.size() ; i++)
		{
			for (unsigned int j = 0 ; j < improvedDetectedCircles_raw.size() ; j++)
			{
				if (norm(referencePattern[i] - improvedDetectedCircles_raw[j]) < pointDiameter[j])
				{
					improvedDetectedCircles.push_back(improvedDetectedCircles_raw[j]);
				}
			}
		}

		cv::perspectiveTransform(improvedDetectedCircles, improvedDetectedCircles, homo);
		homo = cv::findHomography(referencePattern, improvedDetectedCircles);
		cout<<homo<<endl;
	}

	return img_to_show;

}

/**
 * Result error matrix contains n rows and 4 cols, each row represents a projected point
 * x, y, dx, dy
 *
 * (x,y) is the projected point position in the projector view
 * (dx,dy) is the error (measured - groundtruth) on the marker, by milimeter ?
 *
 * trueHomo: the transformation from calibration board to camera image (calculated with control points)
 * projHomo: the transformation from calibration board to projector image (estimated from R and t)
 */
cv::Mat_<double> AcircleGridMatcher::findErrs(const cv::Mat &srcImg,
		const cv::Mat_<double> &trueHomo,
		const cv::Mat_<double> &projHomo,
		const cv::Mat_<double> &projIntrinsic,
		const cv::Mat_<double> &projDistorsion) {

//	cv::Mat img_gray;
//	cv::cvtColor( srcImg, img_gray, CV_BGR2GRAY );
//	vector<cv::Point2d> detectedCirclesInCam;
//	vector<cv::Point2d> projectedCirclesInPro;
//	bool found = cv::findCirclesGrid( 255 - img_gray, cv::Size(rowNum/2, colNum), detectedCirclesInCam, cv::CALIB_CB_ASYMMETRIC_GRID  );

	//Use wrapped image to detect projected points
	vector<cv::Point2d> projectedCirclesInPro;
	vector<cv::Point2d> detectedCircles_raw;
	vector<float> pointDiameter;
	cv::Mat frontView, frontView_gray;
	//Unwrap the projected image by camera homography
	cv::warpPerspective(srcImg, frontView, trueHomo.inv(), patternSz, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);

	//No problem with pyrDown, since big picure is only an interpolation
	int scale = 1;
	while (patternSz.width/scale > srcImg.cols)
	{
		cv::pyrDown(frontView, frontView);
		scale = scale*2;
	}

	MSERDetector* detector = new MSERDetector();
	cv::cvtColor(frontView, frontView_gray, CV_BGR2GRAY);
	detector->detect(255 - frontView_gray, detectedCircles_raw, pointDiameter);

//	for (unsigned int i = 0 ; i < detectedCircles_raw.size() ; i++)
//	{
//		circle(frontView,
////				cv::Point2d(projectedPtsInScene[i].x, img_to_show.rows - projectedPtsInScene[i].y),
//				detectedCircles_raw[i],
//				5,
//				cv::Scalar(47,18,251),
//				2,
//				8,
//				0 );
//	}
//
//	imshow("temp", frontView);
//	cv::waitKey(-1);

	//May be improved here
	for (unsigned int i = 0 ; i < detectedCircles_raw.size() ; i++)
	{
		detectedCircles_raw[i] = detectedCircles_raw[i]*scale;
		pointDiameter[i] = pointDiameter[i]*scale;
	}

	vector<cv::Point2d> projectedUnwraped(0);
	for (unsigned int i = 0 ; i < referencePattern.size() ; i++)
	{
		for (unsigned int j = 0 ; j < detectedCircles_raw.size() ; j++)
		{
			if (norm(referencePattern[i] - detectedCircles_raw[j]) < pointDiameter[j])  //more precisely, need to find the nearest point
			{
				projectedUnwraped.push_back(detectedCircles_raw[j]);
			}
		}
	}

//	cv::perspectiveTransform(detectedCircles, detectedCirclesInCam, trueHomo);
//	homo = cv::findHomography(referencePattern, detectedCircles);

	bool found = (referencePattern.size() == projectedUnwraped.size());
	if (found)
	{
		vector<cv::Point2d> projectedPosOntheMarker = projectedUnwraped;
//		//Now, wrap the projected points to cameraview
//		vector<cv::Point2d> detectedCirclesInCam;
//		cv::perspectiveTransform(projectedUnwraped, detectedCirclesInCam, trueHomo.inv());
//		//Then, Transform detected circle (in camera image plane) to board plan via trueHomo
//		vector<cv::Point2d> projectedPosOntheMarker;
//		cv::perspectiveTransform(detectedCirclesInCam, projectedPosOntheMarker, trueHomo.inv());

		cv::Mat_<double> result = cv::Mat::zeros(referencePattern.size(), 4, CV_64F);
		//Transform ground truth with projector homograpy to get all points positions in projector view
		cv::perspectiveTransform(referencePattern, projectedCirclesInPro, projHomo);
		distortPoints(projectedCirclesInPro, projectedCirclesInPro, projIntrinsic, projDistorsion);

		for (unsigned int i = 0 ; i < referencePattern.size() ; i++)
		{
			result[i][0] = projectedCirclesInPro[i].x;   //point in projector image (pixel)
			result[i][1] = projectedCirclesInPro[i].y;
			result[i][2] = projectedPosOntheMarker[i].x - referencePattern[i].x;
			result[i][3] = projectedPosOntheMarker[i].y - referencePattern[i].y;
			double dx = referencePattern[i].x - projectedPosOntheMarker[i].x;
			double dy = referencePattern[i].y - projectedPosOntheMarker[i].y;
		}


		return result.clone();
	}
	else
	{
		return cv::Mat();
	}

}


}
