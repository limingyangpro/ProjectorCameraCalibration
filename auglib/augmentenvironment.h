/*
 * calibenvironment.h
 *
 *  Created on: Nov 3, 2015
 *      Author: liming
 */

#ifndef AUGMENTENVIRONMENT_H_
#define AUGMENTENVIRONMENT_H_

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "matcher.h"
#include <iostream>

#include <osg/Texture2D>
#include <osgText/Text>
#include <osgViewer/CompositeViewer>
#include <osgGA/GUIEventHandler>

namespace augmentation
{
void distort(const cv::Mat& src, cv::Mat& dst, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);

class AugmentationEnvironment;


//class KeyboardEventHandler : public osgGA::GUIEventHandler
//{
//private:
//	AugmentationEnvironment* augEnvironment;
//
//public:
//
//    KeyboardEventHandler(AugmentationEnvironment* augenv)  { }
//    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
//};

class UpdateCameraView : public osg::NodeCallback
{
private:
	void updateBackgroundTexture(osg::Node* textureNode);
	void updateDetectedPoints(osg::Geode* detectedPointNode);

public:
	UpdateCameraView(cv::VideoCapture &capture, AugmentationEnvironment* calibenv);
	virtual ~UpdateCameraView();

	virtual void operator()( osg::Node* node, osg::NodeVisitor* nv );

	bool imageCaptured;
	cv::VideoCapture videoStream;
	cv::Mat cvImg;
	AugmentationEnvironment* augEnvironment;
	cv::VideoWriter vWriter, vWriter2;
};

/**
 * This class is used for dynamically changing the projected pattern
 */
class UpdateProjection : public osg::NodeCallback
{
private:
	void createProjectionImage(osg::Node* textureNode);

public:
	UpdateProjection(AugmentationEnvironment* calibenv);
	virtual ~UpdateProjection();
	virtual void operator()( osg::Node* node, osg::NodeVisitor* nv );

   cv::Mat cvProjectedImg;
   int ptSize;
   AugmentationEnvironment* augEnvironment;
   cv::Mat_<double>  homo;
};

class AugmentationEnvironment
{

public:
	AugmentationEnvironment();
	AugmentationEnvironment(int pointsize, int pointnumber);
	~AugmentationEnvironment();

	bool readParametersFile(string parameterfile);

	/**
	 * Initialization of the calibration environment:
	 * - Obtain both screen and projector resolutions (sizes)
	 * - Load point pattern on the calibration board
	 * - Generate the projected point pattern.
	 * - Initialization matching algorithm
	 */
	void initialization(const string &parameterFilename,
			const string &boardpatternfilename,
			const string &grdtruthfilename,
			const string &augmentingContentFile,
			const string &errorFile,
			const bool &record);

	static osg::ref_ptr<osgViewer::View> createView( int x, int y, int w, int h, osg::Node* Camera );
	osg::ref_ptr<osgViewer::View> createProjectorView();
	osg::ref_ptr<osgViewer::View> createCameraView(cv::VideoCapture &vCapture, float camWidth, float camHeight);


	bool recordVideo;                             /**< Record the video and the projected pattern for future use >*/

	/**
	 * Calculate proporate geometric transformation for projector
	 */
	cv::Mat match(const cv::Mat &img);
	static void H2Rt(const cv::Mat_<double> intrisinc, const cv::Mat_<double>& homo, cv::Mat_<double>& rot, cv::Mat_<double> &trans);
	static void Rt2H(const cv::Mat_<double> intrisinc, const cv::Mat_<double>& rot, const cv::Mat_<double> &trans, cv::Mat_<double>& homo);
	void calculateProjectorHomo();
	void calculateReprojectionErr(string groundTruthFile, const cv::Mat &srcImg);

	cv::Size getProjectorSize();
	cv::Mat augmentingContent;
	cv::Mat_<double> latestHomoCam;

	void rematch();
	bool calculateErr;
	bool hasProjected;

	bool posFound;
	Chronometer synchroniser;
	string groundTruthFileName;

private:
	static osg::ref_ptr<osg::Geometry> createPrimitiveGeometry(const int drawType, const vector<cv::Point2f>& pointVector, const osg::Vec4& color);
	static osg::ref_ptr<osg::Node> createGeodes(float camWidth, float camHeight);
	static osg::ref_ptr<osgText::Text> createText( const osg::Vec3& pos, const std::string& content, float size );
	/**
	 * Find correct screen and projector resolution
	 */
	bool getScreenSizes();

	AbstractMatcher *matcherPtr;

	vector<cv::Rect> screenRect;
	unsigned int projectorID, screenID;
	unsigned int projector_width, projector_height;  /**<Projector resolution >*/
//	vector<cv::Point2d> physicalPattern;            /**< Vector contains board pattern >*/

public:
	string parameterFileName;
	string errorFileNameBase;
	cv::Mat_<double> cameraMatrix;
	cv::Mat_<double> cameraDistorsion;
	cv::Mat_<double> projectorMatrix;
	cv::Mat_<double> projectorDistorsion;
	cv::Mat_<double> R, t;
	cv::Mat_<double> projectorHomo;

};

}

#endif /* AUGMENTENVIRONMENT_H_ */
