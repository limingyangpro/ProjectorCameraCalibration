/*
 * calibenvironment.h
 *
 *  Created on: Nov 3, 2015
 *      Author: liming
 */

#ifndef CALIBENVIRONMENT_H_
#define CALIBENVIRONMENT_H_

#include "headers.h"
#include <osg/Texture2D>
#include <osgText/Text>
#include <osgViewer/CompositeViewer>



class BoardTracker;
class CalibrationEnvironment;

class UpdateTexture : public osg::NodeCallback
{
private:
	void updateBackgroundTexture(osg::Node* textureNode);
	void updateDetectedPoints(osg::Geode* detectedPointNode);

public:
	UpdateTexture(cv::VideoCapture &capture, CalibrationEnvironment* calibenv);
	virtual ~UpdateTexture();

	virtual void operator()( osg::Node* node, osg::NodeVisitor* nv );

   AbstractKPDetector* detector;
   cv::VideoCapture videoStream;
   cv::Mat cvImg;
   CalibrationEnvironment* calibEnvironment;
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
	UpdateProjection(CalibrationEnvironment* calibenv);
	virtual ~UpdateProjection();
	virtual void operator()( osg::Node* node, osg::NodeVisitor* nv );

   cv::Mat cvProjectedImg;
   int ptSize;
   CalibrationEnvironment* calibEnvironment;
};

/**
 * This class is used for dynamically changing the projected pattern
 */
class UpdateStatus : public osg::NodeCallback
{
private:
	void updateStatusBar(osg::Node* textNode);

public:
	UpdateStatus(BoardTracker* tracker, float width, float height);
	virtual ~UpdateStatus();
	virtual void operator()( osg::Node* node, osg::NodeVisitor* nv );

private:
    float barHeight;
    float barWidth;
	cv::Mat statusBar;
	BoardTracker* boardTrackerPtr;
};


class CalibrationData;
class RawData;

class CalibrationEnvironment
{

public:
	CalibrationEnvironment();
	CalibrationEnvironment(int pointsize, int pointnumber);
	~CalibrationEnvironment();

	/**
	 * Initialization of the calibration environment:
	 * - Obtain both screen and projector resolutions (sizes)
	 * - Load point pattern on the calibration board
	 * - Generate the projected point pattern.
	 * - Initialization matching algorithm
	 */
	void initialization(int numAquisition,
			const string &boardpatternfilename,
			const string &projectpatternfilename,
			const cv::Size &camSize,
			const string & outputFileName,
			const bool &record);

	static osg::ref_ptr<osgViewer::View> createView( int x, int y, int w, int h, osg::Node* Camera );
	static void getScreenResolution(int screenID,  unsigned int &width, unsigned int &height);

	osg::ref_ptr<osgViewer::View> createStatusView(float camWidth, float camHeight);
	osg::ref_ptr<osgViewer::View> createProjectorView();
	osg::ref_ptr<osgViewer::View> createCameraView(cv::VideoCapture &vCapture, float camWidth, float camHeight);
	cv::Mat matchPointSet(const cv::Mat &srcImg);

	/**
	 * Read point pattern from specific file
	 */
	static bool readPointPatternFromFile(const string &patternfilename, vector<cv::Point2d>& oPoints, cv::Size &sz);

	void startCalcul();
	bool stop();
	void setStopFlag(bool flag);

	bool recordVideo;                             /**< Record the video and the projected pattern for future use >*/
	string resultFileDir;                         /**< File contains calibrated results >*/
	/**
	 * Create the projected image according to the pattern
	 */
	cv::Mat createProjectedImage();

	int getProjectionStatus();
	void changeProjectionStatus();
	Chronometer synchroniser;
	enum
	{
		NEW_PATTERN_TO_PROJECT = 0,
		NEW_PATTERN_PROJECTED = 1
	};


private:
	static osg::ref_ptr<osg::Geometry> createPrimitiveGeometry(const int drawType, const vector<cv::Point2f>& pointVector, const osg::Vec4& color);
	static osg::ref_ptr<osg::Node> createGeodes(float camWidth, float camHeight);
	static osg::ref_ptr<osgText::Text> createText( const osg::Vec3& pos, const std::string& content, float size );
	/**
	 * Find correct screen and projector resolution
	 */
	bool getScreenSizes();


	/**
	 * Write point pattern to specific file
	 */
	bool saveProjectedPatternToFile(const string &patternfilename);

	/**
	 * Generate the projected point pattern and register them in the matching algorithm
	 */
	void generateProjectedPattern();

	void changeProjectedPointSize();

	RawData* rawDataPtr;
	BoardTracker* boardTrackerPtr;

	int projection_status;

	vector<cv::Rect> screenRect;
	unsigned int projectorID, screenID;
	unsigned int projector_width, projector_height;  /**<Projector resolution >*/
//	cv::Mat_<unsigned char> img;   /**<projected image>*/
	float ptsize;   /**<projected point size >*/
	float sizeRatio;
//	int ptnumber; /**<projected point number >*/
	vector<cv::Point2d> unwrapedProjectedPattern;
	vector<cv::Point2d> *projectedPatternPtr;        /**< Pointing to Vector contains projected pattern (perpsective transformed) >*/
	vector<bool> projectedPatternMask;              /**< Mask for each projected point, if false (not masked), the point will be shown >*/
	vector<cv::Point2d> physicalPattern;            /**< Vector contains board pattern >*/
//	cv::Size physicalBoardSize;          /**< Vector contains 4 corners of the physical board >*/

//	int ptsize;
	bool stopAcquisition;

};


#endif /* CALIBENVIRONMENT_H_ */
