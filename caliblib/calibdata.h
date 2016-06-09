/*
 * calibdata.h
 *
 *  Created on: Dec 11, 2015
 *      Author: liming
 */

#ifndef CALIBDATA_H_
#define CALIBDATA_H_

#include "headers.h"
#include <numeric>

/**< critic number of homographies for rough calibration >*/
#define CRITIC_NUMBER_OF_HOMO_FOR_ROUGH_CALIB 5

class DeviceResult
{
public:
	int calibrationFlags;                          /**< Flag parameter for openCV function >*/
	cv::Mat_<double> intrinsicMatrix;
	cv::Mat_<double> distorsionMatrix;
	cv::Mat_<double> R, T, E, F;
	vector< cv::Mat > rotations;                /**< No type specifying for this two matrix, or calibrateCamera will not work > */
	vector< cv::Mat > translations;
	vector<double> perViewErrors;
	double totalErr;
	double maxErr;

	DeviceResult();
	void calibrateDevice(const vector< vector<cv::Point3d> > &worldPoints3D, const vector< vector<cv::Point2d> > &imgPoints2D, cv::Size imgSize);
	void computeReprojectionErrors(const vector< vector<cv::Point3d> > &worldPoints3D,
			const vector< vector<cv::Point2d> > &imgPoints2D,
	        vector<double>& perViewErrors,
			double &mean_err,
			double &max_err);

	void setAsStereoOrigin();
	void calibrateStereo(const DeviceResult& baseDevice,
			const vector< vector<cv::Point3d> > &worldPoints3D,
			const vector< vector<cv::Point2d> > &imgPoints2D_baseDevice,
			const vector< vector<cv::Point2d> > &imgPoints2D_thisDevice);
	void write(cv::FileStorage& fs) const;

};

static void write(cv::FileStorage& fs, const std::string&, const DeviceResult& x);

/**
 * This struct contains all the raw information needed for the calibration
 */
class RawData
{
private:
	static cv::Mat_<uchar> getPointTemplate(int iRadius);
	static double getAvgBlobRadius(const cv::Mat_<uchar> &binaryImg, const vector<cv::Point2d> &refPts);
	void improveSingleViewDetection(const cv::Mat_<uchar> &grayImg,
			const cv::Size &dsize,
			const cv::Mat_<double> &homo,
			vector<cv::Point2d> &refPts,
			vector<cv::Point2d> &detectedPts,
			bool useTemplateMatching);
public:
	/**
	 * Set camera image size and projector image size
	 * @param camSize [Input] cv::Size contains the resolution of camera
	 * @param proSize [Input] cv::Size contains the resolution of projector
	 */
	void setAllSizes(const cv::Size &boardSize, const cv::Size &camSize, const cv::Size &proSize);

	/**
	 * Get the calibration board size;
	 */
	cv::Size getBoardSize();

	/**
	 * Set model patterns, both for pattern on the calibration board and the projected pattern.
	 * @param physicalPat [Input] A vector contains the point pattern on the calibration board.
	 * @param projectedPat [Input] A vector contains the projected point pattern.
	 */
	void setModelPatterns(const vector<cv::Point2d> &physicalPat, const vector<cv::Point2d> &unwrapedProjectedPat);

	/**
	 * This function add one series of raw data into rawdata structure
	 * @param img [Input] Current captured image by the camera
	 * @param homo [Input] Current homography estimated between the calibration board and the camera (Hbc)
	 * @param physicalPointsScene [Input] A vector contains coordinates of points on the calibration board viewed by the camera
	 * @param phsycalCorresp [Input] Point correspondences between the points on the calibration board (ref) and
	 *        their images in the camera (img), stored under form of <d, <refID, imgID> >, where d is not used here.
	 * @param projectedPointsScene [Input] A vector contains coordinates of projected points viewed by the camera
	 * @param projectedCorresp [Input] Point correspondences between the points projected by the projector (ref) and
	 *        their images in the camera (img), stored under form of <d, <refID, imgID> >, where d is not used here.
	 */
	void addOneObservation(const cv::Mat &img,
			const cv::Mat &homobc,
			const cv::Mat &homopc,
			const vector<cv::Point2d> &detectedPhysicalPoints,
			const vector< pair<int, pair<int,int> > > &phsycalCorresp,
			const vector<cv::Point2d> *projectedPointModelPtr,
			const vector<cv::Point2d> &detectedProjectedPoints,
			const vector< pair<int, pair<int,int> > > &projectedCorresp);


	/**
	 * Return how many positions have been acquired, = rawdata.recordedImage.size()
	 */
	unsigned int getSize() const;

public:
	const vector<cv::Point2d>* unwrapedprojectedPatternPtr;           /**< Pointer to Vector which contains projected pattern >*/
	const vector<cv::Point2d>* physicalPatternPtr;            /**< Pointer to Vector which contains board pattern >*/

	cv::Size physicalBoardSize;                 /**<The calibration board size     >*/
	cv::Size cameraImageSize;                   /**<The image size of the camera    >*/
	cv::Size projectorImageSize;                /**< The image size of the projector >*/

	vector< cv::Mat > recordedImage;
	vector< cv::Mat_<double> > homobcVector;                       /**< contains Hbc for each data, board -> camera >*/
	vector< cv::Mat_<double> > homopcVector;                       /**< contains Hpc for each data, projector -> camera >*/

	vector< vector<cv::Point2d> > physicalPattern2D;
	vector< vector<cv::Point2d> > physicalPointsScene;    /**< Points' coordinates in calibration board view by the camera >*/

	/**
	 * Since projector is an inverse camera, the model pattern is 2D (similar to scene pattern of a camera)
	 * and the scene pattern is 3D with z=0 (similar to model pattern of a camera)
	 *
	 * We need to undistort the scene pattern
	 */
	vector< vector<cv::Point2d> > allProjectedPts;           /**< All projected points in current view >*/
	vector< vector<cv::Point2d> > projectedPatternModel;  /**< Matched Projected points' coordinates in the projector >*/
	vector< vector<cv::Point2d> > projectedPointsScene;   /**< Matched Projected points' coordinates viewed by the camera >*/

	RawData();

	bool saveData(string filename) const;
	bool loadData(string filename);
	bool saveTextData(string fileName) const;
	bool loadTextData(string fileName);
	bool operator== (const RawData &other);

	void improveDetection();

	void checkDetection() const;
};

class CalibrationData
{
public:
	CalibrationData();
	CalibrationData(const RawData* rd);
	CalibrationData(double minOrientDiff);
	~CalibrationData();

	void setRawData(const RawData* rd);
	void calibrate();

	void printCalibrationInfo();

	/**
	 * Save parameters of the device into filename
	 */
	void saveParams(const string& filename, bool withRawdata = true);


	void improveDetection2();

protected:
	/**
	 * Use cv::calibrateCamera() to calibrate the camera
	 */
	bool calibrateCamera();

	bool calibrateProjector();

	bool stereoCalibrate();

	/**
	 * Preview the matching pairs for debuging
	 */
	cv::Mat previewPointPaires(const cv::Mat &captureImg, const cv::Size imgSize, const vector<cv::Point2d>* allModelPts, const vector<cv::Point2d>* modelPts, const vector<cv::Point2d>* imagePts) const;

	/**
	 * Undistort the points stored in rawdata.projectedPointsScene (both lens distortion and perspective distortion)
	 * This gives undistortedProjectedPointsScene.
	 */
	void undistortProjectedPoints();

	/**
	 * Save extrinsic parameter
	 */
	static void saveExtrinsicParam(cv::FileStorage& fs, const vector<cv::Mat>& rvecs, const vector<cv::Mat>& tvecs);

	const double minOrientationDifference;                   /**< The minimum orientation difference of each position of board >*/

	const RawData* rawDataPtr;

	vector< vector<cv::Point3d> > physicalPattern3D;
	vector< vector<cv::Point3d> > undistortedProjectedPointsScene;    /**< Projected points' coordinates viewed by the camera, remove camera lens' distortion and perspective distortion >*/

	DeviceResult cameraResult, projectorResult;

};



#endif /* CALIBDATA_H_ */
