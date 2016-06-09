/*
 * matchinginterface.hpp
 *
 *  Created on: Apr 3, 2015
 *      Author: liming
 */

#ifndef MATCHINGLIB_MATCHINGINTERFACE_HPP_
#define MATCHINGLIB_MATCHINGINTERFACE_HPP_

/**
 * Including basic header files
 */
#include "basictype.h"
#include "chronometer.h"

/**
 * Including head files of matching methods
 */
#include "affine2D.hpp"
#include "perspective2D.hpp"
#include "lgcmethod.hpp"


template <typename TypePoint2D>
vector<TypePoint2D> convertPointType(const vector<Point2D> &iPoints)
{
	vector<TypePoint2D> result(iPoints.size());
	for (int i = 0 ; i < iPoints.size() ; i ++)
	{
		result[i].x = iPoints[i].x;
		result[i].y = iPoints[i].y;
	}

	return result;
}

/**
 * This class defines an interface for matching points in 2D with only one model
 */
class Homography2DwithSingleModel
{
protected:
	/**
	 * Camera intrinsic matrix
	 */
	cv::Mat_<float> camMatrix;

	/**
	 * Find transformation matrix (OpenGL form) with homography
	 * @param iPoints [Input] homography
	 * @param oHomo [Output] Result transformation matrix
	 */
	bool homo2GLMatrix(const cv::Mat_<float> &iHomo, double oGL[16]);
public:
	/**
	 * Constructor with default parameters
	 * @param camMat [Input] The camera matrix, default value is an empty matrix
	 */
	Homography2DwithSingleModel(cv::Mat camMat = cv::Mat());
	virtual ~Homography2DwithSingleModel() {};

	/**
	 * Set the model point set
	 * @param iPoints [Input] 2D point set serving as model
	 */
	virtual bool setModel(const vector<Point2D> &iPoints) { return false; };

	/**
	 * Give the scene point set and find the homography between registered model set and this scene set
	 * @param iPoints [Input] 2D scene point set
	 * @param oHomo [Output] Result homography
	 * @param oCorrespondenceResultPtr [Output] A pointer to the result correspondence vector, each element contains 3 integers.
	 *       The first integer is a tag of the correspondence used in development.
	 *       The second integer is the ordering number of the model point of this correspondence,
	 *       The last integer is the ordering number of the scene point of this correspondence.
	 */
	virtual bool findHomography2D(const vector<Point2D> &iPoints, cv::Mat_<float> &oHomo, vector< pair<int, pair<int,int> > > *oCorrespondenceResultPtr = NULL) { return false; };

	/**
	 * Give the scene point set and find the transformation between registered model set and this scene set
	 * @param iPoints [Input] 2D scene point set
	 * @param oGL [Output] A matrix representing the transformation under OpenGL form (4*4 matrix)
	 */
	bool findGLMatrix(const vector<Point2D> &iPoints, double oGL[16]);
};


/**
 * This class defines an interface for matching points in 2D with multiple models or with a single model
 */
class Homography2DwithMultiModel : public Homography2DwithSingleModel
{
protected:
	/**
	 * Model vector.
	 */
	vector< vector<Point2D> > models;
public:

	/**
	 * Constructor with default parameters
	 * @param camMat [Input] The camera matrix, default value is an empty matrix
	 */
	Homography2DwithMultiModel(cv::Mat camMat = cv::Mat());
	virtual ~Homography2DwithMultiModel() {};

	/**
	 * Use to set the model point set. (Only used when there is only a single model)
	 * @param iPoints [Input] 2D point set serving as the model
	 */
	bool setModel(const vector<Point2D> &iPoints);

	/**
	 * Match scene point set with the model point set. (Only used when there is only a single model)
	 * @param iPoints [Input] 2D scene point set
	 * @param oHomo [Output] Result homography
	 * @param oCorrespondenceResultPtr [Output] A pointer to the result correspondence vector, each element contains 3 integers.
	 *       The first integer is a tag of the correspondence used in development.
	 *       The second integer is the ordering number of the model point of this correspondence,
	 *       The last integer is the ordering number of the scene point of this correspondence.
	 */
	bool findHomography2D(const vector<Point2D> &iPoints, cv::Mat_<float> &oHomo, vector< pair<int, pair<int,int> > > *oCorrespondenceResultPtr = NULL);

	/**
	 * Clear all registered model point set.
	 */
	virtual bool clearModels();

	/**
	 * Add one model point set, return an integer indicating the model ID. (Used when there are multiple models)
	 * @param iPoints [Input] 2D point set serving as one model
	 */
	virtual int addModel(const vector<Point2D> &iPoints);

	/**
	 * Match scene point set with the model point sets. (Used when there are multiple models)
	 * @param iPoints [Input] 2D scene point set
	 * @param oModelnumber [Output] An integer indicating the ID of the model being matched.
	 * @param oHomo [Output] Result homography
	 * @param oCorrespondenceResultPtr [Output] A pointer to the result correspondence vector, each element contains 3 integers.
	 *       The first integer is a tag of the correspondence used in development.
	 *       The second integer is the ordering number of the model point of this correspondence,
	 *       The last integer is the ordering number of the scene point of this correspondence.
	 */
	virtual bool findModelandHomography2D(const vector<Point2D> &iPoints, int &oModelnumber, cv::Mat_<float> &oHomo, vector< pair<int, pair<int,int> > > *oCorrespondenceResultPtr = NULL) { return false; };

	/**
	 * Match scene point set with the model point sets. (Used when there are multiple models)
	 * @param iPoints [Input] 2D scene point set
	 * @param oModelnumbers [Output] A vector of integers indicating the ID of the models being matched.
	 * @param oHomos [Output] Result homography vector
	 * @param oCorrespondenceResultPtr [Output] A pointer to the result correspondence vector, each element contains 3 integers.
	 *       The first integer is a tag of the correspondence used in development.
	 *       The second integer is the ordering number of the model point of this correspondence,
	 *       The last integer is the ordering number of the scene point of this correspondence.
	 */
	virtual bool findModelsandHomography2Ds(const vector<Point2D> &iPoints, vector<int> &oModelnumbers, vector< cv::Mat_<float> > &oHomos, vector< vector< pair<int, pair<int,int> > > > *oCorrespondenceResultPtr = NULL) { return false; };

	/**
	 * Match scene point set with the model point sets. (Used when there are multiple models)
	 * @param iPoints [Input] 2D scene point set
	 * @param oModelnumber [Output] An integer indicating the ID of the model being matched.
	 * @param oGL [Output] A matrix representing the transformation under OpenGL form (4*4 matrix)
	 */
	bool findModelandGLMatrix(const vector<Point2D> &iPoints, int &oModelnumber, double oGL[16]);

	/**
	 * Get model point set with the model ID
	 * @param iModelNumber [Input] Model ID
	 * @param oModel [Output] An array containing queried model point set
	 */
	bool getModel(int iModelNumber, vector<Point2D>& oModel);
};

class LGC : public Homography2DwithMultiModel
{
private:
	lgc2::LGCmethod<pcl::PointXY> *lgcalgo;
	lgc2::Parameter *lgcparam;
	lgc2::Affine2D* affinetrans;
	lgc2::Perspective2D* persptrans;
	lgc2::Generator2Daff* generator2Daff;
public:
	LGC(cv::Mat camMat = cv::Mat());

	/**
	 * Constructor with parameters
	 * @param neighborsForModelPt [Input] Number of nearest neighbor points to be used for model point set (Default 6)
	 * @param neighborsForScenePt [Input] Number of nearest neighbor points to be used for scene point set (Default 6)
	 * @param eta [Input] Jitter factor (Default 0.05)
	 * @param N_max [Input] Maximum hypotheses number (Default 45). Increase the value with more noisy points. (i.e. extra points)
	 * @param N_large [Input] Default 20
	 * @param camMat [Input] The camera matrix, default value is an empty matrix
	 */
	LGC(int neighborsForModelPt, int neighborsForScenePt, float eta, int N_max, int N_large, cv::Mat camMat = cv::Mat());
	~LGC();
	bool clearModels();
	int addModel(const vector<Point2D> &iPoints);
	bool findModelandHomography2D(const vector<Point2D> &iPoints, int &oModelnumber, cv::Mat_<float> &oHomo, vector< pair<int, pair<int,int> > > *oCorrespondenceResultPtr = NULL);
	bool findModelsandHomography2Ds(const vector<Point2D> &iPoints, vector<int> &oModelnumbers, vector< cv::Mat_<float> > &oHomos, vector< vector< pair<int, pair<int,int> > > > *oCorrespondenceResultPtr = NULL);
};


#endif /* MATCHINGLIB_MATCHINGINTERFACE_HPP_ */
