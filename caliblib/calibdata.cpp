/*
 * calibdata.cpp
 *
 *  Created on: Dec 11, 2015
 *      Author: liming
 */

#include "calibdata.h"
#include "detectionimprovor.h"

/**
 * ======================================CalibrationData==================================
 */

CalibrationData::CalibrationData() : minOrientationDifference(20.0f), rawDataPtr(0)
{
}


CalibrationData::CalibrationData(const RawData* rd)
: minOrientationDifference(20.0f),
  rawDataPtr(rd)
{

}

CalibrationData::~CalibrationData()
{

}

void CalibrationData::setRawData(const RawData* rd)
{
	rawDataPtr = rd;
}

void CalibrationData::calibrate()
{
	DetectionImprovor detectionImpr;
	rawDataPtr = detectionImpr.improveDetection(rawDataPtr);
	calibrateCamera();
	undistortProjectedPoints();
	calibrateProjector();
	stereoCalibrate();
}


void CalibrationData::printCalibrationInfo()
{
	cout<<"Camera intrinsic matrix = "<<endl;
	cout<<cameraResult.intrinsicMatrix<<endl;
	cout<<"Projector intrinsic matrix = "<<endl;
	cout<<projectorResult.intrinsicMatrix<<endl;
}

bool CalibrationData::calibrateCamera()
{
	if ((rawDataPtr->cameraImageSize.width <= 0)
			||(rawDataPtr->cameraImageSize.height <= 0)
			||(rawDataPtr->physicalPattern2D.size() <= 0))
		return false;

	//Show the detected points and the reference point pattern, for debug use
//	for (unsigned int i = 0 ; i < rawDataPtr->physicalPattern2D.size() ; i++)
//	{
//		cout<<"The board size is (w,h): "<<rawDataPtr->physicalBoardSize.width<<" "<<rawDataPtr->physicalBoardSize.height<<endl;
//		cv::Mat img = previewPointPaires(rawDataPtr->recordedImage[i], rawDataPtr->physicalBoardSize, physicalPatternPtr, &rawDataPtr->physicalPattern2D[i], &rawDataPtr->physicalPointsScene[i]);
//
//		vector<int> compression_params;
//		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
//		compression_params.push_back(0);
//		imwrite("pairs.png", img, compression_params);
//		cv::transpose(img,img);
//		pyrDown(img,img);
//		cv::imshow("Mathcing pairs", img);
//		cv::waitKey(-1);
//	}

	//Construct the 3D projector scene point array
	physicalPattern3D.resize(rawDataPtr->physicalPattern2D.size());
	for (unsigned int i = 0 ; i < rawDataPtr->physicalPattern2D.size() ; i++)
	{

		physicalPattern3D[i].resize(rawDataPtr->physicalPattern2D[i].size());
		for (unsigned int j = 0 ; j < rawDataPtr->physicalPattern2D[i].size() ; j++)
		{
			physicalPattern3D[i][j].x = rawDataPtr->physicalPattern2D[i][j].x;
			physicalPattern3D[i][j].y = rawDataPtr->physicalPattern2D[i][j].y;
			physicalPattern3D[i][j].z = 0;
		}
	}

	/**< No type specifying for cameraResult.rotations and cameraResult.translations, or calibrateCamera will not work > */
	cameraResult.calibrateDevice(physicalPattern3D,
			rawDataPtr->physicalPointsScene,
			rawDataPtr->cameraImageSize);

	cameraResult.computeReprojectionErrors(physicalPattern3D,
	        rawDataPtr->physicalPointsScene,
			cameraResult.perViewErrors,
			cameraResult.totalErr,
			cameraResult.maxErr);

	return true;
}

void CalibrationData::undistortProjectedPoints()
{
	vector<cv::Point2d> undistortedProjectedPointsScene2D;
	vector<cv::Point2d> undistortedCameraPointsScene;
	vector<cv::Point2d> physicalPatternModel2D;

	undistortedProjectedPointsScene.resize(rawDataPtr->projectedPointsScene.size());
	cv::Mat Hcb;
	for (unsigned int i = 0 ; i < rawDataPtr->projectedPointsScene.size() ; i++)
	{
		//Undistort the lens distortion of camera scene points
		undistortPoints(rawDataPtr->physicalPointsScene[i],
				undistortedCameraPointsScene,
				cameraResult.intrinsicMatrix,
				cameraResult.distorsionMatrix);

		//Estimate the homography Hcb (camera->board)
		Hcb = findHomography(undistortedCameraPointsScene, rawDataPtr->physicalPattern2D[i]).clone();

		//Undistort the lens distortion of projector scene points
		undistortPoints(rawDataPtr->projectedPointsScene[i],
				undistortedProjectedPointsScene2D,
				cameraResult.intrinsicMatrix,
				cameraResult.distorsionMatrix);

		//Undistort the perspective transformation;
		perspectiveTransform(undistortedProjectedPointsScene2D, undistortedProjectedPointsScene2D, Hcb);

		//Construct the 3D projector scene point array
		undistortedProjectedPointsScene[i].resize(undistortedProjectedPointsScene2D.size());
		for (unsigned int j = 0 ; j < undistortedProjectedPointsScene2D.size() ; j++)
		{
			undistortedProjectedPointsScene[i][j].x = undistortedProjectedPointsScene2D[j].x;
			undistortedProjectedPointsScene[i][j].y = undistortedProjectedPointsScene2D[j].y;
			undistortedProjectedPointsScene[i][j].z = 0;
		}

		//Show the detected points and the reference point pattern, for debug use
//		{
//			cv::Mat img = previewPointPaires(rawDataPtr->recordedImage[i], rawDataPtr->projectorImageSize, projectedPatternPtr, &rawDataPtr->projectedPatternModel[i], &undistortedProjectedPointsScene2D);
//			//Image parameters
//			vector<int> compression_params;
//			compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
//			compression_params.push_back(0);
//			imwrite("pairs.png", img, compression_params);
//			pyrDown(img,img);
//			cv::imshow("Mathcing pairs", img);
//			cv::waitKey(-1);
//		}
	}

}

bool CalibrationData::calibrateProjector()
{
	if ((rawDataPtr->projectorImageSize.width <= 0)
			||(rawDataPtr->projectorImageSize.height <= 0)
			||(rawDataPtr->projectedPatternModel.size() <= 0))
		return false;

	projectorResult.calibrateDevice(undistortedProjectedPointsScene,    //3D object point for projector
			rawDataPtr->projectedPatternModel,                  //2D image point for projector
			rawDataPtr->projectorImageSize);

	projectorResult.computeReprojectionErrors(undistortedProjectedPointsScene,
	        rawDataPtr->projectedPatternModel,
			projectorResult.perViewErrors,
			projectorResult.totalErr,
			projectorResult.maxErr);
	return true;
}

bool CalibrationData::stereoCalibrate()
{
	cameraResult.setAsStereoOrigin();
	projectorResult.calibrateStereo(cameraResult,
			undistortedProjectedPointsScene,
			rawDataPtr->projectedPointsScene,
			rawDataPtr->projectedPatternModel);

	return true;
}

cv::Mat CalibrationData::previewPointPaires(const cv::Mat &captureImg, const cv::Size imgSize, const vector<cv::Point2d>* allModelPts, const vector<cv::Point2d>* modelPts, const vector<cv::Point2d>* imagePts) const
{
	cv::Mat img = cv::Mat::zeros(imgSize.height, imgSize.width, CV_8UC3);
	img.setTo(255);

	//Find homography
	cv::Mat homo;
	vector<cv::Point2f> modelPts_float(modelPts->size());
	vector<cv::Point2f> imagePts_float(imagePts->size());
	vector<cv::Point2f>::iterator itPt2;

	itPt2 = modelPts_float.begin();
	//Draw all matched model points
	for (vector<cv::Point2d>::const_iterator itPt = modelPts->begin(); itPt != modelPts->end() ; ++itPt)
	{
//		cout<<itPt->x<<" "<<itPt->y<<" "<<endl;
		itPt2->x = (float)itPt->x;
		itPt2->y = (float)itPt->y;
		++itPt2;
	}

	itPt2 = imagePts_float.begin();
	for (vector<cv::Point2d>::const_iterator itPt = imagePts->begin(); itPt != imagePts->end() ; ++itPt)
	{
//		cout<<itPt->x<<" "<<itPt->y<<" "<<endl;
		itPt2->x = (float)itPt->x;
		itPt2->y = (float)itPt->y;
		++itPt2;
	}
	homo = cv::findHomography(imagePts_float, modelPts_float);

	//create background image
	warpPerspective(captureImg, img, homo, imgSize, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 255);

	//reproject
	vector<cv::Point2f> reprojectedPoints;
	cv::perspectiveTransform(imagePts_float, reprojectedPoints, homo);

	//Draw all model points
	for (vector<cv::Point2d>::const_iterator itPt = allModelPts->begin() ; itPt != allModelPts->end() ; ++itPt)
	{
//		cout<<itPt->x<<" "<<itPt->y<<" "<<endl;
		cv::circle(img, *itPt, 10, cv::Scalar(0, 0, 0), 1);
	}

	//Draw all matched model points
	for (vector<cv::Point2d>::const_iterator itPt = modelPts->begin(); itPt != modelPts->end() ; ++itPt)
	{
//		cout<<itPt->x<<" "<<itPt->y<<" "<<endl;
		cv::circle(img, *itPt, 5, cv::Scalar(0, 0, 255), 2);
	}

	//Draw all reprojected points
	for (itPt2 = reprojectedPoints.begin() ; itPt2 != reprojectedPoints.end() ; ++itPt2)
	{
//		cout<<itPt2->x<<" "<<itPt2->y<<" "<<endl;
		cv::circle(img, *itPt2, 3, cv::Scalar(255, 0, 0), 2);
	}

	return img;
}

void CalibrationData::saveExtrinsicParam(cv::FileStorage& fs, const vector<cv::Mat>& rvecs, const vector<cv::Mat>& tvecs)
{
	CV_Assert(fs.isOpened());
    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        cv::Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            cv::Mat r = bigmat(cv::Range(i, i+1), cv::Range(0,3));
            cv::Mat t = bigmat(cv::Range(i, i+1), cv::Range(3,6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        //cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "extrinsic_parameters" << bigmat;
    }
}

void CalibrationData::saveParams( const string& filename, bool withRawdata)
{
    cv::FileStorage fs( filename, cv::FileStorage::WRITE );

    time_t t;
    time( &t );
    struct tm *t2 = localtime( &t );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_time" << buf;

    fs << "nframes" << (int)(rawDataPtr->getSize());
    fs << "board_width" << rawDataPtr->physicalBoardSize.width;
    fs << "board_height" << rawDataPtr->physicalBoardSize.height;
    fs << "image_width" << rawDataPtr->cameraImageSize.width;
    fs << "image_height" << rawDataPtr->cameraImageSize.height;
    fs << "projector_width" << rawDataPtr->projectorImageSize.width;
    fs << "projector_height" << rawDataPtr->projectorImageSize.height;

    fs << "Camera  0" << cameraResult;
    fs << "Projector  0" << projectorResult;

    /**
     * Create rawdata file
     */
    if (withRawdata)
    {
    	int lastindex = filename.find_last_of(".");
        string filenameprefix = filename.substr(0, lastindex);
        string rawdatafilename = filenameprefix + "_rawdata.yaml";

    	lastindex = rawdatafilename.find_last_of("/");
        string nodirName = rawdatafilename.substr(lastindex+1);

        cvWriteComment( *fs, "Raw point data are stored in raw_data_file", 0 );
        fs << "raw_data_file"<< nodirName;
        fs.release();
        rawDataPtr->saveData(rawdatafilename);

        rawdatafilename = filenameprefix + "_rawdata.txt";
        rawDataPtr->saveTextData(rawdatafilename);
    }

}

/**
 * ======================================DeviceResult==================================
 */

RawData::RawData()
{

	projectedPatternModel.clear();
	physicalPattern2D.clear();
	physicalPointsScene.clear();
	projectedPointsScene.clear();

	homobcVector.clear();
	homopcVector.clear();

	recordedImage.clear();
}

void RawData::setModelPatterns(const vector<cv::Point2d> &physicalPat, const vector<cv::Point2d> &unwrapedProjectedPat)
{
	physicalPatternPtr = &physicalPat;
	unwrapedprojectedPatternPtr =  &unwrapedProjectedPat;
}

void RawData::setAllSizes(const cv::Size &boardSize, const cv::Size &camSize, const cv::Size &proSize)
{
	physicalBoardSize = boardSize;
	cameraImageSize = camSize;
	projectorImageSize = proSize;
}

unsigned int RawData::getSize() const
{
	return physicalPattern2D.size();
}

cv::Size RawData::getBoardSize()
{
	return physicalBoardSize;
}

void RawData::addOneObservation(const cv::Mat &img,
		const cv::Mat &homobc,
		const cv::Mat &homopc,
		const vector<cv::Point2d> &detectedPhysicalPoints,
		const vector< pair<int, pair<int,int> > > &phsycalCorresp,
		const vector<cv::Point2d> *allProjectedPointPtr,
		const vector<cv::Point2d> &detectedProjectedPoints,
		const vector< pair<int, pair<int,int> > > &projectedCorresp)
{
	recordedImage.push_back(img.clone());
	homobcVector.push_back(homobc.clone());
	homopcVector.push_back(homopc.clone());
	unsigned int num;

	//Cleaning intersecting points



	//For camera
	num = phsycalCorresp.size();
	vector<cv::Point2d> physicalModel(num);
	vector<cv::Point2d> physicalScene(num);
	for (unsigned int i = 0 ; i < num ; i++)
	{
		//model
		physicalModel[i] = (*physicalPatternPtr)[phsycalCorresp[i].second.first];

		//scene
		physicalScene[i] = detectedPhysicalPoints[phsycalCorresp[i].second.second];
	}
	physicalPattern2D.push_back(physicalModel);
	physicalPointsScene.push_back(physicalScene);

	//For projector
	num = projectedCorresp.size();
	vector<cv::Point2d> projectedModel(num);
	vector<cv::Point2d> projectedScene(num);

	for (unsigned int i = 0 ; i < num ; i++)
	{
		//unwraped model
//		projectedModel[i] = (*projectedPatternPtr)[projectedCorresp[i].second.first];

		//wraped model (really projected points)
		projectedModel[i] = (*allProjectedPointPtr)[projectedCorresp[i].second.first];

		//scene
		projectedScene[i] = detectedProjectedPoints[projectedCorresp[i].second.second];
	}
	allProjectedPts.push_back(*allProjectedPointPtr);
	projectedPatternModel.push_back(projectedModel);
	projectedPointsScene.push_back(projectedScene);

//	if (homobcVector.size() > CRITIC_NUMBER_OF_HOMO_FOR_ROUGH_CALIB)
//	{
//		roughCalibCam(homobcVector, img.cols, img.rows);
//	}
}

bool RawData::saveData(string filename) const
{
	cv::FileStorage fs( filename, cv::FileStorage::WRITE );

	fs << "calibration_board_size" << physicalBoardSize;
    fs << "camera_image_size" << cameraImageSize;
    fs << "projector_image_size" << projectorImageSize;

    //May be need record projected and physical patterns
//    fs << "physicalPatterns" << cv::Mat(*physicalPatternPtr);
//    fs << "projectedPatterns" << cv::Mat(*projectedPatternPtr);

	unsigned int numberofData = physicalPattern2D.size();
	stringstream ss;

    fs << "View_num" << (int)numberofData;

	int lastindex = filename.find_last_of("/");
    string dirPath = filename.substr(0, lastindex);

	//Image parameters
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(0);

	for (unsigned int i = 0 ; i < numberofData ; i++)
	{
		ss.str(std::string());
		ss << "view_" << i;

	    cvStartWriteStruct(fs.operator *(), ss.str().c_str(), CV_NODE_MAP, NULL);
	    fs << "Homo_board_cam" << homobcVector[i];
	    fs << "physicalPatternModel" << cv::Mat(physicalPattern2D[i]);
	    fs << "physicalPointsScene" << cv::Mat(physicalPointsScene[i]);

	    fs << "Homo_projector_cam" << homopcVector[i];
	    fs << "allProjectedPoints" << cv::Mat( allProjectedPts[i] );
	    fs << "projectedPatternModel" << cv::Mat(projectedPatternModel[i]);
	    fs << "projectedPointsScene" << cv::Mat(projectedPointsScene[i]);

	    ss << ".png";
	    fs << "image" << ss.str();
	    cv::imwrite( string(dirPath + "/" + ss.str()), recordedImage[i], compression_params);

	    cvEndWriteStruct(fs.operator *());
	}

	fs.release();
	return true;
}

bool RawData::loadData(string filename)
{
    int lastindex = filename.find_last_of("/");
    string path = filename.substr(0, lastindex);

	cv::FileStorage fs( filename, cv::FileStorage::READ );
	unsigned int  numberofData;
	fs["calibration_board_size"] >> physicalBoardSize;
    fs["camera_image_size"] >> cameraImageSize;
    fs["projector_image_size"] >> projectorImageSize;

//    view_node["physicalPatterns"] >> tempMat;
//    tempMat.copyTo(physicalPattern2D[i]);

//    fs << "physicalPatterns" << cv::Mat(*physicalPatternPtr);
//    fs << "projectedPatterns" << cv::Mat(*projectedPatternPtr);

    numberofData = (int)fs["View_num"];

	recordedImage.resize(numberofData);
	homobcVector.resize(numberofData);
	homopcVector.resize(numberofData);

	physicalPattern2D.resize(numberofData);
	physicalPointsScene.resize(numberofData);

	allProjectedPts.resize(numberofData);
	projectedPatternModel.resize(numberofData);
	projectedPointsScene.resize(numberofData);

	stringstream ss;

	//Image parameters
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(0);

	cv::Mat tempMat;

	for (unsigned int i = 0 ; i < numberofData ; i++)
	{
		ss.str(std::string());
		ss << "view_" << i;

		cv::FileNode view_node = fs[ss.str()];

		view_node["Homo_board_cam"] >> homobcVector[i];
	    view_node["physicalPatternModel"] >> tempMat;
	    tempMat.copyTo(physicalPattern2D[i]);
	    view_node["physicalPointsScene"] >> tempMat;
	    tempMat.copyTo(physicalPointsScene[i]);

	    view_node["Homo_projector_cam"] >> homopcVector[i];
	    view_node["allProjectedPoints"] >> tempMat;
	    tempMat.copyTo(allProjectedPts[i]);
	    view_node["projectedPatternModel"] >> tempMat;
	    tempMat.copyTo(projectedPatternModel[i]);
	    view_node["projectedPointsScene"] >> tempMat;
	    tempMat.copyTo(projectedPointsScene[i]);

	    string imgFileName;
	    view_node["image"] >> imgFileName;
	    recordedImage[i] = cv::imread(string(path + "/" + imgFileName));
	}

	return true;
}


bool RawData::saveTextData(string fileName) const
{
	fstream file;
	file.open(fileName.c_str(),std::fstream::out);

	if (!file.is_open())         //Check if the corresponding file exists
	{
		std::cout<<"Fail to open file "<< fileName << std::endl;
		return false;
	}

	unsigned int numberofData = physicalPattern2D.size();

	file<< numberofData<<endl;
	unsigned int pointnum, checksum;
	for (unsigned int i = 0 ; i < numberofData ; i++)
	{
		/**
		 * Read the coordinates for physical pattern
		 * pointnum, checksum
		 * modelx, modely, modelz, imagex, imagey
		 * ...
		 *
		 * checksum = i*10+0 (0 for physical pattern, 1 for projected pattern
		 */
		pointnum = physicalPattern2D[i].size(); checksum = i*10 + 0;
		file << pointnum <<" "<< checksum <<endl;

		for(unsigned int j = 0 ; j < pointnum ; j++)
		{
			file << physicalPattern2D[i][j].x<<" "
					<< physicalPattern2D[i][j].y<<" "
					<< physicalPointsScene[i][j].x<<" "
					<< physicalPointsScene[i][j].y<<endl;
		}

		pointnum = projectedPatternModel[i].size(); checksum = i*10 + 1;
		file << pointnum <<" "<< checksum <<endl;

		for(unsigned int j = 0 ; j < pointnum ; j++)
		{
			file << projectedPatternModel[i][j].x<<" "
					<< projectedPatternModel[i][j].y<<" "
					<< projectedPointsScene[i][j].x<<" "
					<< projectedPointsScene[i][j].y<<endl;
		}
	}

	file.close();

	cout<<"Succesfully saved data to "<<fileName<<endl;
	return true;
}

bool RawData::loadTextData(string fileName)
{
	fstream file;
	file.open(fileName.c_str(),std::fstream::in);

	if (!file.is_open())         //Check if the corresponding file exists
	{
		std::cout<<"Fail to open file "<< fileName << std::endl;
		return false;
	}

//	file >> cameraImageSize.width >> cameraImageSize.height;
//	file >> projectorImageSize.width >> projectorImageSize.height;

	unsigned int numberofData;
	file >> numberofData;
	physicalPattern2D.resize(numberofData);
	physicalPointsScene.resize(numberofData);
	projectedPatternModel.resize(numberofData);
	projectedPointsScene.resize(numberofData);

	unsigned int pointnum, checksum;
	for (unsigned int i = 0 ; i < numberofData ; i++)
	{
		/**
		 * Read the coordinates for physical pattern
		 * pointnum, checksum
		 * modelx, modely, modelz, imagex, imagey
		 * ...
		 *
		 * checksum = i*10+0 (0 for physical pattern, 1 for projected pattern
		 */
		file >> pointnum >> checksum;
		if (checksum != i*10 + 0)
		{
			cout<<"The check sum is not correct: "<<checksum<<"!="<<i*10+0<<endl;
			return false;
		}

		physicalPattern2D[i].resize(pointnum);
		physicalPointsScene[i].resize(pointnum);

		for(unsigned int j = 0 ; j < pointnum ; j++)
		{
			file >> physicalPattern2D[i][j].x
					>> physicalPattern2D[i][j].y
					>> physicalPointsScene[i][j].x
					>> physicalPointsScene[i][j].y;
		}

		file >> pointnum >> checksum;
		if (checksum != i*10 + 1)
		{
			cout<<"The check sum is not correct: "<<checksum<<"!="<<i*10+1<<endl;
			return false;
		}
		projectedPatternModel[i].resize(pointnum);
		projectedPointsScene[i].resize(pointnum);

		for(unsigned int j = 0 ; j < pointnum ; j++)
		{
			file >> projectedPatternModel[i][j].x
					>> projectedPatternModel[i][j].y
					>> projectedPointsScene[i][j].x
					>> projectedPointsScene[i][j].y;
		}
	}

	file.close();

	return true;
}

bool RawData::operator==(const RawData &other)
{
	if (physicalPattern2D != other.physicalPattern2D) return false;
	if (physicalPointsScene != other.physicalPointsScene) return false;
	if (projectedPatternModel != other.projectedPatternModel) return false;
	if (projectedPointsScene != other.projectedPointsScene) return false;
	return true;
}

//void aContrast(cv::Mat_<uchar> &inputouput)
//{
//	cv::Mat_<float> whitefloat, adjustedfloat;
//	cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 11, 11 ));
//
//	cv::Mat_<float> grayfloat = inputouput;
//	cv::morphologyEx(grayfloat, whitefloat, cv::MORPH_CLOSE, element);
//
//	cv::divide(grayfloat, whitefloat, adjustedfloat);
//
//	cv::normalize(adjustedfloat,adjustedfloat,0,255,cv::NORM_MINMAX);
//	inputouput = adjustedfloat;
//}

//void findMeanAndStdev(const vector<double> &v, double& mean, double& stdev)
//{
//	double sum = std::accumulate(v.begin(), v.end(), 0.0);
//	mean = sum / v.size();
//
//	double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
//	stdev = std::sqrt(sq_sum / v.size() - mean * mean);
//}

//// Find average blob size
//double RawData::getAvgBlobRadius(const cv::Mat_<uchar> &binaryImg, const vector<cv::Point2d> &refPts)
//{
//	unsigned int length = refPts.size();
//	cv::Mat_<double> blobRadius = cv::Mat::zeros(length, 1, CV_64F);
//
//	cv::Mat_<uchar> bImg = binaryImg.clone();
//	cv::Rect comp;
//	for (unsigned int i = 0 ; i < refPts.size() ; i++)
//	{
//		if (bImg[int(refPts[i].y)][int(refPts[i].x)] == 0)
//		{
//			blobRadius[i][0] = sqrt((double)cv::floodFill(bImg, refPts[i], cvScalarAll(255), &comp, 8)/CV_PI);          //clean the points from intersectionMap
//		}
//		else
//		{
//			blobRadius[i][0] = 0;
//		}
//	}
//
////	cout<<blobRadius<<endl;
//	cv::Scalar mean, stdev;
//	meanStdDev(blobRadius, mean, stdev);
//
//	double sum = 0;
//	int ptNum = 0;
//	for (unsigned int i = 0 ; i < refPts.size() ; i++)
//	{
//		if ( abs(blobRadius[i][0] - mean.val[0]) < 2*stdev.val[0])
//		{
//			sum += blobRadius[i][0];
//			ptNum++;
//		}
//	}
//
//	return (sum/ptNum);
//}

//cv::Mat_<uchar> RawData::getPointTemplate(int iRadius)
//{
//	cv::Mat_<uchar> templ = cv::Mat::zeros(iRadius*2+1, iRadius*2+1, CV_8U);
//	for (int rx = 0 ; rx <= iRadius*2+1 ; rx++)
//		for (int ry = 0 ; ry <= iRadius*2+1 ; ry++)
//			if (  (rx - iRadius)*(rx - iRadius)+(ry - iRadius)*(ry - iRadius)<=iRadius*iRadius )
//				{
//					templ.at<unsigned char>(ry,rx) = 255;             //Top left as origne, coherant with 09paper
//				}
//
//	return (255 - templ);       //Black point on white background
//}

//NB: the radius parameter is very important
//void RawData::improveSingleViewDetection(const cv::Mat_<uchar> &grayImg,
//		const cv::Size &dsize,
//		const cv::Mat_<double> &homo1,
//		vector<cv::Point2d> &refPts,
//		vector<cv::Point2d> &detectedPts,
//		bool useTemplateMatching)
//{
//
//	cv::Mat_<double> homo = cv::findHomography(refPts, detectedPts);
//	unsigned int pointNum = refPts.size();
//
//	cv::Mat_<uchar> warpedImg;
//	warpPerspective(grayImg, warpedImg, homo.inv(), dsize, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 255);  //Empty area is white
//	threshold(warpedImg, warpedImg, 200, 255, cv::THRESH_BINARY);
//
//	double radius = getAvgBlobRadius(warpedImg, refPts);
//	cout<<"avgRaidus = "<<radius<<endl;
//
//	cv::imwrite("tmpfile.png", warpedImg);
//	cv::imshow("grayImg", grayImg);
//	cv::waitKey(-1);
//
//	//Create a template
//	int iRadius = round(radius);
//	cv::Mat_<uchar> templ = getPointTemplate(iRadius);
//
//	cv::Mat_<uchar> maskwithborder,mask = cv::Mat::zeros(warpedImg.rows, warpedImg.cols, CV_8UC1);
//	int halfwindowSize = iRadius*1.3;
//
//	int fillValue = 255;
//	double templArea = CV_PI*iRadius*iRadius;
//	double filledArea = 0;
//
//	cv::Rect comp;
//	cv::Mat roi;
//	cv::Mat_<float> resRoi;
//	cv::Mat_<uchar> location;
//
//	vector<double> minVals;
//	vector<cv::Point2d> tmpRefPts;
//	vector<cv::Point2d> tmpDetectedPts;
//
//	minVals.reserve(pointNum);
//	tmpRefPts.reserve(pointNum);
//	tmpDetectedPts.reserve(pointNum);
//
//	for (unsigned int i = 0 ; i < pointNum ; i++)
//	{
//		if ( (warpedImg[int(refPts[i].y)][int(refPts[i].x)] == 0)
//				&&(refPts[i].x - halfwindowSize >= 0)&&(refPts[i].y - halfwindowSize >= 0)
//				&&(refPts[i].x + halfwindowSize < warpedImg.cols)&&(refPts[i].y + halfwindowSize < warpedImg.rows))
//		{
////			unsigned int minx = refPts[i].x - halfwindowSize >= 0 ? refPts[i].x - halfwindowSize : 0;
////			unsigned int miny = refPts[i].y - halfwindowSize >= 0 ? refPts[i].y - halfwindowSize : 0;
////			unsigned int width  = refPts[i].x + halfwindowSize < warpedImg.cols ? 2*halfwindowSize+1 : warpedImg.cols - minx;
////			unsigned int height = refPts[i].y + halfwindowSize < warpedImg.rows ? 2*halfwindowSize+1 : warpedImg.rows - miny;
//
////			roi = warpedImg( cv::Rect(minx, miny, width, height) );
//			mask = cv::Mat::zeros(warpedImg.rows, warpedImg.cols, CV_8UC1);
//			cv::copyMakeBorder(mask, maskwithborder, 1, 1, 1, 1, cv::BORDER_REPLICATE);
//			filledArea = (double)cv::floodFill(warpedImg, maskwithborder, refPts[i] , cv::Scalar(255) , &comp, cv::Scalar(), cv::Scalar(), 8 | cv::FLOODFILL_MASK_ONLY | (fillValue << 8));
//
//			if ((filledArea > 0.5*templArea)&&(filledArea < 1.5*templArea))
//			{
//				roi = maskwithborder(cv::Rect(comp.x+1, comp.y+1, comp.width, comp.height));
//
//				cv::Moments m;
//				if (useTemplateMatching)
//				{
//					int ecart = comp.width < comp.height ? templ.rows - comp.width : templ.rows - comp.height;
//					if (ecart > 0)
//						copyMakeBorder( roi, roi, ecart, ecart, ecart, ecart, cv::BORDER_CONSTANT, 0 );
//					roi = 255 - roi;
//					//mask = 255 - maskwithborder(cv::Rect(1,1,mask.cols,mask.rows));
//					matchTemplate( roi, templ, resRoi, CV_TM_SQDIFF );
//
//					double minVal; double maxVal;
//					minMaxLoc( resRoi, &minVal, &maxVal, NULL, NULL, cv::Mat() );
//					copyMakeBorder( resRoi, resRoi, iRadius, iRadius, iRadius, iRadius, cv::BORDER_CONSTANT, 2*minVal+1 );
//					location = (resRoi == minVal);
//
//					/**
//					 * Create a result matrix, each row represents a (reference point, scene point, value):
//					 * refPt.x refPt.y detectedPt.x detectedPt.y minVal
//					 * ....
//					 * ....
//					 */
//
//					if (ecart > 0)
//						location = location(cv::Rect(ecart, ecart, location.cols - 2*ecart, location.rows - 2*ecart));
//
//					m = moments(location);
//					minVals.push_back(minVal);
//				}
//				else
//				{
//					m = moments(roi);
//					minVals.push_back(filledArea);
//				}
//
//				tmpRefPts.push_back(refPts[i]);
//				tmpDetectedPts.push_back(cv::Point2d(m.m10/m.m00 + comp.x, m.m01/m.m00 + comp.y));
//
////				if (cv::norm(tmpRefPts.back() - tmpDetectedPts.back()) > 4)
////				{
////					cv::Mat showRoi = warpedImg( comp );;
////					if (!location.empty())
////					{
////						location.copyTo(showRoi, location);
////					}
////					else
////					{
////						showRoi.at<uchar>((int)tmpDetectedPts.back().y,(int)tmpDetectedPts.back().x) = 255;
////					}
////
//////					cout<<location<<endl;
////					cout<<tmpRefPts.back() - tmpDetectedPts.back()<<" at (reference = )"<<tmpRefPts.back()<<endl;
////					cv::imshow("grayImg", grayImg);
////					cv::imshow("showRoi", showRoi);
////					cv::waitKey(-1);
////				}
//			}
//		}
//	}
//
//	double mean, stdev;
//	findMeanAndStdev(minVals, mean, stdev);
//
//	//Record data whose minVal is smaller than mean+2*stdev
//	double threshold = mean + 2*stdev;
//	refPts.clear();
//	refPts.reserve(pointNum);
//	detectedPts.clear();
//	detectedPts.reserve(pointNum);
//	for (unsigned int i = 0 ; i < minVals.size() ; i++)
//	{
//		if (minVals[i] < threshold)
//		{
//			refPts.push_back(tmpRefPts[i]);
//			detectedPts.push_back(tmpDetectedPts[i]);
//		}
////		cout<<tmpRefPts[i].x<<" "<<tmpRefPts[i].y<<" "<<tmpDetectedPts[i].x<<" "<<tmpDetectedPts[i].y<<" "<<minVals[i]/255/255<<";"<<endl;
//	}
//
//	cout<<"Left "<<refPts.size()<<" points."<<endl;
//	cv::perspectiveTransform(detectedPts, detectedPts, homo);
//}
//
//void CalibrationData::improveDetection2()
//{
//	DetectionImprovor* detectionimpr = new DetectionImprovor();
//	this->rawDataPtr = detectionimpr->improveDetection(this->rawDataPtr);
//}
//
//void RawData::improveDetection()
//{
//	unsigned int viewNum = physicalPattern2D.size();
//	cv::Mat_<uchar> grayImg;
//
//	for (unsigned int i = 0 ; i < viewNum ; i++)
//	{
//		cout<<"View num = "<<i<<endl;
//		cvtColor( recordedImage[i], grayImg, CV_BGR2GRAY );
////		cv::imshow("GrayImg", grayImg);
//		aContrast(grayImg);
////		cv::imshow("PhysicalPtImg", grayImg);
////		cv::waitKey(-1);
//
//		//Improve physical point detection
//		improveSingleViewDetection(grayImg,
//				physicalBoardSize,
//				homobcVector[i],
//				physicalPattern2D[i],
//				physicalPointsScene[i],
//				true);
//
//		cvtColor( recordedImage[i], grayImg, CV_BGR2GRAY );
//		grayImg = 255 - grayImg;
//		cv::imshow("grayImg1", grayImg);
//		aContrast(grayImg);
//		cv::imshow("grayImg2", grayImg);
//		cv::waitKey(-1);
////		cv::imshow("ProjectedPtImg", grayImg);
////		cv::waitKey(-1);
//
//
//		//Improve projected point detection
//		improveSingleViewDetection(grayImg,
//				projectorImageSize,
//				homopcVector[i],
//				projectedPatternModel[i],
//				projectedPointsScene[i],
//				false);
//	}
//}

void RawData::checkDetection() const
{
	unsigned int viewNum = physicalPattern2D.size();
	cv::Mat img_to_show;
	cv::Mat homo;
	vector<cv::Point2d> wrapeddetectedPts;

	for (unsigned int i = 0 ; i < viewNum ; i++)
	{
		cout<<"View num = "<<i<<endl;

		homo = cv::findHomography(physicalPattern2D[i], physicalPointsScene[i]);
		warpPerspective(recordedImage[i], img_to_show, homo.inv(), physicalBoardSize, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 255);  //Empty area is white
		cv::perspectiveTransform(physicalPointsScene[i], wrapeddetectedPts, homo.inv());

		for (int j = 0 ; j < wrapeddetectedPts.size() ; j++)
		{
			cv::circle(img_to_show, physicalPattern2D[i][j], 10, cv::Scalar(0,0,255), 2);
			cv::circle(img_to_show, wrapeddetectedPts[j], 5, cv::Scalar(255,0,0), 2);
		}
		imwrite("detection.png", img_to_show);
		cv::pyrDown(img_to_show,img_to_show);
		imshow("detection", img_to_show);
		cv::waitKey(-1);

		homo = cv::findHomography(projectedPatternModel[i], projectedPointsScene[i]);
		warpPerspective(recordedImage[i], img_to_show, homo.inv(), projectorImageSize, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 255);  //Empty area is white
		cv::perspectiveTransform(projectedPointsScene[i], wrapeddetectedPts, homo.inv());

		for (int j = 0 ; j < wrapeddetectedPts.size() ; j++)
		{
			cv::circle(img_to_show, projectedPatternModel[i][j], 10, cv::Scalar(0,0,255), 2);
			cv::circle(img_to_show, wrapeddetectedPts[j], 5, cv::Scalar(255,0,0), 2);
		}
		imwrite("detection.png", img_to_show);
		cv::pyrDown(img_to_show,img_to_show);
		imshow("detection", img_to_show);
		cv::waitKey(-1);

	}
}

/**
 * ======================================DeviceResult==================================
 */
DeviceResult::DeviceResult()
: maxErr(0),
  totalErr(0)
{
	calibrationFlags = CV_CALIB_FIX_K3 | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5 | CV_CALIB_FIX_K6;
	calibrationFlags = calibrationFlags | CV_CALIB_FIX_INTRINSIC;
}

void DeviceResult::calibrateDevice(const vector< vector<cv::Point3d> > &worldPoints3D, const vector< vector<cv::Point2d> > &imgPoints2D, cv::Size imgSize)
{
	intrinsicMatrix = cv::Mat::eye(3, 3, CV_64F);
	distorsionMatrix = cv::Mat::zeros(4, 1, CV_64F);

	/**
	 * according to samuel's method, the points coordinates are all in double,
	 * but just use float ( vector< vector<cv::Point3f> >  ) to do the calibration
	 * for calibration result (intrinsicMatrix, distorsionMatrix), they are double as well
	 */

	vector< vector<cv::Point2f> > imgPoints2D_float(imgPoints2D.size());
	vector< vector<cv::Point3f> > worldPoints3D_float(worldPoints3D.size());
	for (unsigned int i = 0 ; i < imgPoints2D.size() ; i++)
	{
		imgPoints2D_float[i].resize(imgPoints2D[i].size());
		for (unsigned int j = 0 ; j < imgPoints2D[i].size() ; j++)
		{
			imgPoints2D_float[i][j].x = (float) imgPoints2D[i][j].x;
			imgPoints2D_float[i][j].y = (float) imgPoints2D[i][j].y;
		}

		worldPoints3D_float[i].resize(worldPoints3D[i].size());
		for (unsigned int j = 0 ; j < worldPoints3D[i].size() ; j++)
		{
			worldPoints3D_float[i][j].x = (float) worldPoints3D[i][j].x;
			worldPoints3D_float[i][j].y = (float) worldPoints3D[i][j].y;
			worldPoints3D_float[i][j].z = (float) worldPoints3D[i][j].z;
		}
	}

	cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, DBL_EPSILON);
	cv::calibrateCamera(worldPoints3D_float,
			imgPoints2D_float,
			imgSize,
			intrinsicMatrix, distorsionMatrix,
			rotations, translations,
			calibrationFlags,
			criteria);

}

void DeviceResult::computeReprojectionErrors(const vector< vector<cv::Point3d> > &worldPoints3D,
			const vector< vector<cv::Point2d> > &imgPoints2D,
	        vector<double>& perViewErrors,
			double &mean_err,
			double &max_err)
{
	vector<cv::Point2d> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(worldPoints3D.size(), 0);

	for( i = 0; i < (int)worldPoints3D.size(); ++i )
	{
		projectPoints( cv::Mat(worldPoints3D[i]), rotations[i], translations[i], intrinsicMatrix,  // project
							 distorsionMatrix, imagePoints2);

        for (int j = 0; j < (int)worldPoints3D[i].size(); j++) {
        	double ptErr = cv::norm(imgPoints2D[i][j] - imagePoints2[j]);
        	double err2 = ptErr*ptErr;

        	perViewErrors[i] += err2;
        	totalErr += err2;
        	totalPoints++;

            if (ptErr > max_err) {
                max_err = ptErr;
            }
        }

        perViewErrors[i] = std::sqrt(perViewErrors[i]/(int)worldPoints3D[i].size());
	}

	mean_err = std::sqrt(totalErr/totalPoints);              // calculate the arithmetical mean
}


void DeviceResult::calibrateStereo(const DeviceResult& baseDevice,
					const vector< vector<cv::Point3d> > &worldPoints3D,
					const vector< vector<cv::Point2d> > &imgPoints2D_baseDevice,
					const vector< vector<cv::Point2d> > &imgPoints2D_thisDevice)
{
	//Point2D - Point2f
	assert(worldPoints3D.size() == imgPoints2D_baseDevice.size());
	assert(worldPoints3D.size() == imgPoints2D_thisDevice.size());

	vector< vector<cv::Point3f> > worldPoints3D_float(worldPoints3D.size());
	vector< vector<cv::Point2f> > imgPoints2D_baseDevice_float(imgPoints2D_baseDevice.size());
	vector< vector<cv::Point2f> > imgPoints2D_thisDevice_float(imgPoints2D_thisDevice.size());

	for (unsigned int i = 0 ; i < worldPoints3D.size() ; i++)
	{
		unsigned int num = worldPoints3D[i].size();
		assert(num == imgPoints2D_baseDevice[i].size());
		assert(num == imgPoints2D_thisDevice[i].size());

		worldPoints3D_float[i].resize(num);
		imgPoints2D_baseDevice_float[i].resize(num);
		imgPoints2D_thisDevice_float[i].resize(num);
		for (unsigned int j = 0 ; j < num ; j++)
		{
			worldPoints3D_float[i][j] = (cv::Point3f)worldPoints3D[i][j];
			imgPoints2D_baseDevice_float[i][j] = (cv::Point2f)imgPoints2D_baseDevice[i][j];
			imgPoints2D_thisDevice_float[i][j] = (cv::Point2f)imgPoints2D_thisDevice[i][j];
		}

	}
    cv::stereoCalibrate(worldPoints3D_float, imgPoints2D_baseDevice_float, imgPoints2D_thisDevice_float,
    		baseDevice.intrinsicMatrix, baseDevice.distorsionMatrix, intrinsicMatrix, distorsionMatrix,
            cv::Size(0, 0), R, T, E, F, calibrationFlags,
            cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,100,1e-6));
}

void DeviceResult::setAsStereoOrigin()
{
	//Current Device is at origne
	R = cv::Mat::eye(3,3, CV_64F);
	T = cv::Mat::zeros(3,1,CV_64F);
	E = cv::Mat::zeros(3,3, CV_64F);
	F = cv::Mat::zeros(3,3, CV_64F);
}

//These write and read functions must be defined for the serialization in FileStorage to work
void write(cv::FileStorage& fs, const std::string&, const DeviceResult& dr)
{
    dr.write(fs);
}

void DeviceResult::write(cv::FileStorage& fs) const
{
	fs << "{";

    if( calibrationFlags != 0 )
    {
      char buf[1024];
      //vona        sprintf( buf, "flags: %s%s%s%s",
      sprintf( buf, "flags: %s%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
    		calibrationFlags & CV_CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
    		calibrationFlags & CV_CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
    		calibrationFlags & CV_CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
    		calibrationFlags & CV_CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "",
    		calibrationFlags & CV_CALIB_FIX_FOCAL_LENGTH ? "+fix_focal_length" : "",
    		calibrationFlags & CV_CALIB_FIX_K1 ? "+fix_k1" : "",
    		calibrationFlags & CV_CALIB_FIX_K2 ? "+fix_k2" : "",
    		calibrationFlags & CV_CALIB_FIX_K3 ? "+fix_k3" : "",
    	    calibrationFlags & CV_CALIB_FIX_K4 ? "+fix_k4" : "",
    	    calibrationFlags & CV_CALIB_FIX_K5 ? "+fix_k5" : "",
    	    calibrationFlags & CV_CALIB_FIX_K6 ? "+fix_k6" : "",
    	    calibrationFlags & CV_CALIB_RATIONAL_MODEL ? "+rational_model" : "",
    		calibrationFlags & (CV_CALIB_FIX_K1|CV_CALIB_FIX_K2|CV_CALIB_FIX_K3) ? "+zero_radial_dist" : "",

    		calibrationFlags & CV_CALIB_FIX_INTRINSIC ? "+(stereo)fix_intrinsic" : "",
    		calibrationFlags & CV_CALIB_SAME_FOCAL_LENGTH ? "+(stereo)same_focal_length" : ""); //vona
      cvWriteComment( *fs, buf, 0 );
    }
    fs << "flags" << calibrationFlags;

	fs << "cameraMatrix" << intrinsicMatrix;
	fs << "distortionCoeffs" << distorsionMatrix;

	//Save external parameters
	cv::Mat bigmat((int)rotations.size(), 6, rotations[0].type());
    if( !rotations.empty() && !translations.empty() )
    {
        CV_Assert(rotations[0].type() == translations[0].type());
        for( int i = 0; i < (int)rotations.size(); i++ )
        {
            cv::Mat r = bigmat(cv::Range(i, i+1), cv::Range(0,3));
            cv::Mat t = bigmat(cv::Range(i, i+1), cv::Range(3,6));

            CV_Assert(rotations[i].rows == 3 && rotations[i].cols == 1);
            CV_Assert(translations[i].rows == 3 && translations[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rotations[i].t();
            t = translations[i].t();
        }
        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
    }

	fs << "extrParams" << bigmat;
	fs << "reprojErrs" << cv::Mat(perViewErrors).t();
	fs << "avgReprojErr" << totalErr;
	fs << "maxReprojErr" << maxErr;
	fs << "R"<< R;
	fs << "T" << T;
	fs << "E" << E;
	fs << "F" << F;
	fs << "}";
}
