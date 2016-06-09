/*
 * analyseresult.cpp
 *
 *  Created on: Dec 16, 2015
 *      Author: liming
 *
 *      Analyst and compare calibration results
 */

#include <iostream>
#include <dirent.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace std;

vector<string> readRefFromDir(string dir)
{
	unsigned char isFile =0x8;
	DIR *Dir = opendir(dir.c_str());
	struct dirent *DirEntry;
	vector<string> fileList;
	while(DirEntry=readdir(Dir))
	{
	   if ( DirEntry->d_type == isFile)
	   {
		   string filename(DirEntry->d_name);
		   if ( (filename.compare(filename.size()-4,4,"yaml") == 0)
				   ||(filename.compare(filename.size()-3,3,"xml") == 0))
		   {
			   cout <<"Found a File : " << dir+"/"+filename << endl;
			   fileList.push_back(dir+"/"+filename);
		   }
	   }
	}

	sort(fileList.begin(), fileList.end());
	return fileList;
}

class View
{
public:
	cv::Mat_<float> rVec;       //rotation vector  1*3
	cv::Mat_<float> tVec;       //translation vector  1*3
	float err;          //view error
};

class Device
{
public:
	cv::Mat_<double> intrinsicMatrix;
	cv::Mat_<double> distortionMatrix;
	cv::Mat_<double> externalParams;
	cv::Mat_<double> view_errs;
	double avg_Err, max_Err;

	void readFromNode(const cv::FileNode& node)
	{
		cv::Mat tmpMatrix;
		node["cameraMatrix"] >> tmpMatrix;
		tmpMatrix.convertTo(intrinsicMatrix, CV_64F);
		node["distortionCoeffs"] >> tmpMatrix;
		tmpMatrix.convertTo(distortionMatrix, CV_64F);
		node["extrParams"] >> tmpMatrix;
		tmpMatrix.convertTo(externalParams, CV_64F);
		node["reprojErrs"] >> tmpMatrix;
		tmpMatrix.convertTo(view_errs, CV_64F);

		avg_Err = (double) node["avgReprojErr"];
		max_Err = (double) node["maxReprojErr"];
	}

	void print()
	{
		cout<<"intrinsicMatrix="<<intrinsicMatrix<<endl;
		cout<<"distortionMatrix="<<distortionMatrix<<endl;
		cout<<"externalParams="<<externalParams<<endl;
		cout<<"view_errs="<<view_errs<<endl;
		cout<<"avg_Err="<<avg_Err<<endl;
		cout<<"max_Err="<<max_Err<<endl;
	}
};

class CalibrationResult
{
public:
	Device camera, projector;
	string filename;

	bool readResultFile(string parameterfile)
	{
		filename = parameterfile;

	    cv::FileStorage fs;
	    cv::Mat camMat, distortionMat;
	    fs.open(parameterfile, cv::FileStorage::READ);

	    if (!fs.isOpened())
	    {
	        cerr << "Failed to open " << parameterfile << endl;
	        return false;
	    }
	    else
	    {
	    	cout << "Read camera parameter from " << parameterfile << endl;
	    }

	    cv::FileNode camera_node = fs["Camera  0"];
	    camera.readFromNode(camera_node);

	    cv::FileNode projector_node = fs["Projector  0"];
	    projector.readFromNode(projector_node);

	    fs.release();
	    return true;
	}

};

void printSourceFiles(vector<string> &srcfiles)
{
	cout<<"Source files: "<<endl;
	for (unsigned int i = 0 ; i < srcfiles.size() ; i++)
	{
		cout<<srcfiles[i]<<endl;
	}
}

void printIntrinsics(vector< CalibrationResult >& data)
{
	unsigned int experimentNum = data.size();
	cv::Mat_<double> camMat = cv::Mat::zeros(experimentNum, 4, CV_32F);
	cv::Mat_<double> camDistorsion = cv::Mat::zeros(experimentNum, 4, CV_32F);

	for (unsigned int i = 0 ; i < experimentNum ; i++)
	{
		camMat[i][0] = data[i].camera.intrinsicMatrix[0][0];
		camMat[i][1] = data[i].camera.intrinsicMatrix[1][1];
		camMat[i][2] = data[i].camera.intrinsicMatrix[0][2];
		camMat[i][3] = data[i].camera.intrinsicMatrix[1][2];
	}
	cout<<"Camera Intrinsics : "<<endl;
	cout<<camMat<<endl;

	for (unsigned int i = 0 ; i < experimentNum ; i++)
	{
		camDistorsion[i][0] = data[i].camera.distortionMatrix[0][0];
		camDistorsion[i][1] = data[i].camera.distortionMatrix[0][1];
		camDistorsion[i][2] = data[i].camera.distortionMatrix[0][2];
		camDistorsion[i][3] = data[i].camera.distortionMatrix[0][3];
	}
	cout<<"Camera Distorsion : "<<endl;
	cout<<camDistorsion<<endl;


	cv::Mat_<double> projectorMat(experimentNum, 4);
	cv::Mat_<double> projectorDistorsion(experimentNum, 4);
	for (unsigned int i = 0 ; i < experimentNum ; i++)
	{
		projectorMat[i][0] = data[i].projector.intrinsicMatrix[0][0];
		projectorMat[i][1] = data[i].projector.intrinsicMatrix[1][1];
		projectorMat[i][2] = data[i].projector.intrinsicMatrix[0][2];
		projectorMat[i][3] = data[i].projector.intrinsicMatrix[1][2];
	}
	cout<<"Projector Intrinsics : "<<endl;
	cout<<projectorMat<<endl;

	for (unsigned int i = 0 ; i < experimentNum ; i++)
	{
		projectorDistorsion[i][0] = data[i].projector.distortionMatrix[0][0];
		projectorDistorsion[i][1] = data[i].projector.distortionMatrix[0][1];
		projectorDistorsion[i][2] = data[i].projector.distortionMatrix[0][2];
		projectorDistorsion[i][3] = data[i].projector.distortionMatrix[0][3];
	}
	cout<<"Projector Distorsion : "<<endl;
	cout<<projectorDistorsion<<endl;
}

void printOrientations(vector< CalibrationResult >& data)
{
	unsigned int experimentNum = data.size();
	unsigned int maxViewNum = 0;
	for (unsigned int i = 0 ; i < experimentNum ; i++)
	{
		maxViewNum = data[i].camera.externalParams.rows > maxViewNum ? data[i].camera.externalParams.rows : maxViewNum;
	}

	cv::Mat_<double> camAngleMat = - cv::Mat::ones(experimentNum, maxViewNum, CV_32F);
	cv::Mat_<double> prjAngleMat = - cv::Mat::ones(experimentNum, maxViewNum, CV_32F);
	cv::Mat_<double> rotMat, rotVec;
	for (unsigned int i = 0 ; i < experimentNum ; i++)
	{
		for (unsigned int v = 0 ; v < data[i].camera.externalParams.rows ; v++)
		{
			rotVec = data[i].camera.externalParams(cv::Rect(0,v,3,1));
			cv::Rodrigues(rotVec, rotMat);
			camAngleMat[i][v] = acos(rotMat[2][2])*180/CV_PI;

			rotVec = data[i].projector.externalParams(cv::Rect(0,v,3,1));
			cv::Rodrigues(rotVec, rotMat);
			prjAngleMat[i][v] = acos(rotMat[2][2])*180/CV_PI;
		}
	}

	cout<<"Board angle in camera's view : "<<endl;
	cout<<camAngleMat<<endl;

	cout<<"Board angle in projector's view : "<<endl;
	cout<<prjAngleMat<<endl;
}

void printTotalErrors(vector< CalibrationResult >& data)
{
	unsigned int experimentNum = data.size();
	cv::Mat_<double> camErrorMat = - cv::Mat::ones(experimentNum, 2, CV_32F);
	cv::Mat_<double> prjErrorMat = - cv::Mat::ones(experimentNum, 2, CV_32F);

	for (unsigned int i = 0 ; i < experimentNum ; i++)
	{
		camErrorMat[i][0] = data[i].camera.avg_Err;
		camErrorMat[i][1] = data[i].camera.max_Err;
		prjErrorMat[i][0] = data[i].projector.avg_Err;
		prjErrorMat[i][1] = data[i].projector.max_Err;
	}

	cout<<"Errors of camera : "<<endl;
	cout<<camErrorMat<<endl;

	cout<<"Errors of projector : "<<endl;
	cout<<prjErrorMat<<endl;
}

int main( int argc, char **argv )
{
	vector< CalibrationResult > results;
	vector<string> files = readRefFromDir(string(argv[1]));
	unsigned int experimentNum = files.size();
	results.resize(experimentNum);

	for (unsigned int i = 0 ; i < experimentNum ; i++)
	{
		results[i].readResultFile(files[i]);
	}

	printSourceFiles(files);
	printIntrinsics(results);
	printOrientations(results);
	printTotalErrors(results);
	return 0;
}


