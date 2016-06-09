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
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
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
	cv::Mat_<double> stereo_t;

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
		node["T"] >> tmpMatrix;
		tmpMatrix.convertTo(stereo_t, CV_64F);
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

class Summary
{
public:
	vector<string> fileName;
	vector<double> camFx;
	vector<double> camFy;
	vector<double> camu0;
	vector<double> camv0;
	vector< vector<double> > camDistorsion;  //id0 = distorsion index; id1 = experiment num

	vector<double> proFx;
	vector<double> proFy;
	vector<double> prou0;
	vector<double> prov0;
	vector< vector<double> > proDistorsion;

	vector<double> cam_meanErr;
	vector<double> cam_maxErr;
	vector<double> pro_meanErr;
	vector<double> pro_maxErr;

	vector< vector<double> > camAngles;        //id0 = experiment number; id1 = view number
	vector< vector<double> > proAngles;

	//Stereo info
	vector< cv::Mat_<double> > t;          //distance between camera and projector

private:
	void setFileNames(vector< CalibrationResult >& data)
	{
		unsigned int experimentNum = data.size();
		this->fileName.resize(experimentNum);
		for (unsigned int i = 0 ; i < experimentNum ; i++)
		{
			this->fileName[i] = data[i].filename;
		}
	}

	void setIntrinsics(vector< CalibrationResult >& data)
	{
		unsigned int experimentNum = data.size();
		camFx.resize(experimentNum);
		camFy.resize(experimentNum);
		camu0.resize(experimentNum);
		camv0.resize(experimentNum);

		for (unsigned int i = 0 ; i < experimentNum ; i++)
		{
			camFx[i] = data[i].camera.intrinsicMatrix[0][0];
			camFy[i] = data[i].camera.intrinsicMatrix[1][1];
			camu0[i] = data[i].camera.intrinsicMatrix[0][2];
			camv0[i] = data[i].camera.intrinsicMatrix[1][2];
		}

		proFx.resize(experimentNum);
		proFy.resize(experimentNum);
		prou0.resize(experimentNum);
		prov0.resize(experimentNum);

		for (unsigned int i = 0 ; i < experimentNum ; i++)
		{
			proFx[i] = data[i].projector.intrinsicMatrix[0][0];
			proFy[i] = data[i].projector.intrinsicMatrix[1][1];
			prou0[i] = data[i].projector.intrinsicMatrix[0][2];
			prov0[i] = data[i].projector.intrinsicMatrix[1][2];
		}
	}

	void setDistorsions(vector< CalibrationResult >& data)
	{
		unsigned int experimentNum = data.size();

		camDistorsion.resize(4);
		proDistorsion.resize(4);

		for (unsigned int i = 0 ; i < 4 ; i++)
		{
			camDistorsion[i].resize(experimentNum);
			proDistorsion[i].resize(experimentNum);
		}

		for (unsigned int i = 0 ; i < experimentNum ; i++)
		{
			camDistorsion[0][i] = data[i].camera.distortionMatrix[0][0];
			camDistorsion[1][i] = data[i].camera.distortionMatrix[0][1];
			camDistorsion[2][i] = data[i].camera.distortionMatrix[0][2];
			camDistorsion[3][i] = data[i].camera.distortionMatrix[0][3];

			proDistorsion[0][i] = data[i].projector.distortionMatrix[0][0];
			proDistorsion[1][i] = data[i].projector.distortionMatrix[0][1];
			proDistorsion[2][i] = data[i].projector.distortionMatrix[0][2];
			proDistorsion[3][i] = data[i].projector.distortionMatrix[0][3];
		}
	}

	void setErr(vector< CalibrationResult >& data)
	{
		unsigned int experimentNum = data.size();
		cam_meanErr.resize(experimentNum);
		cam_maxErr.resize(experimentNum);
		pro_meanErr.resize(experimentNum);
		pro_maxErr.resize(experimentNum);
		for (unsigned int i = 0 ; i < experimentNum ; i++)
		{
			cam_meanErr[i] = data[i].camera.avg_Err;
			cam_maxErr[i] = data[i].camera.max_Err;
			pro_meanErr[i] = data[i].projector.avg_Err;
			pro_maxErr[i] = data[i].projector.max_Err;
		}
	}

	void setOrientations(vector< CalibrationResult >& data)
	{
		unsigned int experimentNum = data.size();
		camAngles.resize(experimentNum);
		proAngles.resize(experimentNum);
		unsigned int maxViewNum = 0;
		for (unsigned int i = 0 ; i < experimentNum ; i++)
		{
			camAngles[i].resize(data[i].camera.externalParams.rows);
			proAngles[i].resize(data[i].camera.externalParams.rows);
		}

		cv::Mat_<double> rotMat, rotVec;
		for (unsigned int i = 0 ; i < experimentNum ; i++)
		{
			for (unsigned int v = 0 ; v < data[i].camera.externalParams.rows ; v++)
			{
				rotVec = data[i].camera.externalParams(cv::Rect(0,v,3,1));
				cv::Rodrigues(rotVec, rotMat);
				camAngles[i][v] = acos(rotMat[2][2])*180/CV_PI;

				rotVec = data[i].projector.externalParams(cv::Rect(0,v,3,1));
				cv::Rodrigues(rotVec, rotMat);
				proAngles[i][v] = acos(rotMat[2][2])*180/CV_PI;
			}
		}
	}

	void setStereoInfo(vector< CalibrationResult >& data)
	{
		unsigned int experimentNum = data.size();
		t.resize(experimentNum);

		cv::Mat_<double> rotMat, rotVec;
		for (unsigned int i = 0 ; i < experimentNum ; i++)
		{
			t[i] = data[i].projector.stereo_t;
		}
	}

public:
	void setAll(vector< CalibrationResult >& data)
	{
		setFileNames(data);
		setIntrinsics(data);
		setDistorsions(data);
		setErr(data);
		setOrientations(data);
		setStereoInfo(data);
	}

	static void printCamIntrinsics(vector<Summary> summarys)
	{
		unsigned int num = summarys.size();
		cout<<"camFx:"<<endl;
		for (unsigned int i = 0 ; i < num ; i++)
		{
			cout<<summarys[i].fileName[0]<<",";
			for (unsigned int j = 0 ; j < summarys[i].camFx.size() ; j++)
			{
				cout<<summarys[i].camFx[j]<<",";
			}
			cout<<endl;
		}

		cout<<"camFy:"<<endl;
		for (unsigned int i = 0 ; i < num ; i++)
		{
			cout<<summarys[i].fileName[0]<<",";
			for (unsigned int j = 0 ; j < summarys[i].camFx.size() ; j++)
			{
				cout<<summarys[i].camFy[j]<<",";
			}
			cout<<endl;
		}

		cout<<"camu0:"<<endl;
		for (unsigned int i = 0 ; i < num ; i++)
		{
			cout<<summarys[i].fileName[0]<<",";
			for (unsigned int j = 0 ; j < summarys[i].camFx.size() ; j++)
			{
				cout<<summarys[i].camu0[j]<<",";
			}
			cout<<endl;
		}

		cout<<"camv0:"<<endl;
		for (unsigned int i = 0 ; i < num ; i++)
		{
			cout<<summarys[i].fileName[0]<<",";
			for (unsigned int j = 0 ; j < summarys[i].camFx.size() ; j++)
			{
				cout<<summarys[i].camv0[j]<<",";
			}
			cout<<endl;
		}
	}

	static void printProIntrinsics(vector<Summary> summarys)
	{
		unsigned int num = summarys.size();
		cout<<"proFx:"<<endl;
		for (unsigned int i = 0 ; i < num ; i++)
		{
			cout<<summarys[i].fileName[0]<<",";
			for (unsigned int j = 0 ; j < summarys[i].proFx.size() ; j++)
			{
				cout<<summarys[i].proFx[j]<<",";
			}
			cout<<endl;
		}

		cout<<"proFy:"<<endl;
		for (unsigned int i = 0 ; i < num ; i++)
		{
			cout<<summarys[i].fileName[0]<<",";
			for (unsigned int j = 0 ; j < summarys[i].proFx.size() ; j++)
			{
				cout<<summarys[i].proFy[j]<<",";
			}
			cout<<endl;
		}

		cout<<"prou0:"<<endl;
		for (unsigned int i = 0 ; i < num ; i++)
		{
			cout<<summarys[i].fileName[0]<<",";
			for (unsigned int j = 0 ; j < summarys[i].proFx.size() ; j++)
			{
				cout<<summarys[i].prou0[j]<<",";
			}
			cout<<endl;
		}

		cout<<"prov0:"<<endl;
		for (unsigned int i = 0 ; i < num ; i++)
		{
			cout<<summarys[i].fileName[0]<<",";
			for (unsigned int j = 0 ; j < summarys[i].proFx.size() ; j++)
			{
				cout<<summarys[i].prov0[j]<<",";
			}
			cout<<endl;
		}
	}

	static void printCamErr(vector<Summary> summarys)
	{
		unsigned int num = summarys.size();
		cout<<"camErrMean:"<<endl;
		for (unsigned int i = 0 ; i < num ; i++)
		{
			cout<<summarys[i].fileName[0]<<",";
			for (unsigned int j = 0 ; j < summarys[i].cam_meanErr.size() ; j++)
			{
				cout<<summarys[i].cam_meanErr[j]<<",";
			}
			cout<<endl;
		}

		cout<<"camErrMax:"<<endl;
		for (unsigned int i = 0 ; i < num ; i++)
		{
			cout<<summarys[i].fileName[0]<<",";
			for (unsigned int j = 0 ; j < summarys[i].cam_maxErr.size() ; j++)
			{
				cout<<summarys[i].cam_maxErr[j]<<",";
			}
			cout<<endl;
		}
	}

	static void printProErr(vector<Summary> summarys)
	{
		unsigned int num = summarys.size();
		cout<<"proErrMean:"<<endl;
		for (unsigned int i = 0 ; i < num ; i++)
		{
			cout<<summarys[i].fileName[0]<<",";
			for (unsigned int j = 0 ; j < summarys[i].pro_meanErr.size() ; j++)
			{
				cout<<summarys[i].pro_meanErr[j]<<",";
			}
			cout<<endl;
		}

		cout<<"proErrMax:"<<endl;
		for (unsigned int i = 0 ; i < num ; i++)
		{
			cout<<summarys[i].fileName[0]<<",";
			for (unsigned int j = 0 ; j < summarys[i].pro_maxErr.size() ; j++)
			{
				cout<<summarys[i].pro_maxErr[j]<<",";
			}
			cout<<endl;
		}
	}

	static void printStereo_t(vector<Summary> summarys)
	{
		unsigned int num = summarys.size();
		cout<<"stereo_t_x:"<<endl;
		for (unsigned int i = 0 ; i < num ; i++)
		{
			cout<<summarys[i].fileName[0]<<",";
			for (unsigned int j = 0 ; j < summarys[i].t.size() ; j++)
			{
				cout<<summarys[i].t[j].at<double>(0,0)<<",";
			}
			cout<<endl;
		}

		cout<<"stereo_t_y:"<<endl;
		for (unsigned int i = 0 ; i < num ; i++)
		{
			cout<<summarys[i].fileName[0]<<",";
			for (unsigned int j = 0 ; j < summarys[i].t.size() ; j++)
			{
				cout<<summarys[i].t[j].at<double>(1,0)<<",";
			}
			cout<<endl;
		}

		cout<<"stereo_t_z:"<<endl;
		for (unsigned int i = 0 ; i < num ; i++)
		{
			cout<<summarys[i].fileName[0]<<",";
			for (unsigned int j = 0 ; j < summarys[i].t.size() ; j++)
			{
				cout<<summarys[i].t[j].at<double>(2,0)<<",";
			}
			cout<<endl;
		}
	}
};

int main( int argc, char **argv )
{
	int numSummary = argc - 1;
	vector<Summary> summarys(numSummary);
	for (int s = 0 ; s < numSummary ; s++)
	{
		vector< CalibrationResult > results;
		vector<string> files = readRefFromDir(string(argv[s+1]));
		unsigned int experimentNum = files.size();
		results.resize(experimentNum);

		for (unsigned int i = 0 ; i < experimentNum ; i++)
		{
			results[i].readResultFile(files[i]);
		}

		summarys[s].setAll(results);
	}

	Summary::printCamIntrinsics(summarys); cout<<endl;
	Summary::printProIntrinsics(summarys); cout<<endl;
	Summary::printCamErr(summarys); cout<<endl;
	Summary::printProErr(summarys); cout<<endl;
	Summary::printStereo_t(summarys); cout<<endl;

//	printSourceFiles(files);
//	printDistorsions(results);
//
//	printOrientations(results);
//	printTotalErrors(results);
	return 0;
}


