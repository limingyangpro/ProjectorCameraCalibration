/*
 * lgc2D_test.hpp
 *
 *  Created on: Jul 20, 2015
 *      Author: liming
 */

#ifndef MATCHINGLIB_LGCLIB_LGC2D_TEST_HPP_
#define MATCHINGLIB_LGCLIB_LGC2D_TEST_HPP_

#include <dirent.h>
#include "lgcmethod.hpp"
#include "affine2D.hpp"
#include "perspective2D.hpp"

/**
 * Read point set coordinate from specified file
 */
template <typename PointType>
bool readCoordinateFromFile(string filename, vector<PointType> &pointarray)
{
	ifstream file(filename.c_str());
	if (!file.is_open())         //Check if the corresponding file exists
	{
		cout<<"Fail to open file "<< filename << endl;
		return false;
	}
	// Deal with all the coordinates

	int totalpoints;
	PointType tmppoint;
	pointarray.clear();

	file >> totalpoints;

	for(int i=0;i<totalpoints;i++)
	{
		file >> tmppoint.x >> tmppoint.y;
		pointarray.push_back(tmppoint);
	}

	file.close();

	return true;
}

/**
 * Read point set coordinate from specified directory, each file represents a point set
 */
template <typename PointType>
void readCoordinateFromDir(string dir, string fileextension, vector< vector<PointType> > &pointsetarray)
{
	unsigned char isFile =0x8;
	DIR *Dir = opendir(dir.c_str());
	struct dirent *DirEntry;
	vector<PointType> tempRef;

	pointsetarray.clear();

	set<string> filetoread;
	filetoread.clear();

	while(DirEntry=readdir(Dir))
	{
	   if ( DirEntry->d_type == isFile)
	   {
		   string filename(DirEntry->d_name);
		   if (filename.compare(filename.size()-3,3,fileextension) == 0)
		   {
			   filetoread.insert(filename);
		   }
	   }
	}

	for (set<string>::iterator itFile = filetoread.begin() ; itFile != filetoread.end() ; ++itFile)
	{
		cout <<"Read from : " << dir+"/"+ (*itFile) << endl;
		readCoordinateFromFile(dir+"/"+(*itFile), tempRef);
		pointsetarray.push_back(tempRef);
	}
}

/**
 * Read expected result from specified file, the result is recorded in a map
 * resultmap[refID] = imgID;
 */
bool readResultFromFile(string filename, map<int,int> &resultmap)
{
	ifstream file(filename.c_str());
	if (!file.is_open())         //Check if the corresponding file exists
	{
		cout<<"Fail to open file "<< filename << endl;
		return false;
	}
	// Deal with all the coordinates

	int recordnum, refID, imgID;

	file >> recordnum;

	resultmap.clear();
	for(int i=0;i<recordnum;i++)
	{
		file >> refID >> imgID;
		resultmap[refID] = imgID;
	}

	file.close();

	return true;
}

/**
 * Read expected results from specified directory, each file represents a result relative to a point set
 */
void readResultFromDir(string dir, vector< map<int,int> > &resultmaparray)
{
	unsigned char isFile =0x8;
	DIR *Dir = opendir(dir.c_str());
	struct dirent *DirEntry;
	map<int,int> tempresult;

	resultmaparray.clear();
	set<string> filetoread;
	filetoread.clear();

	while(DirEntry=readdir(Dir))
	{
	   if ( DirEntry->d_type == isFile)
	   {
		   string filename(DirEntry->d_name);
		   if (filename.compare(filename.size()-3,3,"res") == 0)
		   {
			   filetoread.insert(filename);
		   }
	   }
	}

	for (set<string>::iterator itFile = filetoread.begin() ; itFile != filetoread.end() ; ++itFile)
	{
		cout <<"Read from : " << dir+"/"+(*itFile) << endl;
		readResultFromFile(dir+"/"+ (*itFile), tempresult);
		resultmaparray.push_back(tempresult);
	}
}

class LGC2D_for_test : public lgc2::LGCmethod<pcl::PointXY>
{
public:
	LGC2D_for_test()
	{
		generateSameRandomSeed = true;    //Use the same rand seed for this test
		localTransform = new lgc2::Affine2D(1.3f,10.0f);
		globalTransform = new lgc2::Perspective2D();
	}

	virtual ~LGC2D_for_test()
	{
		if (localTransform != NULL)
		{
			delete localTransform;
		}
		if (globalTransform != NULL)
		{
			delete globalTransform;
		}
		if (generatorPtr != NULL)
		{
			delete generatorPtr;
		}
		if (param != NULL)
		{
			delete param;
		}
	}

	void set_eta(float eta)
	{
		eta = (eta < 0.02 ? 0.02 : eta);
		param = new lgc2::Parameter(
				6, 6,   //kr, ki
				eta,   //eta
				10, 1000,
				1.3, 100,
				2, 0.1,
				45, 20,
				8, 4);
		generatorPtr = new lgc2::Generator2Daff(param,localTransform);
	}

	bool runMatchingTestWithInput(string reffilename, string imgfilename, string resultfilename)
	{
		vector<pcl::PointXY> refpts, imgpts;
		map<int,int> expectedResult, testResult;

		readCoordinateFromFile(reffilename,refpts);
		readCoordinateFromFile(imgfilename,imgpts);
		readResultFromFile(resultfilename, expectedResult);

		int model;

		setRefPoints(&refpts);
		setImgPointsAndFindOneModel(&imgpts, NULL, model, NULL);

		//Test result:
		testResult.clear();
		for (typename list< boost::shared_ptr< lgc2::Correspondance_<pcl::PointXY> > >::iterator itList = resultList.begin() ; itList != resultList.end() ; ++itList)
		{
//			cout<<(*itList)->refkeypt->id<<" "<<(*itList)->imgkeypt->id<<endl;
			testResult[(*itList)->refkeypt->id] = (*itList)->imgkeypt->id;
		}

		return (expectedResult == testResult);
	}

	bool runTrackingTestWithInput(string directory)
	{
		vector< vector<pcl::PointXY> > refpts, imgpts;
		vector< map<int,int> > expectedResult;
		map<int,int> testResult;

		readCoordinateFromDir(directory, "ref", refpts);   //only get one point set normally
		readCoordinateFromDir(directory, "img", imgpts);
		readResultFromDir(directory, expectedResult);

		int model;
		setRefPoints(&refpts[0]);

		if (imgpts.size() != expectedResult.size())
		{
			cout<<"The frame number is not equal to the result number !!"<<endl;
			return false;
		}

		for (unsigned int frameNum = 0 ; frameNum < imgpts.size() ; frameNum++)
		{
			setImgPointsAndFindOneModel(&imgpts[frameNum], NULL, model, NULL);
			//Test result:
			testResult.clear();
			for (typename list< boost::shared_ptr< lgc2::Correspondance_<pcl::PointXY> > >::iterator itList = resultList.begin() ; itList != resultList.end() ; ++itList)
			{
				testResult[(*itList)->refkeypt->id] = (*itList)->imgkeypt->id;
			}

			if (expectedResult[frameNum] != testResult) return false;
		}

		return true;
	}
};

#endif /* MATCHINGLIB_LGCLIB_LGC2D_TEST_HPP_ */
