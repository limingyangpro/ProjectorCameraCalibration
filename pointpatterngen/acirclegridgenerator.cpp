/*
 * chessboardgenerator.cpp
 *
 *  Created on: Feb 27, 2016
 *      Author: liming
 */


#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <unistd.h>
#include <time.h>       /* time */
#include <fstream>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/opencv.hpp"
using namespace std;

//#include "basictype.h"

using namespace std;

int main(int argc, char **argv)
{
	float tmp1,tmp2,tmp3,tmp4;
	bool drawingmode = false;
	cout<<sizeof(unsigned char)<<endl;
	char oc;
	int width,height;
	int paperType;
	int circleDistance;
	string basename;
	int white_boarder_size = 0;
	cout<<"------------------------Functionality--------------------------"<<endl
			<<"1 - Draw random point pattern and write to \".txt\" file." <<endl
			<<"2 - Draw random point patter from \".txt\" file."<<endl
			<<"--------------------------------------------------------------"<<endl;
	while((oc = getopt(argc, argv, "?u:w:h:n:s:i:r:dA:")) != -1)
	{
		switch (oc)
		{
		case '?':
			 	cout<< "For Example 1 (generate mode): pointpatterngen -w1280 -h720 -s40 -i\"testimage.jpg\" -r\"testref.txt\""<<endl
			 	 << "For Example 2 : -A 2 - s 400 -n circleA2"<<endl
			 	 << endl;
			return 0;
		case 'w':
			width = atoi(optarg);
			break;
		case 'h':
			height = atoi(optarg);
			break;
		case 'A':
			paperType = atoi(optarg);
			switch (paperType)
			{
			case 4:
				height = 2100;
				width = 2970;

				break;
			case 3:
				height = 2970;
				width = 4200;

				break;
			case 2:
				height = 4200;
				width = 5940;
				break;
			case 1:
				height = 5940;
				width = 8410;
				break;
			case 0:
				height = 8410;
				width = 11880;
				break;
			};
			break;
		case 's':
			circleDistance = atoi(optarg);
			break;
		case 'n':
			basename = optarg;
			break;
		default :
			cout<<"Incorrect Argument!! "<<endl;
			return 0;
		}
	}

	int colNum = int ( (width - circleDistance*2)/circleDistance );
	int rowNum = int ( (height - circleDistance*2)/circleDistance );

	if (colNum % 2 == 0) colNum --;   //make the colNum to be odd
	if (rowNum % 2 == 1) rowNum --;

	colNum = 11;
	rowNum = 8;

	int x_offset = (width - (colNum-1)*circleDistance)/2;
	int y_offset = (height - (rowNum-1)*circleDistance)/2;

	assert(basename.size() > 0);

	fstream referencefile;
	vector<cv::Point2f> printCircles;
	vector<cv::Point2f> projectedCircles;
	if (!drawingmode)            //write to txt file
	{
		for (int i = 0 ; i < rowNum ; i++)
		{
			for (int j = 0 ; j < colNum ; j++)
			{
				cv::Point2f center( j*circleDistance + x_offset, i*circleDistance + y_offset);
				if ( (j + i) % 2 == 0)
				{
					printCircles.push_back(center);
				}
				else
				{
					projectedCircles.push_back(center);
				}
			}
		}

		//Draw
		float circleRadius = circleDistance*0.3;
		cv::Mat printImg(height, width, CV_8UC1, cv::Scalar::all(255));   //circles are black
		for (unsigned int i = 0 ; i < printCircles.size() ; i++)
		{
			cv::circle(printImg, printCircles[i], circleRadius, cv::Scalar::all(0), -1);
		}

		cv::Mat projectedImg(height, width, CV_8UC1, cv::Scalar::all(0));    //circles are white
		for (unsigned int i = 0 ; i < projectedCircles.size() ; i++)
		{
			cv::circle(projectedImg, projectedCircles[i], circleRadius, cv::Scalar::all(255), -1);
		}

		imwrite( basename + "Print.png", printImg);
		imwrite( basename + "Projected.png", projectedImg);


		//Detect circles
		//Add a customized blob detector since sometimes the blobs are too big
		cv::SimpleBlobDetector::Params detectorparams;
		detectorparams.filterByColor = true;        //Find black points
		detectorparams.blobColor = 0;

		detectorparams.minDistBetweenBlobs = 0.0;  // minimum 10 pixels between blobs

		detectorparams.filterByArea = true;
		detectorparams.minArea = 5.0;              				// min 5 pixels squared
		detectorparams.maxArea = circleRadius*circleRadius*4;             // max 200 pixels squared

		//Without specifying "minConvexity, some floued points can be missed
		detectorparams.filterByConvexity = true;         // filter  by area of blob
		detectorparams.minConvexity = 0.5;              // min 5 pixels squared

		cv::Ptr< cv::SimpleBlobDetector > blobDetector = cv::SimpleBlobDetector::create(detectorparams);

		vector<cv::Point2f> detectedCircles;
		bool found = cv::findCirclesGrid( printImg, cv::Size(rowNum/2, colNum), detectedCircles, cv::CALIB_CB_ASYMMETRIC_GRID, blobDetector);

		//Write circles for print one
		referencefile.open( string(basename + "Print.txt").c_str(),std::fstream::out);
		if(!referencefile.is_open())
		{
			cout<<"Can't open output file correctly !"<<endl;
			return 0;
		}
		referencefile<<"0 0 "<<width<<" "<<height<<endl;
		referencefile<<colNum<<" "<<rowNum<<endl;
		referencefile<<detectedCircles.size()<<endl;

		/**
		 *     17    9    1
		 *        13    5
		 *     18    10   2
		 *        14    6
		 *     19    11   3
		 *        15    7
		 *     20    12   4
		 *        16    8
		 */
		for (int i = 0 ; i < detectedCircles.size() ; i++)
		{
			//It writes the first column (even dimension), then second column...
			cout<<detectedCircles[i]<<" ";
			referencefile<<detectedCircles[i].x<<" "<<detectedCircles[i].y<<endl;
		}
		cout<<endl;
		referencefile.close();

		//Detect circles
		found = cv::findCirclesGrid( 255 - projectedImg, cv::Size(rowNum/2, colNum), detectedCircles, cv::CALIB_CB_ASYMMETRIC_GRID, blobDetector  );

		//Write circles for projected one
		referencefile.open(string(basename + "Projected.txt").c_str(),std::fstream::out);
		if(!referencefile.is_open())
		{
			cout<<"Can't open output file correctly !"<<endl;
			return 0;
		}
		referencefile<<"0 0 "<<width<<" "<<height<<endl;
		referencefile<<colNum<<" "<<rowNum<<endl;
		referencefile<<detectedCircles.size()<<endl;

		for (int i = 0 ; i < detectedCircles.size() ; i++)
		{
			//It writes the first row, then second row...
			cout<<detectedCircles[i]<<" ";
			referencefile<<detectedCircles[i].x<<" "<<detectedCircles[i].y<<endl;
		}
		referencefile.close();


	}


}

