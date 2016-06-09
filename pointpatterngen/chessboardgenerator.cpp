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
	int squareSize;
	string basicname;
	int white_boarder_size = 0;
	cout<<"------------------------Functionality--------------------------"<<endl
			<<"1 - Draw random point pattern and write to \".txt\" file." <<endl
			<<"2 - Draw random point patter from \".txt\" file."<<endl
			<<"--------------------------------------------------------------"<<endl;
	while((oc = getopt(argc, argv, "?u:w:h:n:s:n:dA:")) != -1)
	{
		switch (oc)
		{
		case '?':
			 	cout<< "For Example 1 (generate mode): chessboardgenerator -w1280 -h720 -s40 -n basicname"<<endl
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
			};
			break;
		case 's':
			squareSize = atoi(optarg);
			break;
		case 'n':
			basicname = optarg;
			break;
		default :
			cout<<"Incorrect Argument!! "<<endl;
			return 0;
		}
	}

	int colNum = int ( (width - squareSize*2)/squareSize );
	int rowNum = int ( (height - squareSize*2)/squareSize );
	int x_offset = (width - (colNum-1)*squareSize)/2 - squareSize;
	int y_offset = (height - (rowNum-1)*squareSize)/2 - squareSize;

	assert(basicname.size() > 0);

	fstream referencefile;
	vector< vector<cv::Point2f> > corners;
	if (!drawingmode)            //write to txt file
	{
		corners.resize(rowNum+1);
		//Generate the point array
		for (int i = 0 ; i < rowNum+1 ; i++)
		{
			corners[i].resize(colNum+1);
			for (int j = 0 ; j < colNum+1 ; j++)
			{
				cv::Point2f corner( j*squareSize + x_offset, i*squareSize + y_offset);
				corners[i][j] = corner;
				cout<<corner<<" ";
			}
			cout<<endl;
		}

		//Draw
		cv::Mat chessBoard(height, width, CV_8UC3, cv::Scalar::all(255));
		unsigned char color=0;
		for (int i = 0 ; i < rowNum+1 ; i++)
		{
			if (colNum % 2 == 1)
			{
				color=~color;
			}

		      for (int j = 0 ; j < colNum+1 ; j++)
		      {
		    	  cv::Mat ROI=chessBoard(cv::Rect(corners[i][j].x, corners[i][j].y, squareSize,squareSize));
		    	  ROI.setTo(cv::Scalar::all(color));
		    	  color=~color;
		      }
		}

		//direction circle
		cv::circle(chessBoard, cv::Point2f(100,100), 30, cv::Scalar::all(0), -1);

		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(0);

		imwrite(basicname + "Print.png", chessBoard, compression_params);

		cv::Mat bgr[3], red;
		split(chessBoard,bgr);
		bgr[0].setTo(0);
		bgr[1].setTo(0);
		vector<cv::Mat> channel;
		channel.push_back(bgr[0]);
		channel.push_back(bgr[1]);
		channel.push_back(bgr[2]);
		merge(channel, red);
		imwrite(basicname + "Project.png", red);

		vector<cv::Point2f> detectedcorners;
		bool found = cv::findChessboardCorners( chessBoard, cv::Size(colNum, rowNum), detectedcorners,
		                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

		for (int i = 0 ; i < detectedcorners.size() ; i++)
		{
			if (i%colNum == 0)
			{
				cout<<endl;
			}
			cout<<detectedcorners[i]<<" ";
		}

		//Write corners
		referencefile.open(string(basicname + "Print.txt").c_str(),std::fstream::out);
		if(!referencefile.is_open())
		{
			cout<<"Can't open output file correctly !"<<endl;
			return 0;
		}
		referencefile<<"0 0 "<<width<<" "<<height<<endl;
		referencefile<<colNum<<" "<<rowNum<<endl;

		for (int i = 0 ; i < detectedcorners.size() ; i++)
		{
			referencefile<<detectedcorners[i].x<<" "<<detectedcorners[i].y<<endl;
		}

		referencefile.close();
	}


}

