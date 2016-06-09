/*
 * augmentation.cpp
 *
 *  Created on: Feb 26, 2016
 *      Author: liming
 */


/**
 * Use:
 *
 * augmentation -c calibrationresult.xml
 *
 *
 */
#include "auglib/augmentenvironment.h"
using namespace std;

int main( int argc, char **argv )
{
	bool recordVideo = false;
    string calibrationFile = "0";
    string inputStream = "0";

    string augmentingContentFile = "latestResult.xml";
    string boardPatternFile;
    string groundtruthfile = "";
    string testsuite;
    string errorFile = "error";

	char oc;
	while((oc = getopt(argc, argv, "?i:a:b:c:g:s:r:e:")) != -1)
	{
		switch (oc)
		{
		case '?':
			cout << "Help of utility : "<<endl
			<< left <<setw(6)<< "-i : input stream name (filename or camera ID)" <<endl
			<< left <<setw(6)<< "-c : calibration result" <<endl
			<< left <<setw(6)<< "-a : augmenting content file name" <<endl
			<< left <<setw(6)<< "-b : file including points coordinates on calibration board" <<endl
			<< left <<setw(6)<< "-g : file including points coordinates to project (if specified)" <<endl
			<< left <<setw(6)<< "-r : record the calibration input (video stream and images used for calibration)" <<endl
			<< left <<setw(6)<< "-e : out put error file name" <<endl
			<< left <<setw(6)<< "-s : test suit (chessboard/acircle)" <<endl
			<< "Example: augmentation -i result1/recordedStream1.mpg -c result.yaml -b patterns/A4/A4Pattern.txt -r"<<endl
			<< endl;
			return 0;
		case 'i':
			inputStream = string(optarg);
			break;
		case 'c':
			calibrationFile = string(optarg);
			break;
		case 'b':
			boardPatternFile = string(optarg);
			break;
		case 'g':
			groundtruthfile = string(optarg);
			break;
		case 'a':
			augmentingContentFile = string(optarg);
			break;
		case 's':
			testsuite = string(optarg);
			break;
		case 'r':
			recordVideo = true;
			break;
		case 'e':
			errorFile = string(optarg);
			break;

		default :
			cout<<"Incorrect Argument!! "<<endl;
			return 0;
		}
	}

	if (testsuite.compare("chessboard") == 0)
	{
		boardPatternFile = "./patterns/chessboard/ChessBoardA4Print.txt";
		augmentingContentFile = "patterns/chessboard/ChessBoardA4Project.png";
	}
	else if (testsuite.compare("acircleA0") == 0)
	{
		boardPatternFile = "./patterns/asymCirclesA0/AcircleA0Print.txt";
		groundtruthfile = "./patterns/asymCirclesA0/AcircleA0Projected.txt";
		augmentingContentFile = "./patterns/asymCirclesA0/AcircleA0Projected.png";
	} else if (testsuite.compare("acircleA4") == 0)
	{
		boardPatternFile = "./patterns/asymCircles/AcircleA4Print.txt";
		groundtruthfile = "./patterns/asymCircles/AcircleA4Projected.txt";
		augmentingContentFile = "./patterns/asymCircles/AcircleA4Projected.png";
	}

	//Check input parameters
	if (boardPatternFile.empty())
	{
		cout<<"Input board pattern file not specified !"<<endl;
		exit(1);
	}

	cv::VideoCapture vCapture;
	char * p ;
	int camID = strtol(inputStream.c_str(), &p, 10 );

	if (*p == 0) //mloader.inputVideo is an integer
	{
		if ( !vCapture.open(camID))
		   {
			   std::cout<<"There is no input video stream from camera "<<camID<<"!"<<std::endl;
			   return EXIT_FAILURE;
		   }
		else
		{
			//reference method only accepts 640*480
//			vCapture.set(CV_CAP_PROP_FRAME_WIDTH, 10000);
//			vCapture.set(CV_CAP_PROP_FRAME_HEIGHT, 10000);
		}
	}
	else
	{
		if ( !vCapture.open(inputStream))
		   {
			   std::cout<<"There is no input video stream from file "<<inputStream<<std::endl;
			   return EXIT_FAILURE;
		   }
	}


	osgViewer::CompositeViewer viewer;

	/**
	 * Set the calibration environment
	 */
	cv::Mat img;
	vCapture >> img;
	augmentation::AugmentationEnvironment augementEnv;
	augementEnv.initialization(calibrationFile, boardPatternFile, groundtruthfile, augmentingContentFile, errorFile, recordVideo);

	cout<<"Initialization finished."<<endl;
	/**
	 * Projector
	 */
	viewer.addView( augementEnv.createProjectorView() );

	/**
	 * Camera
	 */
	viewer.addView( augementEnv.createCameraView(vCapture, img.cols, img.rows) );


	cout<<"Main loop starts..."<<endl;
	while (!viewer.done())
	{
		viewer.updateTraversal();
	    viewer.frame();
	}

	cout<<"Program is successfully terminated."<<endl;
	return 0;
}


