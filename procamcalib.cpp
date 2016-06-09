/* OpenSceneGraph example, osgcamera.
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
*  THE SOFTWARE.
*/

#include "caliblib/calibenvironment.h"
#include "boost/filesystem.hpp"
using namespace std;

/**
 * Use:
 *
 * procamcalib inputstreamname inputpatternname outputfilename
 * For example:
 * procamcalib test.webm pattern.txt result.xml
 */


int main( int argc, char **argv )
{
	bool recordVideo = false;
    string inputStream = "0";
    string outputFileDir = "latestResult.xml";
    string boardPatternFile;
    string projectionPatternFile = "";
    int numberOfView = 10;
	char oc;
	while((oc = getopt(argc, argv, "?i:o:b:p:n:r")) != -1)
	{
		switch (oc)
		{
		case '?':
			cout << "Help of utility : "<<endl
			<< left <<setw(6)<< "-i : input stream name (filename or camera ID)" <<endl
			<< left <<setw(6)<< "-o : output file name" <<endl
			<< left <<setw(6)<< "-b : file including points coordinates on calibration board" <<endl
			<< left <<setw(6)<< "-p : file including points coordinates to project (if specified)" <<endl
			<< left <<setw(6)<< "-n : number of views" <<endl
			<< left <<setw(6)<< "-r : record the calibration input (video stream and images used for calibration)" <<endl
			<< "Example: build/bin/Debug/procamcalib -i 0 -b ./patterns/A4/A4Pattern.txt -p ./patterns/A4/A4PatternProjected.txt -o test -r -n 5"<<endl
			<< endl;
			return 0;
		case 'i':
			inputStream = string(optarg);
			break;
		case 'o':
			outputFileDir = string(optarg);
			break;
		case 'b':
			boardPatternFile = string(optarg);
			break;
		case 'p':
			projectionPatternFile = string(optarg);
			break;
		case 'r':
			recordVideo = true;
			break;
		case 'n':
			numberOfView = atoi(optarg);
			break;
		default :
			cout<<"Incorrect Argument!! "<<endl;
			return 0;
		}
	}

	//Check input parameters
	if (boardPatternFile.empty())
	{
		cout<<"Input board pattern file not specified !"<<endl;
		exit(1);
	}

	//Create result directory if it does not exist, first remove the last /, if it exist
	while (outputFileDir[outputFileDir.length()-1] == '/')
	{
		outputFileDir = outputFileDir.substr(0, outputFileDir.length()-1);
		cout<<outputFileDir<<endl;
	}

	boost::filesystem::path dir(outputFileDir.c_str());
	if(boost::filesystem::create_directory(dir))
	{
	    std::cerr<< "Directory Created: "<<dir<<std::endl;
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
	CalibrationEnvironment calibenv;
	calibenv.initialization(numberOfView, boardPatternFile, projectionPatternFile, cv::Size(img.cols, img.rows), outputFileDir, recordVideo);

	cout<<"Initialization finished."<<endl;
	/**
	 * Projector
	 */
	viewer.addView( calibenv.createProjectorView() );

	/**
	 * Camera
	 */
	viewer.addView( calibenv.createCameraView(vCapture, img.cols, img.rows) );

	/**
	 * Status Bar: must be after calibenv.initialization
	 */
	viewer.addView( calibenv.createStatusView(img.cols, img.rows) );


	cout<<"Main loop starts..."<<endl;
	while ((!viewer.done())&&(!calibenv.stop()))
	{
		viewer.updateTraversal();
	    viewer.frame();
	}

	calibenv.startCalcul();

	cout<<"Program is successfully terminated."<<endl;
	return 0;
}
