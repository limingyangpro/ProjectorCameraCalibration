/*
 * calibenvironment.cpp
 *
 *  Created on: Nov 3, 2015
 *      Author: liming
 */
#include "calibenvironment.h"

#include <X11/extensions/Xinerama.h>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <opencv2/calib3d/calib3d.hpp>
#include <osgText/Font>

#include <osg/Point>
#include <osg/BlendFunc>
#include <osg/AlphaFunc>

CalibrationEnvironment::CalibrationEnvironment()
: ptsize(6),
  sizeRatio(1.0f),
stopAcquisition(false),
rawDataPtr(0),
boardTrackerPtr(0),
projection_status(NEW_PATTERN_TO_PROJECT)
{
}

CalibrationEnvironment::CalibrationEnvironment(int pointsize, int pointnumber)
: ptsize(pointsize),
  sizeRatio(1.0f),
  stopAcquisition(false),
  rawDataPtr(0),
  boardTrackerPtr(0),
  projection_status(NEW_PATTERN_TO_PROJECT)
{

}

CalibrationEnvironment::~CalibrationEnvironment()
{
	if (rawDataPtr)
	{
		delete rawDataPtr;
	}

	if (boardTrackerPtr)
	{
		delete boardTrackerPtr;
	}

	if (projectedPatternPtr)
	{
		delete projectedPatternPtr;
	}
}

bool CalibrationEnvironment::getScreenSizes()
{
	Display *d=XOpenDisplay(NULL);
	if (d) {
		int dummy1, dummy2;
		if (XineramaQueryExtension(d, &dummy1, &dummy2)) {
			if (XineramaIsActive(d)) {
				int heads=0;
				XineramaScreenInfo *p=XineramaQueryScreens(d, &heads);
				if (heads>0) {
					for (int x=0; x<heads; ++x)
					{
						cout << "Head " << x+1 << " of " << heads << ": " <<
							p[x].width << "x" << p[x].height << " at " <<
							p[x].x_org << "," << p[x].y_org << endl;
						screenRect.push_back(cv::Rect(p[x].x_org, p[x].y_org, p[x].width, p[x].height));
					}
					return true;
				} else cout << "XineramaQueryScreens says there aren't any" << endl;
				XFree(p);
			} else cout << "Xinerama not active" << endl;
		} else cout << "No Xinerama extension" << endl;
		XCloseDisplay(d);
	} else cout << "Can't open display" << endl;

	return false;
}

void CalibrationEnvironment::generateProjectedPattern()
{
	int ptnumber = sqrt(projector_height*projector_width)/8;

	srand (time(NULL));
	int i,j,rx,ry;
	unwrapedProjectedPattern.resize(ptnumber);
	projectedPatternMask.resize(ptnumber);
	for (i = 0 ; i < ptnumber ; i++)
	{
		//Generate non overlap point sets
		bool ok = false;
		projectedPatternMask[i] = false;
		while (!ok)
		{
			unwrapedProjectedPattern[i].x = rand() % projector_width;
			unwrapedProjectedPattern[i].y = rand() % projector_height;
			ok = true;
			for (j = 0 ; j < i ; j++)
			{
				if (norm(unwrapedProjectedPattern[i] - unwrapedProjectedPattern[j]) < 8*ptsize)
				{
					ok = false;
					break;
				}
			}
		}
	}
}


int CalibrationEnvironment::getProjectionStatus()
{
	return projection_status;
}

void CalibrationEnvironment::changeProjectionStatus()
{
	projection_status = 1-projection_status;
	if (projection_status == NEW_PATTERN_PROJECTED)
	{
		synchroniser.tic();
	}
}

cv::Mat CalibrationEnvironment::createProjectedImage()
{
	/**
	 * The feed back does not work very well
	 */
//	ptsize = ptsize/(sqrt(sizeRatio));
	int ptsize_int = round(ptsize);
//
//	if (ptsize_int < 1) ptsize_int = 1;
//	if (ptsize_int > 10) ptsize_int = 10;
//
//	cout<<"sizeRatio = "<<sizeRatio<<endl;
//	cout<<"ptsize = "<<ptsize<<endl;

	int rx,ry;

	cv::perspectiveTransform(unwrapedProjectedPattern, *projectedPatternPtr, boardTrackerPtr->latestPrewrapProjector);

//	int ptnumber = unwrapedProjectedPattern.size();
//	cv::Mat projectedImg = cv::Mat::zeros(projector_height,projector_width,CV_8UC3);
//	for (unsigned int i = 0 ; i < ptnumber ; i++)
//	{
//		if (!projectedPatternMask[i])
//		{
//			for (rx = unwrapedProjectedPattern[i].x - ptsize_int ; rx <= unwrapedProjectedPattern[i].x + ptsize_int ; rx++)
//				for (ry = unwrapedProjectedPattern[i].y - ptsize_int ; ry <= unwrapedProjectedPattern[i].y + ptsize_int ; ry++)
//					if ( (rx >= 0)&&(rx < projector_width)&&(ry >= 0)&&(ry < projector_height)
//							&&( (rx - unwrapedProjectedPattern[i].x)*(rx - unwrapedProjectedPattern[i].x)+(ry - unwrapedProjectedPattern[i].y)*(ry - unwrapedProjectedPattern[i].y)<=ptsize_int*ptsize_int) )
//						{
//	//						img.at<unsigned char>(projector_height - ry,rx) = 255;  //Left low corner is the orignal
//							projectedImg.at<cv::Vec3b>(ry,rx) = cv::Vec3b(255,255,255);      //Top left as origne, coherant with 09paper
//						}
//		}
//	}

	int ptnumber = projectedPatternPtr->size();
	cv::Mat projectedImg = cv::Mat::zeros(projector_height,projector_width,CV_8UC3);
	for (unsigned int i = 0 ; i < ptnumber ; i++)
	{
		for (rx = (*projectedPatternPtr)[i].x - ptsize_int ; rx <= (*projectedPatternPtr)[i].x + ptsize_int ; rx++)
			for (ry = (*projectedPatternPtr)[i].y - ptsize_int ; ry <= (*projectedPatternPtr)[i].y + ptsize_int ; ry++)
				if ( (rx >= 0)&&(rx < projector_width)&&(ry >= 0)&&(ry < projector_height)
						&&( (rx - (*projectedPatternPtr)[i].x)*(rx - (*projectedPatternPtr)[i].x)+(ry - (*projectedPatternPtr)[i].y)*(ry - (*projectedPatternPtr)[i].y)<=ptsize_int*ptsize_int) )
					{
//						img.at<unsigned char>(projector_height - ry,rx) = 255;  //Left low corner is the orignal
						projectedImg.at<cv::Vec3b>(ry,rx) = cv::Vec3b(255,255,255);      //Top left as origne, coherant with 09paper
					}
	}

	return projectedImg;
}

/**
 * Initialization of the whole working environment
 */
void CalibrationEnvironment::initialization(int numAquisition,
											const string &boardpatternfilename,
											const string &projectpatternfilename,
											const cv::Size &camSize,
											const string & outputFileDir,
											const bool &record)
{
	resultFileDir = outputFileDir;
	recordVideo = record;

//	getScreenResolution(0, projector_width, projector_height);
	getScreenSizes();
	screenID = 0;
	if (screenRect.size() >= 2)
	{
		projector_height = screenRect[1].height;
		projector_width = screenRect[1].width;
		projectorID  = 1;
	}
	else  //for single screen debug
	{
		projector_height = screenRect[0].height;
		projector_width = screenRect[0].width;
		projectorID  = 0;
	}

	cv::Size boardSize;
	cv::Size readPrjSize;

	rawDataPtr = new RawData();

	if (readPointPatternFromFile(boardpatternfilename, physicalPattern, boardSize))
	{
		rawDataPtr->setAllSizes(boardSize, camSize, cv::Size(projector_width, projector_height) );

		if (projectpatternfilename.empty())
		{
			cout<<"No projected point file !"<<endl;
			exit(1);
//			generateProjectedPattern();

//			if (record)
//			{
//				int lastindex = resultFileName.find_last_of(".");
//				string projectedPatternFileName = "projected_points.txt";
//				saveProjectedPatternToFile(projectedPatternFileName);
//			}
		}
		else
		{

			if (!readPointPatternFromFile(projectpatternfilename, unwrapedProjectedPattern, readPrjSize))
			{
				cout<<"Initialization failure!"<<endl;
				exit(1);
			}
		}

		projectedPatternMask.assign(unwrapedProjectedPattern.size(), false);
		projectedPatternPtr = new vector<cv::Point2d>(unwrapedProjectedPattern.size());
		*projectedPatternPtr = unwrapedProjectedPattern;
		rawDataPtr->setModelPatterns(physicalPattern, unwrapedProjectedPattern);

		float intialPatternRatio = projector_width/float(readPrjSize.width);
		boardTrackerPtr = new BoardTracker(intialPatternRatio);
		boardTrackerPtr->initialize(numAquisition,
				this,
				rawDataPtr,
				projectedPatternPtr,
				&projectedPatternMask);

	}
	else
	{
		cout<<"Initialization failure!"<<endl;
		exit(1);
	}
}


/**
 * Read point pattern from specific file
 * The origne of the point list in the file is at bottom left,
 * We convert it to top left
 */
bool CalibrationEnvironment::readPointPatternFromFile(const string &patternfilename, vector<cv::Point2d>& oPoints, cv::Size &sz)
{
	ifstream file(patternfilename.c_str());
	if (!file.is_open())         //Check if the corresponding file exists
	{
		std::cout<<"Fail to open file "<< patternfilename << std::endl;
		return false;
	}

	int x_min, y_min, x_max, y_max;
	file >> x_min >> y_min >> sz.width >> sz.height;
	if ((x_min != 0)||(y_min != 0))
	{
		cout<<"Warning: the min of point pattern is not zero !"<<endl;
	}

	int totalpoints;
	file >> totalpoints;

	oPoints.resize(totalpoints);
	//Always top left as origin
	for(int i=0;i<totalpoints;i++)
	{
		file >> oPoints[i].x >> oPoints[i].y;
//		oPoints[i].y = sz.height - oPoints[i].y;     //coordinates conversion
	}
	file.close();

	return true;
}

bool CalibrationEnvironment::saveProjectedPatternToFile(const string &patternfilename)
{
	fstream projectedPatternFile;
	projectedPatternFile.open(patternfilename.c_str(),std::fstream::out);
	if(!projectedPatternFile.is_open())
	{
		cout<<"Can't open output file correctly !"<<endl;
		return false;
	}
	projectedPatternFile<<"0 0 "<<projector_width<<" "<<projector_height<<endl;

	unsigned int ptnum = unwrapedProjectedPattern.size();
	projectedPatternFile<<ptnum<<endl;
	//Always top left as origin
	for (unsigned int i = 0 ; i < ptnum ; i++)
	{
		projectedPatternFile << unwrapedProjectedPattern[i].x << " " << unwrapedProjectedPattern[i].y <<endl;
//		projectedPatternFile << projectedPattern[i].x << " " << projector_height - projectedPattern[i].y <<endl;
	}

	projectedPatternFile.close();
	return true;
}


/**
 * create geometry for any list of 2D points
 * @param drawType: [Input] An integer indicating how to connect points, such as osg::PrimitiveSet::LINE_LOOP
 * @param pointVector: [Input] An vector contains 2D verteces
 * @param color: [Input] Color of these primitives
 * @return: A created OGS geometry
 */
osg::ref_ptr<osg::Geometry> CalibrationEnvironment::createPrimitiveGeometry(const int drawType, const vector<cv::Point2f>& pointVector, const osg::Vec4& color)
{
	osg::ref_ptr<osg::Geometry> pointSet (new osg::Geometry());
	osg::ref_ptr<osg::Vec3Array> vertices (new osg::Vec3Array());
	osg::ref_ptr<osg::Vec4Array> overallColor = new osg::Vec4Array;
	overallColor->push_back(color);

	for (unsigned int j = 0 ; j < pointVector.size() ; j++)
	{
		vertices->push_back (osg::Vec3 ( pointVector[j].x, pointVector[j].y, 0.0));
	}

	pointSet->setVertexArray (vertices.get());
	pointSet->addPrimitiveSet(new osg::DrawArrays(drawType,0,vertices->size()));
	pointSet->setColorArray(overallColor.get());
	pointSet->setColorBinding(osg::Geometry::BIND_OVERALL);
	pointSet->getOrCreateStateSet()->setAttribute( new osg::Point( 3.0f ), osg::StateAttribute::ON );

	return (pointSet.get());
}

/**
 * Create geodes for texture
 * @param camWidth: [Input] The width of camera image
 * @param camHeight: [Input] The height of camera image
 * @return: A texture attached node
 */
osg::ref_ptr<osg::Node> CalibrationEnvironment::createGeodes(float camWidth, float camHeight)
{
	//Vortex of the geometry
    osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array;
    v->push_back( osg::Vec3( 0, 0, -1 ) );
    v->push_back( osg::Vec3( camWidth, 0, -1 ) );
    v->push_back( osg::Vec3( camWidth, camHeight, -1 ) );
    v->push_back( osg::Vec3( 0, camHeight, -1 ) );

    //Corner of the texture
    osg::ref_ptr<osg::Vec2Array> tc = new osg::Vec2Array;
    tc->push_back( osg::Vec2( 0.f, 0.f ) );
    tc->push_back( osg::Vec2( 1.f, 0.f ) );
    tc->push_back( osg::Vec2( 1.f, 1.f ) );
    tc->push_back( osg::Vec2( 0.f, 1.f ) );

    osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;
    c->push_back( osg::Vec4( 1.f, 1.f, 1.f, 1.f ) );

    osg::ref_ptr<osg::Group> grp = new osg::Group;
    {
        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
        geom->setVertexArray( v.get() );
        geom->setTexCoordArray( 0, tc.get() );
        geom->setColorArray( c.get() );
        geom->setColorBinding( osg::Geometry::BIND_OVERALL );

        // -X panal
        GLushort indices[] = { 0, 1, 2, 3 };
        geom->addPrimitiveSet( new osg::DrawElementsUShort(
            osg::PrimitiveSet::QUADS, 4, indices ) );

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->addDrawable( geom.get() );
        grp->addChild( geode.get() );
    }

    return( grp.release() );
}


osg::ref_ptr<osgText::Text> CalibrationEnvironment::createText( const osg::Vec3& pos, const std::string& content, float size )
{
    osg::ref_ptr<osgText::Font> g_font = osgText::readFontFile("fonts/arial.ttf");
	osg::ref_ptr<osgText::Text> text = new osgText::Text;
	text->setFont( g_font.get() );
	text->setCharacterSize( size );
	text->setAxisAlignment( osgText::TextBase::XY_PLANE );
	text->setPosition( pos );
	text->setText( content );
	return text.release();
}

/**
 * Takes the slave camera as scene
 */
osg::ref_ptr<osgViewer::View> CalibrationEnvironment::createView( int x, int y, int w, int h, osg::Node* scene )
{
	cout<<"View is at : "<<x<<","<<y<<endl;
	/**Create the traits and graphic context */
	osg::ref_ptr<osg::GraphicsContext::Traits> traits =	new osg::GraphicsContext::Traits;
	traits->x = x;
	traits->y = y;
	traits->width = w;
	traits->height = h;
	traits->windowDecoration = false;
	traits->doubleBuffer = true;
	traits->samples = 4;
	traits->overrideRedirect = false;

	osg::ref_ptr<osg::GraphicsContext> gc =	osg::GraphicsContext::createGraphicsContext( traits.get() );

	osg::ref_ptr<osgViewer::View> view = new osgViewer::View;
	view->setSceneData( scene );
	view->getCamera()->setGraphicsContext(gc);
	view->getCamera()->setViewport( new osg::Viewport(0, 0, traits->width, traits->height) );
	return view.release();
}

//void CalibrationEnvironment::getScreenResolution(int screenID, unsigned int &width, unsigned int &height)
//{
//    osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
//    if (!wsi)
//    {
//        osg::notify(osg::NOTICE)<<"Error, no WindowSystemInterface available, cannot create windows."<<std::endl;
//        exit(1);
//    }
//
//    osg::GraphicsContext::ScreenIdentifier si;
//    si.readDISPLAY();
//    cout<<wsi->getNumScreens(si)<<" screens has been connected to the system."<<endl;
//
//    wsi->getScreenResolution(osg::GraphicsContext::ScreenIdentifier(screenID), width, height);
//    cout<<"The resolution of screens N."<<screenID<<" is "<<width<<"*"<<height<<"."<<endl;
//}

osg::ref_ptr<osgViewer::View> CalibrationEnvironment::createProjectorView()
{
	/**Create the background camera */
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;

    //Set camera parameters
    camera->setRenderOrder( osg::Camera::PRE_RENDER );
    camera->setReferenceFrame( osg::Camera::ABSOLUTE_RF );

    camera->setProjectionMatrixAsOrtho(0,projector_width,0, projector_height, -1 , 1);
    camera->setViewMatrixAsLookAt(
	        osg::Vec3(0.0f, 0.0f, 0.0f),
			osg::Vec3(0, 0, -1),
			osg::Vec3(0, 1, 0)
		);

    osg::ref_ptr<UpdateProjection> backgroundCallback = new UpdateProjection(this);
    camera->setUpdateCallback( backgroundCallback );

	//Create the geometry for texture 2D
    {
		osg::ref_ptr<osg::Node> textureNode = createGeodes(projector_width, projector_height);
		osg::StateSet* state = textureNode->getOrCreateStateSet();
		state->setMode( GL_LIGHTING, osg::StateAttribute::OFF |  osg::StateAttribute::PROTECTED );

		// Set the texture object but not connect with any image
		osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D;
		tex->setResizeNonPowerOfTwoHint(false);
		tex->setUnRefImageDataAfterApply( true );
//		tex->setImage( image.get() );
		state->setTextureAttributeAndModes( 0, tex.get() );

		// Turn on blending
		osg::BlendFunc* bf = new osg::BlendFunc( osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA );
		state->setAttributeAndModes( bf );

		// Turn on alpha testing
		osg::AlphaFunc* af = new osg::AlphaFunc(osg::AlphaFunc::GREATER, 0.05f );
		state->setAttributeAndModes( af );

		textureNode->setDataVariance( osg::Object::DYNAMIC );
		textureNode->setName("Projected pattern");
		camera->addChild( textureNode.get() );
    }

	osg::ref_ptr<osgViewer::View>  projectorView = createView(screenRect[projectorID].x, screenRect[projectorID].y,
													screenRect[projectorID].width, screenRect[projectorID].height, camera);
	projectorView->getCamera()->setClearMask( GL_DEPTH_BUFFER_BIT );

    return projectorView;
}

osg::ref_ptr<osgViewer::View> CalibrationEnvironment::createCameraView(cv::VideoCapture &vCapture, float camWidth, float camHeight)
{
	/**Create the background camera */
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;

    //Set camera parameters
    camera->setRenderOrder( osg::Camera::PRE_RENDER );
    camera->setReferenceFrame( osg::Camera::ABSOLUTE_RF );

    camera->setProjectionMatrixAsOrtho(0,camWidth,0, camHeight, -1 , 1);
    camera->setViewMatrixAsLookAt(
	        osg::Vec3(0.0f, 0.0f, 0.0f),
			osg::Vec3(0, 0, -1),
			osg::Vec3(0, 1, 0)
		);
    osg::ref_ptr<UpdateTexture> backgroundCallback = new UpdateTexture(vCapture, this);
    camera->setUpdateCallback( backgroundCallback );

	//Create the geometry for texture 2D
    {
		osg::ref_ptr<osg::Node> textureNode = createGeodes(camWidth, camHeight);
		osg::StateSet* state = textureNode->getOrCreateStateSet();
		state->setMode( GL_LIGHTING, osg::StateAttribute::OFF |  osg::StateAttribute::PROTECTED );

		// Set the texture object but not connect with any image
		osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D;
		tex->setResizeNonPowerOfTwoHint(false);
		tex->setUnRefImageDataAfterApply( true );
		state->setTextureAttributeAndModes( 0, tex.get() );

		// Turn on blending
		osg::BlendFunc* bf = new osg::BlendFunc( osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA );
		state->setAttributeAndModes( bf );

		// Turn on alpha testing
		osg::AlphaFunc* af = new osg::AlphaFunc(osg::AlphaFunc::GREATER, 0.05f );
		state->setAttributeAndModes( af );

		textureNode->setDataVariance( osg::Object::DYNAMIC );
		textureNode->setName("Background texture");
		camera->addChild( textureNode.get() );
    }

    //Create geometry for detected points
    {
    	osg::ref_ptr<osg::Geode> detectedPointsNode (new osg::Geode());
    	//Why the color is black?
    	detectedPointsNode->addDrawable (createPrimitiveGeometry(osg::PrimitiveSet::POINTS, vector<cv::Point2f>(), osg::Vec4(0.0,0.0,1.0,1.0)));
    	detectedPointsNode->setDataVariance( osg::Object::DYNAMIC );
    	detectedPointsNode->setName("Detected points");
    	camera->addChild( detectedPointsNode.get() );
    }

	osg::ref_ptr<osgViewer::View>  cameraView = createView(0, 0, camWidth, camHeight, camera);
	cameraView->getCamera()->setClearMask( GL_DEPTH_BUFFER_BIT );

	return cameraView;
}

osg::ref_ptr<osgViewer::View> CalibrationEnvironment::createStatusView(float camWidth, float camHeight)
{
	/**Create the background camera */
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;

    //Set camera parameters
    camera->setRenderOrder( osg::Camera::PRE_RENDER );
    camera->setReferenceFrame( osg::Camera::ABSOLUTE_RF );

    float barHeight = 70;
    camera->setProjectionMatrixAsOrtho(0, camWidth, 0, barHeight, -1 , 1);
    camera->setViewMatrixAsLookAt(
	        osg::Vec3(0.0f, 0.0f, 0.0f),
			osg::Vec3(0, 0, -1),
			osg::Vec3(0, 1, 0)
		);


    osg::ref_ptr<UpdateStatus> statusCallback = new UpdateStatus(this->boardTrackerPtr, camWidth, barHeight);
    camera->setUpdateCallback( statusCallback );

	//Create the geometry for texture 2D
    {
		osg::ref_ptr<osg::Node> textureNode = createGeodes(camWidth, barHeight);
		osg::StateSet* state = textureNode->getOrCreateStateSet();
		state->setMode( GL_LIGHTING, osg::StateAttribute::OFF |  osg::StateAttribute::PROTECTED );

		// Set the texture object but not connect with any image
		osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D;
		tex->setResizeNonPowerOfTwoHint(false);
		tex->setUnRefImageDataAfterApply( true );
//		tex->setImage( image.get() );
		state->setTextureAttributeAndModes( 0, tex.get() );

		// Turn on blending
		osg::BlendFunc* bf = new osg::BlendFunc( osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA );
		state->setAttributeAndModes( bf );

		// Turn on alpha testing
		osg::AlphaFunc* af = new osg::AlphaFunc(osg::AlphaFunc::GREATER, 0.05f );
		state->setAttributeAndModes( af );

		textureNode->setDataVariance( osg::Object::DYNAMIC );
		textureNode->setName("Status Bar");

		camera->addChild( textureNode.get() );
    }

    {
    	osg::ref_ptr<osg::Geode> textGeode = new osg::Geode;
    	textGeode->addDrawable( createText(osg::Vec3(150.0f, barHeight/3, 0.0f), "Calibrating...",	20.0f) );
//    	textGeode->addDrawable( createText(osg::Vec3(150.0f, barHeight/2, 0.0f), "Six-seat, low-wing and twin-engined", 15.0f) );
    	textGeode->setName("Status text");

		camera->addChild( textGeode.get() );
    }

    camera->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
	osg::ref_ptr<osgViewer::View>  projectorView = createView(0, 1.9*camHeight,
															camWidth, barHeight, camera);
	projectorView->getCamera()->setClearMask( GL_DEPTH_BUFFER_BIT );

    return projectorView;
}

cv::Mat CalibrationEnvironment::matchPointSet(const cv::Mat &srcImg)
{
	return boardTrackerPtr->matchPointSet(srcImg);
}

bool CalibrationEnvironment::stop()
{
	stopAcquisition = !(boardTrackerPtr->getCompleteRatio() < 1.0);
	return stopAcquisition;
}

void CalibrationEnvironment::setStopFlag(bool flag)
{
	stopAcquisition = flag;
}


void CalibrationEnvironment::startCalcul()
{
	cout<<"I'm going to start calculation ... "<<endl;
	CalibrationData* calibDataPtr = new CalibrationData(rawDataPtr);
	calibDataPtr->calibrate();
	calibDataPtr->saveParams(string(resultFileDir + "/" + resultFileDir + ".yaml"));

	if (recordVideo)
		calibDataPtr->printCalibrationInfo();

	if (calibDataPtr)
	{
		delete calibDataPtr;
	}
}

/*
 * ===========================================UpdateTexture====================================
 */
UpdateTexture::UpdateTexture(cv::VideoCapture &capture, CalibrationEnvironment* calibenv)
: calibEnvironment(calibenv),
  videoStream(capture),
  detector(new SimplePointDetector())
//  detector(new ContoursPointDetector())
{

}

UpdateTexture::~UpdateTexture()
{
	delete detector;
}

void UpdateTexture::updateBackgroundTexture(osg::Node* textureNode)
{
	cv::flip(cvImg,cvImg,0);

	osg::ref_ptr<osg::Image> image = new osg::Image();
	image->setImage(cvImg.cols,
					  cvImg.rows,
					  3,
					  GL_RGB,
					  GL_BGR,
					  GL_UNSIGNED_BYTE,
					  cvImg.data,
					  osg::Image::NO_DELETE,1);

	osg::ref_ptr<osg::StateSet> state = textureNode->getOrCreateStateSet();
	osg::ref_ptr<osg::Texture2D> tex = (osg::Texture2D*) state->getTextureAttribute(0,osg::StateAttribute::TEXTURE);
	tex->setImage( image.get() );
}

void UpdateTexture::operator()( osg::Node* node, osg::NodeVisitor* nv )
{
	   Chronometer timer;
	   timer.tic();
	   cv::Mat currentImg;

//	   if (calibEnvironment->getProjectionStatus() == CalibrationEnvironment::NEW_PATTERN_PROJECTED)
//	   {
////		   cv::waitKey(100);
//		   videoStream >> currentImg;
////		   imshow("changedPattern", currentImg);
////		   cv::waitKey(-1);
////		   calibEnvironment->changeProjectionStatus();
//	   }
//	   else
	   {
		   videoStream >> currentImg;
	   }

	   if (!currentImg.empty())
	   {
		   cvImg = calibEnvironment->matchPointSet(currentImg);
		   osg::ref_ptr<osg::Group> backgroundCameraNode = dynamic_cast<osg::Group*>(node);
		   for (unsigned int i = 0 ; i < backgroundCameraNode->getNumChildren() ; i++)
		   {
			   if (backgroundCameraNode->getChild(i)->getName() == string("Background texture"))
			   {
				   updateBackgroundTexture(backgroundCameraNode->getChild(i));
			   }
		   }

			if (calibEnvironment->recordVideo)
			{
				if (! vWriter.isOpened())
				{
					vWriter.open(string(calibEnvironment->resultFileDir + "/recordedStream.mpg"), CV_FOURCC('P','I','M','1'), 20.0f, currentImg.size(), true);
				}
				vWriter << currentImg;

				if (! vWriter2.isOpened())
				{
					vWriter2.open(string(calibEnvironment->resultFileDir + "/outStream.mpg"), CV_FOURCC('P','I','M','1'), 20.0f, currentImg.size(), true);
				}
				cv::Mat invcvImg;
				cv::flip(cvImg,invcvImg,0);
				vWriter2 << invcvImg;
			}

		   traverse( node, nv );
	   }
	   else
	   {
		   calibEnvironment->setStopFlag(true);
	   }
}

/*
 * ===========================================UpdateProjection====================================
 */

void UpdateProjection::createProjectionImage(osg::Node* textureNode)
{
	cvProjectedImg = calibEnvironment->createProjectedImage().clone();
	cv::flip(cvProjectedImg,cvProjectedImg,0);

	osg::ref_ptr<osg::Image> image = new osg::Image();
	image->setImage(cvProjectedImg.cols,
					cvProjectedImg.rows,
					  3,
					  GL_RGB,
					  GL_BGR,
					  GL_UNSIGNED_BYTE,
					  cvProjectedImg.data,
					  osg::Image::NO_DELETE,1);

	osg::ref_ptr<osg::StateSet> state = textureNode->getOrCreateStateSet();
	osg::ref_ptr<osg::Texture2D> tex = (osg::Texture2D*) state->getTextureAttribute(0,osg::StateAttribute::TEXTURE);
	tex->setImage( image.get() );
}

UpdateProjection::UpdateProjection(CalibrationEnvironment* calibenv)
: calibEnvironment(calibenv)
{
}

UpdateProjection::~UpdateProjection()
{

}

void UpdateProjection::operator()( osg::Node* node, osg::NodeVisitor* nv )
{
	//The initial state is NEW_PATTERN_TO_PROJECT, so we can get a pattern
	if (calibEnvironment->getProjectionStatus() == CalibrationEnvironment::NEW_PATTERN_TO_PROJECT)
	{
		osg::ref_ptr<osg::Group> backgroundCameraNode = dynamic_cast<osg::Group*>(node);
		for (unsigned int i = 0 ; i < backgroundCameraNode->getNumChildren() ; i++)
		{
			if (backgroundCameraNode->getChild(i)->getName() == string("Projected pattern"))
			{
				createProjectionImage(backgroundCameraNode->getChild(i));
			}
		}

		calibEnvironment->changeProjectionStatus();
	}

	traverse( node, nv );
}


/*
 * ===========================================UpdateStatus====================================
 */

void UpdateStatus::updateStatusBar(osg::Node* textureNode)
{
	statusBar = cv::Mat::zeros((int)barHeight, (int)barWidth, CV_8UC3);

	int seperateLine = (int)(barHeight*0.3) - 1;
	assert(seperateLine >= 1);
	//Tracking status
	{
		float r = boardTrackerPtr->getElapsedTime();
		if (r > 1) r = 1;

		if (r > 0)
		{
			cv::Mat roi = statusBar(cv::Rect(0,0, int(barWidth*r) , seperateLine) );
			roi.setTo(cv::Scalar(0,255,0));
		}
	}

	//Captured status
	{
		float r = boardTrackerPtr->getCompleteRatio();
		if (r > 1) r = 1;

		if (r > 0)
		{
			cv::Mat roi = statusBar(cv::Rect(0, seperateLine+1, int(barWidth*r) ,(int)barHeight - seperateLine - 1) );
			roi.setTo(cv::Scalar(0,255,0));
		}
	}


	osg::ref_ptr<osg::Image> image = new osg::Image();
	image->setImage(statusBar.cols,
					statusBar.rows,
					  3,
					  GL_RGB,
					  GL_BGR,
					  GL_UNSIGNED_BYTE,
					  statusBar.data,
					  osg::Image::NO_DELETE,1);

	osg::ref_ptr<osg::StateSet> state = textureNode->getOrCreateStateSet();
	osg::ref_ptr<osg::Texture2D> tex = (osg::Texture2D*) state->getTextureAttribute(0,osg::StateAttribute::TEXTURE);
	tex->setImage( image.get() );
}

UpdateStatus::UpdateStatus(BoardTracker* tracker, float width, float height)
: boardTrackerPtr(tracker),
  barHeight(height),
  barWidth(width)
{

}
UpdateStatus::~UpdateStatus()
{
}

void UpdateStatus::operator()( osg::Node* node, osg::NodeVisitor* nv )
{
	osg::ref_ptr<osg::Group> backgroundCameraNode = dynamic_cast<osg::Group*>(node);
	for (unsigned int i = 0 ; i < backgroundCameraNode->getNumChildren() ; i++)
    {
		if (backgroundCameraNode->getChild(i)->getName() == string("Status Bar"))
		{
			updateStatusBar(backgroundCameraNode->getChild(i));
		}
	}

	traverse( node, nv );
}


