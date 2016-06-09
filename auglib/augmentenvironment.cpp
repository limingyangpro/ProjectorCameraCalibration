/*
 * augmentenvironment.cpp
 *
 *  Created on: Nov 3, 2015
 *      Author: liming
 */
#include "augmentenvironment.h"

#include <X11/extensions/Xinerama.h>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <opencv2/calib3d/calib3d.hpp>
#include <osgText/Font>

#include <osg/Point>
#include <osg/BlendFunc>
#include <osg/AlphaFunc>

#include "../lgc/include/affine2D.hpp"
#include "../lgc/include/perspective2D.hpp"

namespace augmentation
{


void distort(const cv::Mat& src, cv::Mat& dst, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs)
{
	vector<cv::Point2f> pixel_locations_src(src.rows*src.cols), fractional_locations_dst;

	for (int i = 0; i < src.rows; i++) {
		for (int j = 0; j < src.cols; j++) {
			pixel_locations_src[i*src.cols + j] = cv::Point2f(j,i);
		}
	}

	cv::undistortPoints(pixel_locations_src, fractional_locations_dst, cameraMatrix, distCoeffs);

	cv::Mat pixel_locations_dst = cv::Mat(src.size(), CV_32FC2);

	const float fx = cameraMatrix.at<double>(0,0);
	const float fy = cameraMatrix.at<double>(1,1);
	const float cx = cameraMatrix.at<double>(0,2);
	const float cy = cameraMatrix.at<double>(1,2);

	// is there a faster way to do this?
	for (int i = 0; i < src.rows; i++) {
		for (int j = 0; j < src.cols; j++) {
			const float x = fractional_locations_dst[i*src.cols + j].x*fx + cx;
			const float y = fractional_locations_dst[i*src.cols + j].y*fy + cy;
			pixel_locations_dst.at<cv::Point2f>(i,j) = cv::Point2f(x,y);
		}
	}
  cv::remap(src, dst, pixel_locations_dst, cv::Mat(), CV_INTER_LINEAR);
}

AugmentationEnvironment::AugmentationEnvironment()
: matcherPtr(0),
  hasProjected(false),
  calculateErr(false),
  posFound(false)
{
	synchroniser.tic();
}

AugmentationEnvironment::AugmentationEnvironment(int pointsize, int pointnumber)
: matcherPtr(0),
  hasProjected(false),
  calculateErr(false),
  posFound(false)
{
	synchroniser.tic();
}

AugmentationEnvironment::~AugmentationEnvironment()
{
	if (matcherPtr)
	{
		delete matcherPtr;
	}
}

bool AugmentationEnvironment::getScreenSizes()
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

bool AugmentationEnvironment::readParametersFile(string parameterfile)
{
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

    cv::Mat tmpMatrix;
//    double matricCeof = 15/200.0f;    //mm/pixel : reference
    double matricCeof = 210/2100.f;   //for A4paper
//    double matricCoef = 236/3000.0f;  //their
    {
		cv::FileNode camera_node = fs["Camera  0"];
		camera_node["cameraMatrix"] >> tmpMatrix;
		tmpMatrix.convertTo(cameraMatrix, CV_64F);
//		cameraMatrix = cameraMatrix/matricCeof;     //not needed
		camera_node["distortionCoeffs"] >> tmpMatrix;
		tmpMatrix.convertTo(cameraDistorsion, CV_64F);
    }

    {
		cv::FileNode projector_node = fs["Projector  0"];
		projector_node["cameraMatrix"] >> tmpMatrix;
		tmpMatrix.convertTo(projectorMatrix, CV_64F);
//		projectorMatrix = projectorMatrix/matricCeof;  //not needed
		projector_node["distortionCoeffs"] >> tmpMatrix;
		tmpMatrix.convertTo(projectorDistorsion, CV_64F);

		projector_node["R"] >> tmpMatrix;
		tmpMatrix.convertTo(R, CV_64F);
		projector_node["T"] >> tmpMatrix;
		tmpMatrix.convertTo(t, CV_64F);
		t = matricCeof*t;
    }

    fs.release();
    return true;
}

/**
 * Initialization of the whole working environment
 */
void AugmentationEnvironment::initialization(const string &paramFilename,
		const string &boardpatternfilename,
		const string &grdtruthfilename,
		const string &augmentingContentFile,
		const string &errorFile,
		const bool &record)
{
	recordVideo = record;
	augmentingContent = cv::imread(augmentingContentFile);
	parameterFileName = paramFilename;
	errorFileNameBase = errorFile;
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

	if (readParametersFile(parameterFileName))
	{
		groundTruthFileName = grdtruthfilename;
		if (groundTruthFileName.empty())
		{
			matcherPtr = new ChessBoardMatcher(boardpatternfilename);
			calculateErr = false;
		}
		else
		{
			matcherPtr = new AcircleGridMatcher(boardpatternfilename);
			calculateErr = true;
		}

	}
	else
	{
		cout<<"Initialization failure!"<<endl;
		exit(1);
	}

}


/**
 * create geometry for any list of 2D points
 * @param drawType: [Input] An integer indicating how to connect points, such as osg::PrimitiveSet::LINE_LOOP
 * @param pointVector: [Input] An vector contains 2D verteces
 * @param color: [Input] Color of these primitives
 * @return: A created OGS geometry
 */
osg::ref_ptr<osg::Geometry> AugmentationEnvironment::createPrimitiveGeometry(const int drawType, const vector<cv::Point2f>& pointVector, const osg::Vec4& color)
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
osg::ref_ptr<osg::Node> AugmentationEnvironment::createGeodes(float camWidth, float camHeight)
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


osg::ref_ptr<osgText::Text> AugmentationEnvironment::createText( const osg::Vec3& pos, const std::string& content, float size )
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
osg::ref_ptr<osgViewer::View> AugmentationEnvironment::createView( int x, int y, int w, int h, osg::Node* scene )
{
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

osg::ref_ptr<osgViewer::View> AugmentationEnvironment::createProjectorView()
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

osg::ref_ptr<osgViewer::View> AugmentationEnvironment::createCameraView(cv::VideoCapture &vCapture, float camWidth, float camHeight)
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
    osg::ref_ptr<UpdateCameraView> backgroundCallback = new UpdateCameraView(vCapture, this);
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

//	cameraView->addEventHandler(new KeyboardEventHandler(this));
	return cameraView;
}

cv::Size AugmentationEnvironment::getProjectorSize()
{
	return cv::Size(projector_width, projector_height);
}

cv::Mat AugmentationEnvironment::match(const cv::Mat &img)
{
	cv::Mat undistortImg;
	cv::undistort(img, undistortImg, cameraMatrix, cameraDistorsion);    //remove camera distortion
	return (matcherPtr->calHomography(undistortImg, latestHomoCam));

//	return (matcherPtr->calHomography(img, latestHomoCam));
}

void AugmentationEnvironment::rematch()
{
	synchroniser.tic();
	posFound = false;
	hasProjected = false;
}

/**
 * Find 3D rotation and translation with homography (i.e. calibration boards position in the camera cooperate system )
 * @param homo: [Input] Estimated Homography
 * @param rot: [Output] Rotation result
 * @param trans: [Output] Translation result
 */
void AugmentationEnvironment::H2Rt(const cv::Mat_<double> intrisinc, const cv::Mat_<double>& homo, cv::Mat_<double>& rot, cv::Mat_<double> &trans)
{
	double coefH;

	cv::Mat_<double> invcam = intrisinc.inv();
	rot = invcam*homo;

	//scaling rotation matrix to right order = find the coefficient lost while calculating homography
	/**
	 * [Cam]*[Rot]' = [Homo]*coefH
	 *          |fx  0  u0|
	 * [Cam]' = | 0 fy  v0|
	 *          | 0  0   1|
	 *
	 *          |R11 R12 t1|
	 * [Rot]' = |R21 R22 t2|
	 *          |R31 R32 t3|
	 *
	 *           |H11 H12 H13|
	 * [Homo]' = |H21 H22 H23|
	 *           |H31 H32   1|
	 */
	coefH = 2/(cv::norm(rot.col(0)) + cv::norm(rot.col(1)));
	rot = rot * coefH;
	trans = rot.col(2).clone();

	//The their rotation vector in the rotation matrix is produced as the cross product of other two vectors
	cv::Mat tmp = rot.col(2);
	rot.col(0).cross(rot.col(1)).copyTo(tmp);

	//Add here to make rotation matrix a "real" rotation matrix in zhang's method
	/**
	 * rot = UDVt
	 * rot_real = UVt
	 */
	cv::Mat_<double> D,U,Vt;
	cv::SVD::compute(rot,D,U,Vt);
	rot = U*Vt;
}

/**
 * Find simplfied homography representation from rotation and translation
 * @param homo: [Output] Estimated Homography
 * @param rot: [Input] Rotation result
 * @param trans: [Input] Translation result
 */
void AugmentationEnvironment::Rt2H(const cv::Mat_<double> intrisinc, const cv::Mat_<double>& rot, const cv::Mat_<double> &trans, cv::Mat_<double>& homo)
{
	cv::Mat_<double> R = rot.clone();

	R.at<double>(0,2) = trans.at<double>(0,0);
	R.at<double>(1,2) = trans.at<double>(1,0);
	R.at<double>(2,2) = trans.at<double>(2,0);

	homo = (intrisinc*R)/trans.at<double>(2,0);
}

void AugmentationEnvironment::calculateProjectorHomo()
{
	assert(!cameraMatrix.empty());
	assert(!projectorMatrix.empty());

	if (!latestHomoCam.empty())
	{
		/**
		 * R_bc, t_bc express the board position in the camera frame
		 */
//		double matricCoef = 223/2600.0;     //for paper chessboard
//		double matricCoef = 680/1080.0;   //for tele
//		double matricCoef = 680/2160.0;   //for tele
		double matricCeof = 210/2100.0;  //For A4paper
		cv::Mat_<double> R_bc, t_bc;
		cv::Mat_<double> matricCoefMat = (cv::Mat_<double>(3,3) << 1/matricCeof, 0, 0, 0, 1/matricCeof, 0, 0, 0, 1);
		cv::Mat_<double> homo_matric = latestHomoCam*matricCoefMat;

		homo_matric = homo_matric/homo_matric.at<double>(2,2);
		H2Rt(cameraMatrix, homo_matric, R_bc, t_bc);

		/**
		 * Find board position in the projector frame: R_bp, t_bp
		 */
		cv::Mat_<double> R_bp = R*R_bc,
				t_bp = R*t_bc + t;

		/**
		 * Find the simplified homography in projector's view
		 */
		cv::Mat_<double> homo;
		Rt2H(projectorMatrix, R_bp, t_bp, homo);

		cv::Mat_<double> result = homo*matricCoefMat.inv();
		projectorHomo = result/result.at<double>(2,2);
	}
	else
	{
		projectorHomo.release();
	}
}

void AugmentationEnvironment::calculateReprojectionErr(string groundTruthFile, const cv::Mat &srcImg)
{
	AcircleGridMatcher *circleMatcherPtr = new AcircleGridMatcher(groundTruthFile);

	cv::Mat undistortImg;
	cv::undistort(srcImg, undistortImg, cameraMatrix, cameraDistorsion);    //remove camera distortion
	cv::Mat_<double> errs = circleMatcherPtr->findErrs(undistortImg, latestHomoCam, projectorHomo, projectorMatrix, projectorDistorsion);

//	cv::Mat_<double> errs = circleMatcherPtr->findErrs(srcImg, latestHomoCam, projectorHomo, projectorMatrix, projectorDistorsion);

	if (!errs.empty())
	{
    	int lastindex = parameterFileName.find_last_of("/");
        string dir = parameterFileName.substr(0, lastindex);
        string errFilename = dir + "/" + errorFileNameBase + ".err";

		fstream errFile;
		errFile.open(errFilename.c_str(), std::fstream::out);
		if(!errFile.is_open())
		{
			cout<<"Can't open output file correctly !"<<endl;
		}

		errFile<<"x y dx dy"<<endl;
		errFile<<errs.rows<<endl;
		for (unsigned int r = 0 ; r < errs.rows ; r++)
		{
			for (unsigned int c = 0 ; c < errs.cols ; c++)
			{
				errFile<<errs[r][c]<<" ";
			}
			errFile<<endl;
		}

//		errFile<<"Fx_pro = "<<projectorMatrix[0][0]<<endl;
//		errFile<<"Fy_pro = "<<projectorMatrix[1][1]<<endl;
		errFile.close();

		cout<<"Results written into "<<errFilename<<endl;

		exit(0);
	}
}

/*
 * ===========================================UpdateCameraView====================================
 */
UpdateCameraView::UpdateCameraView(cv::VideoCapture &capture, AugmentationEnvironment* augmentenv)
: augEnvironment(augmentenv),
  videoStream(capture),
  imageCaptured(false)
//  detector(new ContoursPointDetector())
{

}

UpdateCameraView::~UpdateCameraView()
{
}

void UpdateCameraView::updateBackgroundTexture(osg::Node* textureNode)
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



void UpdateCameraView::operator()( osg::Node* node, osg::NodeVisitor* nv )
{
	   Chronometer timer;
	   timer.tic();
	   cv::Mat currentImg;
	   videoStream >> currentImg;

	   if (!currentImg.empty())
	   {
		   if ((!augEnvironment->posFound)&&(augEnvironment->synchroniser.tac() > 200))   //wait 1s to match
		   {
			   cvImg = augEnvironment->match(currentImg);
			   augEnvironment->posFound = true;
		   }
		   else if ((augEnvironment->hasProjected)&&(augEnvironment->synchroniser.tac() > 300)&&(!imageCaptured))
		   {
			   static int count = 0;
		    	int lastindex = augEnvironment->parameterFileName.find_last_of("/");
		        string dir = augEnvironment->parameterFileName.substr(0, lastindex);
		        string errFilename = dir + "/" + augEnvironment->errorFileNameBase + "err.png";

			   imwrite(errFilename, currentImg);

			   if (augEnvironment->calculateErr)
			   {
				   double meanErr, maxErr;
				   augEnvironment->calculateReprojectionErr(augEnvironment->groundTruthFileName, currentImg);
				   cout<<meanErr<<" "<<maxErr<<endl;
			   }
			   cvImg = currentImg.clone();
			   imageCaptured = true;
		   }
		   else
		   {
			   cvImg = currentImg.clone();
		   }
		   osg::ref_ptr<osg::Group> backgroundCameraNode = dynamic_cast<osg::Group*>(node);
		   for (unsigned int i = 0 ; i < backgroundCameraNode->getNumChildren() ; i++)
		   {
			   if (backgroundCameraNode->getChild(i)->getName() == string("Background texture"))
			   {
				   updateBackgroundTexture(backgroundCameraNode->getChild(i));
			   }
		   }

			if (augEnvironment->recordVideo)
			{
				if (! vWriter.isOpened())
				{
					vWriter.open("recordedStream.mpg", CV_FOURCC('P','I','M','1'), 20.0f, currentImg.size(), true);
				}
				vWriter << currentImg;

				if (! vWriter2.isOpened())
				{
					vWriter2.open("outStream.mpg", CV_FOURCC('P','I','M','1'), 20.0f, currentImg.size(), true);
				}
				vWriter2 << cvImg;
			}

		   traverse( node, nv );
	   }
}


/*
 * ===========================================UpdateProjection====================================
 */

void UpdateProjection::createProjectionImage(osg::Node* textureNode)
{
	cvProjectedImg = augEnvironment->augmentingContent.clone();
	cv::flip(cvProjectedImg,cvProjectedImg,0);

	augEnvironment->calculateProjectorHomo();
	if ((augEnvironment->projectorHomo.empty())||(!augEnvironment->posFound))
	{
		cvProjectedImg = cv::Mat::zeros(augEnvironment->getProjectorSize(), CV_8UC3);
	}
	else
	{
		cvProjectedImg = augEnvironment->augmentingContent.clone();
		warpPerspective(cvProjectedImg, cvProjectedImg, augEnvironment->projectorHomo, augEnvironment->getProjectorSize());
//		{
//			vector< cv::Point2f > tele_ref(4), tele_img;
//			tele_ref[0] = cv::Point2f(0,0);
//			tele_ref[1] = cv::Point2f(1920,0);
//			tele_ref[2] = cv::Point2f(1920,1080);
//			tele_ref[3] = cv::Point2f(0,1080);
//			cv::perspectiveTransform(tele_ref, tele_img, homo_bp);
//			for(int i = 0 ; i < 4 ; i++)
//			{
//				cout<<tele_img[i]<<endl;
//			}
//
//			imshow("tmpimg", cvProjectedImg);
//			cv::waitKey(-1);
//		}
		cv::flip(cvProjectedImg,cvProjectedImg,0);
		distort(cvProjectedImg, cvProjectedImg, augEnvironment->projectorMatrix, augEnvironment->projectorDistorsion);
		cout<<"camHomo= "<<augEnvironment->latestHomoCam<<endl;
		cout<<"projhomo = "<<augEnvironment->projectorHomo<<endl;

		augEnvironment->hasProjected = true;
		augEnvironment->synchroniser.tic();
	}
//	imwrite("temp.png",cvProjectedImg);
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

UpdateProjection::UpdateProjection(AugmentationEnvironment* augmentenv)
: augEnvironment(augmentenv)
{

}
UpdateProjection::~UpdateProjection()
{
}

void UpdateProjection::operator()( osg::Node* node, osg::NodeVisitor* nv )
{
	if (!augEnvironment->hasProjected)
	{
		osg::ref_ptr<osg::Group> backgroundCameraNode = dynamic_cast<osg::Group*>(node);
		for (unsigned int i = 0 ; i < backgroundCameraNode->getNumChildren() ; i++)
		{
			if (backgroundCameraNode->getChild(i)->getName() == string("Projected pattern"))
			{
				createProjectionImage(backgroundCameraNode->getChild(i));
			}
		}

	}

	traverse( node, nv );
}


//bool KeyboardEventHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&)
//{
//    switch(ea.getEventType())
//    {
//        case(osgGA::GUIEventAdapter::KEYDOWN):
//        {
//            if (ea.getKey()=='n')
//            {
//            	augEnvironment->rematch();
//            }
//            break;
//        }
//        default:
//            break;
//    }
//    return false;
//}

}

