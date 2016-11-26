#include "SamplePlugin.hpp"

void SamplePlugin::setupQT(){
	setupUi(this);

	_timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
}

void SamplePlugin::setupVision(){
	Image textureImage(300,300,Image::GRAY,Image::Depth8U);
	_textureRender = new RenderImage(textureImage);
	Image bgImage(0,0,Image::GRAY,Image::Depth8U);
	_bgRender = new RenderImage(bgImage,2.5/1000.0);
	_framegrabber = NULL;
}

SamplePlugin::SamplePlugin(): RobWorkStudioPlugin("PluginUI", QIcon(":/pa_icon.png"))
{
	setupQT();
	setupVision();
}

SamplePlugin::~SamplePlugin(){
	delete _textureRender;
	delete _bgRender;
}

void SamplePlugin::initialize() {
	log().info() << "INITALIZE - START\n";

	getRobWorkStudio()->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);
	WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/theis/workspace/robotics/PA10WorkCell/ScenePA10RoVi1.wc.xml");
	getRobWorkStudio()->setWorkCell(wc);

	std::string filename = "/home/theis/workspace/robotics/final/SamplePluginPA10/motions/MarkerMotionSlow.txt";
	temp_marker = new testMarker( filename );

	frameMarker = (MovableFrame*) wc->findFrame("Marker");
	if (frameMarker == NULL){
		log().info() << "Device: Marker" << " not found!\n";
	}
	deviceRobot = wc->findDevice("PA10");
	if (deviceRobot == NULL){
		log().info() << "Device: PA10" << " not found!\n";
	}
	log().info() << "INITALIZE - DONE\n";
}

void SamplePlugin::open(WorkCell* workcell)
{
	log().info() << "OPEN" << "\n";
	_wc = workcell;
	_state = _wc->getDefaultState();
	_defaultState = _state;

	log().info() << workcell->getFilename() << "\n";

	if (_wc != NULL) {
		// Add the texture render to this workcell if there is a frame for texture
		Frame* textureFrame = _wc->findFrame("MarkerTexture");
		if (textureFrame != NULL) {
			getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
		}

		// Add the background render to this workcell if there is a frame for texture
		Frame* bgFrame = _wc->findFrame("Background");
		if (bgFrame != NULL) {
			getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
		}

		// Create a GLFrameGrabber if there is a camera frame with a Camera property set
		Frame* cameraFrame = _wc->findFrame("CameraSim");
		if (cameraFrame != NULL) {
			if(cameraFrame->getPropertyMap().has("Camera")) {
				// Read the dimensions and field of view
				double fovy;
				int width,height;
				std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
				//TODO:
				//std::istringstream iss (camParam, std::istringstream::in);
				std::istringstream iss (camParam);
				iss >> fovy >> width >> height;
				// Create a frame grabber
				_framegrabber = new GLFrameGrabber(width,height,fovy);
				SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
				_framegrabber->init(gldrawer);
			}
		}
	}
}

void SamplePlugin::close() {
	log().info() << "CLOSE\n";

	// Stop the timer
	_timer->stop();

	// Remove the texture render
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
	}

	// Remove the background render
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
	}

	// Delete the old framegrabber
	if (_framegrabber != NULL) {
		delete _framegrabber;
	}

	_framegrabber = NULL;
	_wc = NULL;
}

Mat SamplePlugin::toOpenCVImage(const Image& img) {
	Mat res(img.getHeight(), img.getWidth(), CV_8SC3);
	res.data = (uchar*)img.getImageData();
	return res;
}

void SamplePlugin::btnPressed() {
	QObject *obj = sender();
	if(obj==_btn0){
		log().info() << "Button 0\n";
		// Set a new texture (one pixel = 1 mm)
		Image::Ptr image;
		image = ImageLoader::Factory::load("/home/theis/workspace/robotics/final/SamplePluginPA10/markers/Marker1.ppm");
		_textureRender->setImage(*image);
		image = ImageLoader::Factory::load("/home/theis/workspace/robotics/final/SamplePluginPA10/backgrounds/color1.ppm");
		_bgRender->setImage(*image);
		getRobWorkStudio()->updateAndRepaint();
	} else if(obj==_btn1){
		log().info() << "Button 1\n";
		// Toggle the timer on and off
		if (!_timer->isActive())
		    _timer->start(100); // run 10 Hz
		else
			_timer->stop();
	}
}

void SamplePlugin::timer() {
	if(_framegrabber != NULL ){
		/**
		 * 		Do a test if the output from the marker->step is the same as getQ.
		 * 		We want to use these to update the states for the device.
		 */
		//Q temp = deviceRobot->getQ(_state);
		//deviceRobot->setQ( temp,_state );
		//Vector3D<> temp_vector = temp_marker.P();
		//Rotation3D<> temp_rotation = temp_marker.R();

		Transform3D<> markerPR = frameMarker->getTransform(_state);
		frameMarker->setTransform( temp_marker->step(), _state );
		log().info() << temp_marker->index << "\t";
		log().info() << markerPR << "\n\n";

		getRobWorkStudio()->setState(_state);

		// Get the image as a RW image
		Frame* cameraFrame = _wc->findFrame("CameraSim");
		_framegrabber->grab(cameraFrame, _state);
		const Image& image = _framegrabber->getImage();

		// Convert to OpenCV image
		Mat im = toOpenCVImage(image);
		Mat imflip;
		cv::flip(im, imflip, 0);

		// Show in QLabel
		QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
		QPixmap p = QPixmap::fromImage(img);
		unsigned int maxW = 400/2;
		unsigned int maxH = 800/2;
		_cameraView->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
		_cvView->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
	}
}

void SamplePlugin::stateChangedListener(const State& state) {
	_state = state;
}

Q_EXPORT_PLUGIN(SamplePlugin);






