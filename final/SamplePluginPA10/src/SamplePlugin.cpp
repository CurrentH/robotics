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
	RobWorkStudio * _rsHandle = getRobWorkStudio();
}

SamplePlugin::~SamplePlugin()
{
	delete _textureRender;
	delete _bgRender;
}

void SamplePlugin::initialize() {
	_rsHandle->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);
	WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/theis/workspace/robotics/PA10WorkCell/ScenePA10RoVi1.wc.xml");
	_rsHandle->setWorkCell(wc);

	temp_marker = new testMarker("/home/theis/workspace/robotics/final/SamplePluginPA10/motions/MarkerMotionSlow.txt");
	temp_ik = new testIK();

	_frameMarker = (MovableFrame*) wc->findFrame("Marker");
	_deviceRobot = wc->findDevice("PA10");
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
			_rsHandle->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
		}

		// Add the background render to this workcell if there is a frame for texture
		Frame* bgFrame = _wc->findFrame("Background");
		if (bgFrame != NULL) {
			_rsHandle->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
		}

		// Create a GLFrameGrabber if there is a camera frame with a Camera property set
		_cameraFrame = _wc->findFrame("CameraSim");
		if (_cameraFrame != NULL) {
			if(_cameraFrame->getPropertyMap().has("Camera")) {
				// Read the dimensions and field of view
				double fovy;
				int width,height;
				std::string camParam = _cameraFrame->getPropertyMap().get<std::string>("Camera");
				//TODO:
				//std::istringstream iss (camParam, std::istringstream::in);
				std::istringstream iss (camParam);
				iss >> fovy >> width >> height;
				// Create a frame grabber
				_framegrabber = new GLFrameGrabber(width,height,fovy);
				SceneViewer::Ptr gldrawer = _rsHandle->getView()->getSceneViewer();
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
		_rsHandle->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
	}

	// Remove the background render
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		_rsHandle->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
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

		Image::Ptr image;
		image = ImageLoader::Factory::load("/home/theis/workspace/robotics/final/SamplePluginPA10/markers/Marker1.ppm");
		_textureRender->setImage(*image);

		image = ImageLoader::Factory::load("/home/theis/workspace/robotics/final/SamplePluginPA10/backgrounds/color1.ppm");
		_bgRender->setImage(*image);
		_rsHandle->updateAndRepaint();

	}
	else if(obj==_btn1){
		// Toggle the timer on and off
		log().info() << "Button 1 - ";
		if (!_timer->isActive()){
			_timer->start(100); // run 10 Hz
			log().info() << "Timer on\n";
		}
		else{
			_timer->stop();
			log().info() << "Timer off\n";
		}

	}
}

void SamplePlugin::timer() {
	if(_framegrabber != NULL )
	{
		_frameMarker->setTransform( temp_marker->step(), _state );
		_deviceRobot->setQ( temp_ik->step(), _state );
		_rsHandle->setState(_state);

		// Get the image as a RW image
		_framegrabber->grab(_cameraFrame, _state);
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






