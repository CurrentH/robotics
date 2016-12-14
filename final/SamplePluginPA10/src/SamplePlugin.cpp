#include "SamplePlugin.hpp"

#include "Path.hpp"

void SamplePlugin::setupQT(){
	setupUi(this);

	_timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// COnnect stuff to UI
	connect(_btnStart    	,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btnStop    	,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btnRestart		,SIGNAL(pressed()), this, SLOT(btnPressed()) );

	connect(_ddMarker  		,SIGNAL(currentIndexChanged(QString )), this, SLOT(ddMarker(QString)) );
	connect(_ddSequence		,SIGNAL(currentIndexChanged(QString )), this, SLOT(ddSequence(QString)) );
}

void SamplePlugin::setupVision(){
	Image textureImage(300,300,Image::GRAY,Image::Depth8U);
	_textureRender = new RenderImage(textureImage);
	Image bgImage(0,0,Image::GRAY,Image::Depth8U);
	_bgRender = new RenderImage(bgImage,2.5/1000.0);
	_framegrabber = NULL;
}

void SamplePlugin::setupHandles(){
	_wc = WorkCellLoader::Factory::load(WORKCELL_XML_PATH);

	_rsHandle = getRobWorkStudio();
	_rsHandle->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);
	_rsHandle->setWorkCell(_wc);

	_state = _wc->getDefaultState();

	_frameMarker = (MovableFrame*) _wc->findFrame("Marker");
	_deviceRobot = _wc->findDevice("PA10");
}

void SamplePlugin::setupIK(){
	temp_ik = new testIK( 0.5 ); //todo: define global dT?
	temp_ik->setCurrentState( _state );
	temp_ik->setDevice( _wc );
	temp_ik->setToolFrame( _wc );
	temp_ik->setWorkspace( _wc );
	temp_ik->setMarkerFrame( _wc );
	temp_ik->setTarget();

	temp_ik->resetPose();
	_rsHandle->setState(_state);
}

void SamplePlugin::setupMarker(){
	temp_marker = new testMarker(SEQUENCE_PATH + std::string("Marker1.ppm"));
}

void SamplePlugin::setupBackground(){
	Image::Ptr image;
	image = ImageLoader::Factory::load(BACKGROUND_COLOR_PATH + std::string("color1.ppm"));
	_bgRender->setImage(*image);
	_rsHandle->updateAndRepaint();
}

SamplePlugin::SamplePlugin(): RobWorkStudioPlugin("PluginUI", QIcon(":/pa_icon.png")){
	setupQT();
	setupVision();
}

SamplePlugin::~SamplePlugin(){
	delete _textureRender;
	delete _bgRender;
}

void SamplePlugin::initialize() {
	setupHandles();
	setupIK();
	setupMarker();
	setupBackground();

	ddMarker( "Marker1.ppm" );
	ddSequence( "MarkerMotionSlow.txt" );

	_defaultState = _state;
}

void SamplePlugin::open(WorkCell* workcell)
{
	log().info() << "OPEN: " << workcell->getFilename() << "\n";

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

	if ( temp_ik != NULL ) {
		delete temp_ik;
	}

	if ( temp_marker != NULL ) {
		delete temp_marker;
	}

	_framegrabber = NULL;
	_wc = NULL;
	temp_ik = NULL;
	temp_marker = NULL;
}

void SamplePlugin::timer() {
	if( _framegrabber != NULL && !temp_marker->sequenceDone() )
	{
		_frameMarker->setTransform(temp_marker->step(), _state);
		_deviceRobot->setQ(temp_ik->step(), _state);
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




	}else if( temp_marker->sequenceDone() ){
		_timer->stop();

		temp_marker->resetIndex();
		temp_ik->resetPose();
		updateState();

		log().info() << "Sequency done - logging" << "\n";
		temp_ik->finishLog();
		log().info() << "Logging done" << "\n";

	}
	updateState();
}

void SamplePlugin::updateState(){
	temp_ik->setCurrentState(_state);
}

void SamplePlugin::stateChangedListener(const State& state) {
	_state = state;
}

Mat SamplePlugin::toOpenCVImage(const Image& img) {
	Mat res(img.getHeight(), img.getWidth(), CV_8SC3);
	res.data = (uchar*)img.getImageData();
	return res;
}

void SamplePlugin::btnPressed() {
	QObject *obj = sender();
	if(obj==_btnStart){
		log().info() << "Start\n";
		if(!_timer->isActive()){
			_timer->start(100);
			log().info() << "\t - Timer on\n";
		}
	}
	else if(obj==_btnStop){
		// Toggle the timer on and off
		log().info() << "Stop\n";
		if (_timer->isActive()){
			_timer->stop();
			log().info() << "\t - Timer off\n";
		}
	}
	else if(obj==_btnRestart){
		log().info() << "Restart\n";
		temp_marker->resetIndex();
		temp_ik->resetPose();
		updateState();
	}
}

void SamplePlugin::ddMarker( QString file ){
	std::string path = MARKER_PATH;
	std::string _file = file.toUtf8().constData();
	std::string temp = std::string(path) + std::string(_file);

	Image::Ptr imageMarker = ImageLoader::Factory::load(temp);
	_textureRender->setImage(*imageMarker);
	_rsHandle->updateAndRepaint();

	log().info() << "Marker: " << _file << "\n";
}

void SamplePlugin::ddSequence( QString file ){
	std::string path = SEQUENCE_PATH;
	std::string _file = file.toUtf8().constData();
	std::string temp = std::string(path) + std::string(_file);

	temp_marker->loadMotions(temp);
	temp_ik->resetPose();
	updateState();
}


Q_EXPORT_PLUGIN(SamplePlugin);






