#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include "../build/ui_plugin.h"

#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include "/usr/share/qt4/include/Qt/qtimer.h"
#include "/usr/share/qt4/include/Qt/qpushbutton.h"
#include "/usr/share/qt4/include/Qt/qplugin.h"

#include <rw/rw.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rws/RobWorkStudio.hpp>

#include <opencv2/opencv.hpp>
#include "testMarker.hpp"
#include "testIK.hpp"

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::math;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace rws;
using namespace cv;

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::Plugin
{
	Q_OBJECT
	Q_INTERFACES( rws::RobWorkStudioPlugin )

	//	Public methods
	public:
		SamplePlugin();
		virtual ~SamplePlugin( );
		virtual void open( rw::models::WorkCell* workcell );
		virtual void close();
		virtual void initialize();

	//	Private QT methods
	private slots:
		void btnPressed();
		void timer();
		void stateChangedListener(const rw::kinematics::State& state);
		void ddMarker( QString );
		void ddSequence( QString);

	//	Private methods
	private:
		void setupQT();
		void setupVision();
		void setupHandles();
		void setupIK();
		void setupMarker();
		void setupBackground();

		void updateState();

		static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

	//	Public attributes
	public:

	//	Private attributes
	private:
		QTimer* _timer;

		rw::models::WorkCell::Ptr _wc;
		rw::kinematics::State _defaultState;
		rw::kinematics::State _state;
		rwlibs::opengl::RenderImage * _textureRender;
		rwlibs::opengl::RenderImage * _bgRender;
		rwlibs::simulation::GLFrameGrabber* _framegrabber;
		RobWorkStudio * _rsHandle = NULL;

		testMarker * temp_marker = NULL;
		testIK * temp_ik = NULL;

		Device::Ptr _deviceRobot = NULL;
		Frame* _cameraFrame = NULL;
		MovableFrame * _frameMarker = NULL;

		std::vector<std::vector<double> > data;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
