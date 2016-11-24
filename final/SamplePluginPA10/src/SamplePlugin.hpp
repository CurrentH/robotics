#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include "../build/ui_SamplePlugin.h"

#include "Marker.hpp"
#include "IK.hpp"

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

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
	Q_OBJECT
	Q_INTERFACES( rws::RobWorkStudioPlugin )

	public:
		SamplePlugin();
		virtual ~SamplePlugin();
		virtual void open( rw::models::WorkCell* workcell );
		virtual void close();
		virtual void initialize();

	private slots:
		void btnPressed();
		void timer();
		void stateChangedListener(const rw::kinematics::State& state);

	private:
		void setupQT();
		void setupVision();

		static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

		QTimer* _timer;

		rw::models::WorkCell::Ptr _wc;
		rw::kinematics::State _state;
		rw::kinematics::State _defaultState;
		rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
		rwlibs::simulation::GLFrameGrabber* _framegrabber;

		Marker *marker = NULL;
		IK *ik = NULL;

		MovableFrame* frameMarker = NULL;
		Device::Ptr deviceRobot = NULL;

		std::vector<std::vector<double> > data;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
