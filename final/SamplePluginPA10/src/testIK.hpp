/*
 * testIK.h
 *
 *  Created on: Nov 22, 2016
 *      Author: theis
 */

#include <vector>
#include <string>
#include <fstream>
#include <iostream>

#include <rw/rw.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rws/RobWorkStudio.hpp>

#include <opencv2/opencv.hpp>

#include "Path.hpp"
#include "vision.hpp"

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

#ifndef testIK_H_
#define testIK_H_

class testIK {
	//	Public methods
	public:
		testIK( double );
		virtual ~testIK();
		rw::math::Q step();

		cv::Mat getCameraView();
		cv::Mat getCvView();

		void setCurrentState( rw::kinematics::State );
		void setDevice( rw::models::WorkCell::Ptr );
		void setToolFrame( rw::models::WorkCell::Ptr );
		void setWorkspace( rw::models::WorkCell::Ptr );
		void setMarkerFrame( rw::models::WorkCell::Ptr );
		void setFrameGrabber( rwlibs::simulation::GLFrameGrabber* );
		void setTarget();

		void finishLog();
		void resetPose();

	//	Private methods
	private:
		cv::Mat getImageFrameGrabber();
		static cv::Mat toOpenCVImage(const rw::sensor::Image&);

		rw::math::Transform3D<> getMarkerToCameraTransformation();

		rw::math::Jacobian getUvPointsCV();
		rw::math::Jacobian getUvPointsNoCV();
		rw::math::Jacobian getDuDv( rw::math::Jacobian );
		rw::math::Jacobian getImageJacobian( rw::math::Jacobian );
		rw::math::Jacobian getJ();
		rw::math::Jacobian getS();
		rw::math::Q calculateDq( rw::math::Jacobian, rw::math::Jacobian, rw::math::Jacobian, rw::math::Jacobian );


		rw::math::Q bracketJointVelocity( rw::math::Q );
		rw::math::Q getDeltaQ( rw::math::Jacobian );

		double calculateMaxError( std::vector<rw::math::Jacobian>  );
		rw::math::Jacobian transpose( rw::math::Jacobian );

	//	Public attributes
	public:
		double _dT;
		unsigned int numP = 3;

	//	Private attributes
	private:
		bool useCV = false;
		bool initialRun = true;
		bool doLogging = true;

		Vision* _vision = NULL;

		Device::Ptr _device;
		MovableFrame* _cameraFrame = NULL;
		MovableFrame* _markerFrame = NULL;
		rwlibs::simulation::GLFrameGrabber* _frameGrabber = NULL;

		rw::kinematics::State _state;
		rw::models::WorkCell::Ptr _wc;

		rw::math::Q _maxJointVelocity;

		rw::math::Vector3D<> P0;
		rw::math::Vector3D<> P1;
		rw::math::Vector3D<> P2;

		std::vector<double> target{0,0,0,0,0,0};


		double z = 0.5;
		double f = 823;


		std::vector<rw::math::Vector3D<> > logToolPos;
		std::vector<rw::math::RPY<> > logToolRPY;
		std::vector<rw::math::Q > logJointPosition;
		std::vector<rw::math::Q > logJointVelocity;
		std::vector<rw::math::Jacobian> logTrackingError;

};

#endif /* testIK_H_ */
