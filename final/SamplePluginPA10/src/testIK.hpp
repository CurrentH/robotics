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
		testIK();
		virtual ~testIK();
		rw::math::Q step();

		void setCurrentState( rw::kinematics::State );
		void setDevice( rw::models::WorkCell::Ptr );
		void setToolFrame( rw::models::WorkCell::Ptr );

	//	Private methods
	private:
		rw::math::VelocityScrew6D<double> calculateDeltaU(rw::math::Transform3D<double>, rw::math::Transform3D<double>);
		rw::math::Q algorithm1(const rw::math::Transform3D<double>, const rw::math::Q);

		rw::math::Q bracketJointVelocity( rw::math::Q );
		rw::math::Jacobian getJacobian();
		void setDesiredChange();

	//	Public attributes
	public:

	//	Private attributes
	private:
		Device::Ptr _device;
		Frame* _toolFrame = NULL;
		rw::kinematics::State _state;

		rw::math::Q maxVelocity;



};

#endif /* testIK_H_ */
