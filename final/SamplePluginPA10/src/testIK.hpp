/*
 * IK.h
 *
 *  Created on: Nov 22, 2016
 *      Author: theis
 */

//#include "computerVision.hpp"

#include <rw/rw.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rws/RobWorkStudio.hpp>

#include <opencv2/opencv.hpp>

#ifndef IK_H_
#define IK_H_

class testIK {
	//	Public methods
	public:
		testIK();
		virtual ~testIK();
		rw::math::Q step();

	//	Private methods
	private:
		rw::math::VelocityScrew6D<double> calculateDeltaU(rw::math::Transform3D<double>, rw::math::Transform3D<double>);
		rw::math::Q algorithm1(const rw::models::Device::Ptr, rw::kinematics::State, const rw::kinematics::Frame*, const rw::math::Transform3D<double>, const rw::math::Q);

	//	Public attributes
	public:

	//	Private attributes
	private:
		rw::kinematics::State state;



};

#endif /* IK_H_ */
