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

class IK {
	//	Public methods
	public:
		IK();
		virtual ~IK();
		rw::kinematics::State &step();

	//	Private methods
	private:


	//	Public attributes
	public:

	//	Private attributes
	private:
		rw::kinematics::State state;



};

#endif /* IK_H_ */
