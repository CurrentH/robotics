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

#ifndef testIK_H_
#define testIK_H_

class testIK {
	//	Public methods
	public:
		testIK();
		virtual ~testIK();
		rw::math::Q step();

	//	Private methods
	private:


	//	Public attributes
	public:

	//	Private attributes
	private:



};

#endif /* testIK_H_ */
