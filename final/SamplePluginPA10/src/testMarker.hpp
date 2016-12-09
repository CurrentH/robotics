/*
 * testMarker.h
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

#ifndef TESTMARKER_H_
#define TESTMARKER_H_

class testMarker {
	//	Public methods
	public:
		testMarker( std::string );
		virtual ~testMarker();

		rw::math::Transform3D<> step();
		void loadMotions( std::string );
		void resetIndex();
		bool sequenceDone();

	//	Private methods
	private:


	//	Public attributes
	public:
		int index = 0;

	//	Private attributes
	private:
		std::vector<std::vector<double> > motions;

};

#endif /* TESTMARKER_H_ */
