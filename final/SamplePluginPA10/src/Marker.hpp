/*
 * Marker.h
 *
 *  Created on: Nov 22, 2016
 *      Author: theis
 */

#include <vector>
#include <string>

#include <rw/rw.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rws/RobWorkStudio.hpp>

#include <opencv2/opencv.hpp>

#ifndef MARKER_H_
#define MARKER_H_

class Marker {
	//	Public methods
	public:
		Marker( std::string );
		virtual ~Marker();
		Transform3D<> T1 &step();

	//	Private methods
	private:
		void loadTextFile( std::string );

	//	Public attributes
	public:

	//	Private attributes
	private:
		std::vector<std::vector<double> > stateList;
		rw::math::Transform3D<> T1;
		rw::kinematics::State state;


		int index = 0;
};

#endif /* MARKER_H_ */
