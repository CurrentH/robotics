#ifndef VISION_HPP
#define VISION_HPP

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

class Vision{

	bool sift;
	std::vector<rw::math::Vector3D<> > points;

public:
	Vision();

	std::vector<rw::math::Vector3D<> > initColor(cv::Mat image);
	std::vector<rw::math::Vector3D<> > stepColor(cv::Mat image);
	cv::Mat getCvImage();

private:
	cv::Mat imgCv;
	rw::math::Vector3D<> nearest(rw::math::Vector3D<> point, std::vector<rw::math::Vector3D<> > vect);
	std::vector<rw::math::Vector3D<> > getPoints(cv::Mat image);
	std::vector<rw::math::Vector3D<> > orderPoints(std::vector<rw::math::Vector3D<> > newPoints);
	double dist(rw::math::Vector3D<> p1, rw::math::Vector3D<> p2);

};

#endif
