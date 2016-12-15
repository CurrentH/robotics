#include "vision.hpp"

#define RED_LOW_MIN  cv::Scalar(  0,  50,  50)
#define RED_LOW_MAX  cv::Scalar( 10, 255, 255)
#define RED_HIGH_MIN cv::Scalar(160, 100, 100)
#define RED_HIGH_MAX cv::Scalar(180, 255, 255)

#define BLUE_MIN cv::Scalar(100, 120, 50)
#define BLUE_MAX cv::Scalar(120, 160, 100)

#define BLACK_MIN cv::Scalar(70, 40, 40)
#define BLACK_MAX cv::Scalar(100, 110, 110)

#define WHITE_MIN cv::Scalar(0, 0, 120)
#define WHITE_MAX cv::Scalar(180, 50, 255)

template<class T>
std::vector<T> operator+(const std::vector<T> &vec1, const std::vector<T> &vec2){
	std::vector<T> result = vec1;
	result.insert(result.end(), vec2.begin(), vec2.end());
	return result;
}

std::vector<cv::Vec3f> getCircles(const cv::Mat &source, const cv::Scalar &min, const cv::Scalar &max){
	cv::Mat threshold;
	cv::inRange(source, min, max, threshold);
	cv::GaussianBlur(threshold, threshold, cv::Size(9,9), 2, 2);
	std::vector<cv::Vec3f> circles;
	cv::HoughCircles(threshold, circles, CV_HOUGH_GRADIENT, 1, threshold.rows / 8, 100, 20, 50, 65);
	return circles;
}

void circleDetection(const cv::Mat &source, std::vector<cv::Vec3f> &circles){

	// Convert to hsv color
	cv::Mat hsv;
	cv::cvtColor(source, hsv, cv::COLOR_BGR2HSV);

	// Extract circles from image
	circles = getCircles(hsv, RED_LOW_MIN, RED_LOW_MAX) + getCircles(hsv, RED_HIGH_MIN, RED_HIGH_MAX) + getCircles(hsv, BLUE_MIN, BLUE_MAX);
}

Vision::Vision(){

}

cv::Mat Vision::getCvImage(){
	return imgCv;
}

void Vision::initColor(cv::Mat image){
	points = stepColor(image);
}

rw::math::Vector3D<> Vision::nearest(rw::math::Vector3D<> point, std::vector<rw::math::Vector3D<> > vect){
	int min = dist(point, vect[0]);
	int index = 0;
	for (int i = 1; i < 3; i++){
		if (min > dist(point, vect[i])){
			min = dist(point, vect[i]);
			index = i;
		}
	}
	return vect[i];
}

std::vector<rw::math::Vector3D<> > Vision::orderPoints(std::vector<rw::math::Vector3D<> > newPoints){
	std::vector<rw::math::Vector3D<> > result;
	for (int i = 0; i < points.size(); i++){
		result.push_back(nearest(point[i], newPoints));
	}
	return result;
}

std::vector<rw::math::Vector3D<> > Vision::stepColor(cv::Mat image){
	imgCv = image.clone();
	std::vector<cv::Vec3f> circles;
	circleDetection(imgCv, circles);
	std::vector<rw::math::Vector3D<> > result;
	for (int i = 0; i < circles.size(); i++){
		result.push_back(rw::math::Vector3D<>(circles[i][0], circles[i][1], circles[i][2]));
		cv::Point center(std::round(circles[i][0]), std::round(circles[i][1]));
		int radius = std::round(circles[i][2]);
		cv::circle(imgCv, center, radius, cv::Scalar(0, 0, 0), 20);
	}

	return orderPoints();
}
