/*
 * testIK.cpp
 *
 *  Created on: Nov 22, 2016
 *      Author: theis
 */

#include "testIK.hpp"

testIK::testIK( double dT ) {
	_vision = new Vision();

	_dT = dT;

	P0 = rw::math::Vector3D<>(0.0,		0.0,	0.0);
	P1 = rw::math::Vector3D<>(0.1,		0.0,	0.0);
	P2 = rw::math::Vector3D<>(0.0,		0.1,	0.0);

	P0 = rw::math::Vector3D<>(0.15,		0.15,	0.0);
	P1 = rw::math::Vector3D<>(-0.15,	0.15,	0.0);
	P2 = rw::math::Vector3D<>(0.15,		-0.15,	0.0);

	P0 = rw::math::Vector3D<>(0.5,		0.5,	0.0);
	P1 = rw::math::Vector3D<>(-0.5,		0.5,	0.0);
	P2 = rw::math::Vector3D<>(0.5,		-0.5,	0.0);
}

testIK::~testIK() {
}

void testIK::setWorkspace( rw::models::WorkCell::Ptr wc ){
	_wc = wc;
}

void testIK::setCurrentState( rw::kinematics::State state ){
	_state = state;
}

void testIK::setDevice( rw::models::WorkCell::Ptr wc ){
	_device = wc->findDevice("PA10");
	_maxJointVelocity = _device->getVelocityLimits();
}

void testIK::setToolFrame( rw::models::WorkCell::Ptr wc ){
	_cameraFrame = (MovableFrame*) wc->findFrame("Camera");
}

void testIK::setMarkerFrame( rw::models::WorkCell::Ptr wc ){
	_markerFrame = (MovableFrame*) wc->findFrame("Marker");
}

void testIK::setFrameGrabber( rwlibs::simulation::GLFrameGrabber *frameGrabber ){
	_frameGrabber = frameGrabber;
}

void testIK::setTarget(){
	rw::math::Jacobian uv(2*numP,1);
	if( useCV ){
		std::vector<rw::math::Vector3D<> > P = _vision->stepColor( getCameraView() );
		for( unsigned int i = 0; i < numP; i++ ){
			uv(i*2,0) 	= P[i][0];
			uv(i*2+1,0) = P[i][1];
		}
	}else{
		uv = getUvPointsNoCV();
	}

	std::cout << "Target: " << std::endl;
	for( unsigned int i = 0; i < 2*numP; i++ ){
		target[i] = uv(i,0);
		std::cout << target[i] << "\t";
	}
	std::cout << std::endl;
}

void testIK::resetPose(){
	rw::math::Q pose(7, 0, -0.65, 0, 1.80, 0, 0.42, 0);
	_device->setQ( pose, _state );
}

cv::Mat testIK::getImageFrameGrabber(){
	_frameGrabber->grab( _wc->findFrame("CameraSim"), _state );
	const Image& image = _frameGrabber->getImage();
	cv::Mat img = toOpenCVImage( image );
	cv::Mat imgFlip;
	cv::flip(img, imgFlip, 0);
	return imgFlip;
}

cv::Mat testIK::getCameraView(){
	return getImageFrameGrabber();
}

cv::Mat testIK::getCvView(){
	return _vision->getCvImage();
}

cv::Mat testIK::toOpenCVImage(const Image& img) {
	cv::Mat res(img.getHeight(), img.getWidth(), CV_8UC3);
	res.data = (uchar*)img.getImageData();
	return res;
}

rw::math::Transform3D<> testIK::getMarkerToCameraTransformation(){
	return inverse(_markerFrame->fTf(_cameraFrame, _state));
}

rw::math::Q testIK::step(){
	rw::math::Jacobian uv(2*numP,2);
	if( useCV ){
		uv = getUvPointsCV();
	}else{
		uv = getUvPointsNoCV();
	}
	return getDeltaQ( uv );
}

rw::math::Jacobian testIK::getUvPointsCV(){
	std::vector<rw::math::Vector3D<> > P = _vision->stepColor( getCameraView() );

	rw::math::Jacobian uv(2*numP,1);
	for( unsigned int i = 0; i < numP; i++ ){
		uv(i*2,0) 	= P[i][0];
		uv(i*2+1,0) = P[i][1];
	}

	return uv;
}


rw::math::Jacobian testIK::getUvPointsNoCV(){
	rw::math::Transform3D<> marTcam = getMarkerToCameraTransformation();

	std::vector<rw::math::Vector3D<> > P;
	P.push_back( marTcam * P0 );
	P.push_back( marTcam * P1 );
	P.push_back( marTcam * P2 );

	rw::math::Jacobian uv(6,1);
	for( unsigned int i = 0; i < numP; i++ ){
		uv(i*2,0) 	= (P[i][0]*f)/z;
		uv(i*2+1,0) = (P[i][1]*f)/z;
	}
	return uv;
}

rw::math::Jacobian testIK::getDuDv( rw::math::Jacobian _uv ){
	rw::math::Jacobian dudv(6,1);
	for( unsigned int i = 0; i < 2*numP; i++ ){
		dudv(i,0) = target[i] - _uv(i,0);
	}
	return dudv;
}

rw::math::Jacobian testIK::getImageJacobian( rw::math::Jacobian _uv ){
	rw::math::Jacobian J_image(2*numP,6);
	for( unsigned int i = 0; i < numP; i++ ){
		double u = _uv(i*2,0);
		double v = _uv(i*2+1,0);

		J_image(2*i,0) 	= -f/z;
		J_image(2*i,1) 	= 0;
		J_image(2*i,2) 	= u/z;
		J_image(2*i,3) 	= (u*v)/f;
		J_image(2*i,4) 	= -(pow(f,2)+pow(u,2))/f;
		J_image(2*i,5) 	= v;

		J_image(2*i+1,0) 	= 0;
		J_image(2*i+1,1) 	= -f/z;
		J_image(2*i+1,2) 	= v/z;
		J_image(2*i+1,3) 	= (pow(f,2)+pow(v,2))/f;
		J_image(2*i+1,4) 	= -(u*v)/f;
		J_image(2*i+1,5) 	= -u;
	}
	return J_image;
}

rw::math::Jacobian testIK::getJ(){
	return  _device->baseJframe(_cameraFrame, _state);
}

rw::math::Jacobian testIK::getS(){
	return rw::math::Jacobian(inverse(_device->baseTframe(_cameraFrame, _state)).R());
}

rw::math::Q testIK::calculateDq( rw::math::Jacobian _ji, rw::math::Jacobian _s, rw::math::Jacobian _j, rw::math::Jacobian _dudv ){
	auto Z_image 	= (_ji * _s * _j).e();
	auto Z_image_T 	= Z_image.transpose();
	rw::math::Jacobian dq( Z_image_T * (Z_image * Z_image_T).inverse() * _dudv.e() );
	return rw::math::Q(dq.e());
}


rw::math::Q testIK::getDeltaQ( rw::math::Jacobian uv ){
	std::cout << "uv:" << std::endl << uv << std::endl;
	rw::math::Jacobian dudv = getDuDv( uv );
	std::cout << "dudv:" << std::endl << dudv << std::endl;
	rw::math::Jacobian J_image = getImageJacobian( uv );
	rw::math::Jacobian S = getS();
	rw::math::Jacobian J = getJ();
	rw::math::Q dq = calculateDq( J_image, S, J, dudv );

	if( doLogging){ logTrackingError.push_back( dudv ); }

	return _device->getQ(_state);
	return bracketJointVelocity( dq );
}

rw::math::Q testIK::bracketJointVelocity( rw::math::Q _dq ){
	rw::math::Q q = _device->getQ( _state );
	for( unsigned int i = 0; i < q.m().size(); i++ ){
		if( fabs(_dq[i]) > _maxJointVelocity[i] * _dT ){
			if( _dq[i] > 0){
				_dq[i] = _maxJointVelocity[i] * _dT;
			}else{
				_dq[i] = -_maxJointVelocity[i] * _dT;
			}
		}
		q[i] += _dq[i];
	}

    if( doLogging ){
    	logJointPosition.push_back( _dq );
    	logJointVelocity.push_back( q );
    	logToolPos.push_back( Vector3D<>(_device->baseTframe( _cameraFrame, _state ).P()) );
    	logToolRPY.push_back( RPY<>(_device->baseTframe( _cameraFrame, _state ).R()) );
    }

	return q;
}

rw::math::Jacobian testIK::transpose( rw::math::Jacobian J){
	rw::math::Jacobian JT(J.size2(), J.size1());
	for( unsigned int i = 0; i < J.size1()-1; i++ ){
		for( unsigned int j = 0; j < J.size2()-1; j++ ){
			JT(i,j) = J(j,i);
		}
	}
	return JT;
}

void testIK::finishLog(){
	std::cout << "Start logging" << std::endl;
	std::ofstream statFile;
	statFile.open(LOG_FILE_PATH);

	std::cout << logToolPos.size() << "\t";
	std::cout << logToolRPY.size() << "\t";
	std::cout << logJointPosition.size() << "\t";
	std::cout << logJointVelocity.size() << "\t";
	std::cout << logTrackingError.size() << std::endl;
	std::cout << logTrackingError[1].size1() << " " << logTrackingError[1].size2() << std::endl;

	for( unsigned int i = 0; i < logJointPosition.size(); i++ ){
			statFile << logToolPos[i] << ",";
			statFile << logToolRPY[i] << ",";
			statFile << logJointPosition[i] << ",";
			statFile << logJointVelocity[i] << ",";
			statFile << logTrackingError[i] << ",";
	}

	statFile.close();
	std::cout << "Logging done" << std::endl;
}

/*
	Eigen::MatrixXd U;
	Eigen::VectorXd SIGMA;
	Eigen::MatrixXd V;
	rw::math::LinearAlgebra::svd(J_image.e(),U,SIGMA,V);

	std::cout << "---SV---" << std::endl;
	std::cout << SIGMA << std::endl;
	std::cout << std::endl;
*/
