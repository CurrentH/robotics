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

	P0 = rw::math::Vector3D<>(0.15,		0.15,	0.0);
	P1 = rw::math::Vector3D<>(-0.15,	0.15,	0.0);
	P2 = rw::math::Vector3D<>(0.15,		-0.15,	0.0);

	P0 = rw::math::Vector3D<>(0.5,		0.5,	0.0);
	P1 = rw::math::Vector3D<>(-0.5,		0.5,	0.0);
	P2 = rw::math::Vector3D<>(0.5,		-0.5,	0.0);

	P0 = rw::math::Vector3D<>(0.0,		0.0,	0.0);
	P1 = rw::math::Vector3D<>(0.1,		0.0,	0.0);
	P2 = rw::math::Vector3D<>(0.0,		0.1,	0.0);
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
		std::vector<rw::math::Vector3D<> > P = _vision->initColor( getCameraView() );
		for( unsigned int i = 0; i < numP; i++ ){
			uv(i*2,0) 	= P[i][0];
			uv(i*2+1,0) = P[i][1];
		}
	}else{
		uv = getUvPointsNoCV();
	}
	for( unsigned int i = 0; i < 2*numP; i++ ){
		target[i] = uv(i,0);
	}
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
	if( useCV )	return _vision->getCvImage();
	return getImageFrameGrabber();
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
	rw::math::Jacobian dudv = getDuDv( uv );
	rw::math::Jacobian J_image = getImageJacobian( uv );
	rw::math::Jacobian S = getS();
	rw::math::Jacobian J = getJ();
	rw::math::Q dq = calculateDq( J_image, S, J, dudv );

	if( doLogging){ logTrackingError.push_back( dudv ); }

	//return _device->getQ(_state);
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

double testIK::calculateMaxError( std::vector<rw::math::Jacobian> _eList ){
	double dist;
	double tdist;
	for( unsigned int i = 0; i < _eList.size(); i++ ){
		for (int j = 0; j < numP; j++) {
			dist = sqrt( pow( _eList[i](i*2,0), 2 ) + pow( _eList[i](i*2+1,0), 2 ) );
			tdist += abs(dist);
		}
	}
	return tdist;
}

void testIK::finishLog(){
	std::cout << "Start logging" << std::endl;
	std::ofstream posFile, rpyFile, jposFile, jrpyFile, jvelFile, errorFile;

	std::string seq = "fast_" + std::to_string(numP) + "_";
	std::string del = std::to_string(_dT);
	std::string end = ".csv";

	posFile.open( LOG_FILE_PATH + seq + std::string("pos_") + del + end );
	for( unsigned int i = 0; i < logToolPos.size(); i++ ){
		for( unsigned int j = 0; j < 3; j++ ){
			posFile << logToolPos[j] << ",";
		}
		posFile << "\n";
	}
	posFile.close();
	std::cout << "1";

	rpyFile.open( LOG_FILE_PATH + seq + std::string("rpy_") + del + end );
	for( unsigned int i = 0; i < logToolRPY.size(); i++ ){
		for( unsigned int j = 0; j < 3; j++ ){
			rpyFile << logToolRPY[j] << ",";
		}
		rpyFile << "\n";
	}
	rpyFile.close();
	std::cout << "2";

	jposFile.open( LOG_FILE_PATH + seq + std::string("jpos_") + del + end );
	for( unsigned int i = 0; i < logJointPosition.size(); i++ ){
		for( unsigned int j = 0; j < 7; j++ ){
			jposFile << logJointPosition[i][j] << ",";
		}
		jposFile << "\n";
	}
	jposFile.close();
	std::cout << "3";

	jvelFile.open( LOG_FILE_PATH + seq + std::string("jvel_") + del + end );
	for( unsigned int i = 0; i < logJointVelocity.size(); i++ ){
		for( unsigned int j = 0; j < 7; j++ ){
			jvelFile << logJointVelocity[i][j] << ",";
		}
		jvelFile << "\n";
	}
	jvelFile.close();
	std::cout << "4";

	errorFile.open( LOG_FILE_PATH + seq + std::string("error") + end , std::ios_base::app);
	errorFile << calculateMaxError(logTrackingError) << "\n";
	errorFile.close();
	std::cout << "5 - " << _dT << std::endl;;

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
