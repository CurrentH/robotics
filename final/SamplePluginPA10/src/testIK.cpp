/*
 * testIK.cpp
 *
 *  Created on: Nov 22, 2016
 *      Author: theis
 */

#include "testIK.hpp"

testIK::testIK( double dT ) {
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

void testIK::setInitialTrueGoal(){
	//	Set the initial goal by taking it from the marker.
}

void testIK::setInitialCvGoal(){
	//	Set the initial goal by taking it from the CV.
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

void testIK::setTarget(){
	rw::math::Jacobian uv = getUvPoints();
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

rw::math::Transform3D<> testIK::getMarkerToCameraTransformation(){
	return inverse(_markerFrame->fTf(_cameraFrame, _state));
}

rw::math::Transform3D<> testIK::getMarkerTransformation(){
	rw::kinematics::FKRange forwardKinematicRangeMarker( _device->getBase(), _markerFrame, _state );
	return forwardKinematicRangeMarker.get( _state );
}

rw::math::Transform3D<> testIK::getCameraTransformation(){
	rw::kinematics::FKRange forwardKinematicRangeCamera( _device->getBase(), _cameraFrame, _state );
	return forwardKinematicRangeCamera.get( _state );
}

rw::math::Transform3D<> testIK::getTrueMarkerPosition(){
	rw::math::Transform3D<> cameraTransformation = getCameraTransformation();
	rw::math::Transform3D<> markerTransformation = getMarkerTransformation();

	rw::math::Vector3D<> positionOffset(-0.5, 0, 0);
	rw::math::Vector3D<> desiredPosition = markerTransformation.P() + positionOffset;
	rw::math::Rotation3D<> desiredRotation = cameraTransformation.R();
	rw::math::Transform3D<> desired_transform( desiredPosition, desiredRotation);

	if( doLogging ){ logToolPose.push_back( cameraTransformation ); }

	return desired_transform;
}

rw::math::Q testIK::step(){
	/*
	// 	Get current camera position.
	rw::math::Q q_cur = _device->getQ(_state);

	//	Get desired camera position.
	//rw::math::Transform3D<double> baseTcamera_desired = getTrueMarkerPosition();

	//	Apply algorithm 1
	return bracketJointVelocity( algorithm1( baseTcamera_desired, q_cur ) );
	*/

	return getTrueDeltaQ();

}

rw::math::Jacobian testIK::getUvPoints(){
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


rw::math::Q testIK::getTrueDeltaQ(){
	rw::math::Jacobian uv = getUvPoints();
	std::cout << "uv" << std::endl << uv << std::endl;
	rw::math::Jacobian dudv = getDuDv( uv );
	std::cout << "dudv" << std::endl << dudv << std::endl;

	rw::math::Jacobian J_image = getImageJacobian( uv );
	rw::math::Jacobian S = getS();
	rw::math::Jacobian J = getJ();
	rw::math::Q dq = calculateDq( J_image, S, J, dudv );

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

rw::math::VelocityScrew6D<double> testIK::calculateDeltaU(rw::math::Transform3D<double> baseTtool, rw::math::Transform3D<double> baseTtool_desired) {
    // Calculate dp
    rw::math::Vector3D<double> dp = baseTtool_desired.P() - baseTtool.P();

    // Calculate dw
    rw::math::EAA<double> dw(baseTtool_desired.R() * baseTtool.R().inverse());

    return rw::math::VelocityScrew6D<double>(dp, dw);
}

rw::math::Q testIK::algorithm1(const rw::math::Transform3D<double> baseTtool_desired, const rw::math::Q q_in) {

	rw::math::Transform3D<double> baseTtool = _device->baseTframe(_cameraFrame, _state);
    rw::math::VelocityScrew6D<double> deltaU = calculateDeltaU(baseTtool, baseTtool_desired);
    rw::math::Q q = q_in;

    const double epsilon = 0.0001;
    while( deltaU.norm2() > epsilon ) {
    	rw::math::Jacobian J = _device->baseJframe(_cameraFrame, _state);
        rw::math::Q deltaQ(J.e().inverse() * deltaU.e());
        q += deltaQ;
        _device->setQ(q, _state);
        baseTtool = _device->baseTframe(_cameraFrame, _state);
        deltaU = calculateDeltaU(baseTtool, baseTtool_desired);
    }

    return q;
}

void testIK::finishLog(){
	std::ofstream statFile;
	statFile.open(LOG_FILE_PATH);
	//statFile.open("/home/theis/workspace/robotics/final/SamplePluginPA10/test_stat_file.csv");

	statFile << "test" << std::endl;

	statFile << logToolPose.size() << std::endl;
	statFile << logJointPosition.size() << std::endl;
	statFile << logJointVelocity.size() << std::endl;

	for( unsigned int i = 0; i < logJointPosition.size(); i++ ){
		for( unsigned int j = 0; j < logJointPosition[i].size(); j++ ){
			statFile << logJointPosition[i][j] << ",";
		}
		for( unsigned int j = 0; j < logJointVelocity[i].size(); j++ ){
			statFile << logJointVelocity[i][j] << ",";
		}

		//	todo: the rotation matrix should be outputtet as RPY instead of a 3x3.
		statFile << logToolPose[i].P() << "," << logToolPose[i].R() << ",";
		statFile << std::endl;
	}

	statFile.close();

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
