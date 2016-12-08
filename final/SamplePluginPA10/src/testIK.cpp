/*
 * testIK.cpp
 *
 *  Created on: Nov 22, 2016
 *      Author: theis
 */

#include "testIK.hpp"

testIK::testIK( double dT ) {
	_dT = dT;
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
	_toolFrame = wc->findFrame("CameraSim");
}

void testIK::resetPose(){
	rw::math::Q pose(7, 0, -0.65, 0, 1.8, 0, 0.42, 0);
	_device->setQ( pose, _state );
}

rw::math::Transform3D<> testIK::getMarkerTransformation(){
	rw::kinematics::FKRange forwardKinematicRangeMarker( _device->getBase(), _wc->findFrame("Marker"), _state );
	return forwardKinematicRangeMarker.get( _state );
}

rw::math::Transform3D<> testIK::getCameraTransformation(){
	rw::kinematics::FKRange forwardKinematicRangeCamera( _device->getBase(), _wc->findFrame("CameraSim"), _state );
	return forwardKinematicRangeCamera.get( _state );
}

rw::math::Transform3D<> testIK::getTrueMarkerPosition(){
	rw::math::Transform3D<> cameraTransformation = getCameraTransformation();
	rw::math::Transform3D<> markerTransformation = getMarkerTransformation();

	rw::math::Vector3D<> positionOffset(-0.5, 0, 0); //	We need to keep the robot 0.5m back.
	rw::math::Vector3D<> desiredPosition = markerTransformation.P() + positionOffset;
	rw::math::Rotation3D<> desiredRotation = cameraTransformation.R();
	rw::math::Transform3D<> desired_transform( desiredPosition, desiredRotation);

	temp1 = markerTransformation;
	temp2 = cameraTransformation;
	temp3 = desired_transform;

	if( doLogging ){ logToolPose.push_back( cameraTransformation ); }

	return desired_transform;
}

rw::math::Jacobian testIK::getJacobian(){
	double z = 100;	//Der står vist at denne skal være 0.5m.
	double f = 823;
	double u = 0;	//double u = vision->getChangeU();
	double v = 0; 	//double v = vision->getChangeV();

	rw::math::Jacobian jacobian(2,6);
	jacobian(0,0) = -f/z;
	jacobian(0,1) = 0;
	jacobian(0,2) = u/z;
	jacobian(0,3) = (u*v)/f;
	jacobian(0,4) = -(pow(f,2)+pow(u,2))/f;
	jacobian(0,5) = v;

	jacobian(1,0) = 0;
	jacobian(1,1) = -f/z;
	jacobian(1,2) = v/z;
	jacobian(1,3) = (pow(f,2)+pow(v,2))/f;
	jacobian(1,4) = -(u*v)/f;
	jacobian(1,5) = -u;

	return jacobian;
}

rw::math::Q testIK::bracketJointVelocity( rw::math::Q _q ){
	rw::math::Q q = _q;

	for( unsigned int i = 0; i < q.m().size(); i++ ){
		double currentJointVelocity = q[i] - _device->getQ( _state )(i);
		if( fabs(currentJointVelocity) > _maxJointVelocity[i] * _dT ){
			if( currentJointVelocity > 0){
				q[i] = _maxJointVelocity[i] * _dT;
			}else{
				q[i] = -_maxJointVelocity[i] * _dT;
			}
		}
	}

	return q;
}

rw::math::Q testIK::step(){
	// 	Get current camera position.
	rw::math::Q q_cur = _device->getQ(_state);

	//	Get desired camera position.
	rw::math::Transform3D<double> baseTcamera_desired = getTrueMarkerPosition();

	//	Apply algorithm 1
	return bracketJointVelocity( algorithm1( baseTcamera_desired, q_cur ) );
}

rw::math::VelocityScrew6D<double> testIK::calculateDeltaU(rw::math::Transform3D<double> baseTtool, rw::math::Transform3D<double> baseTtool_desired) {
    // Calculate dp
    rw::math::Vector3D<double> dp = baseTtool_desired.P() - baseTtool.P();

    // Calculate dw
    rw::math::EAA<double> dw(baseTtool_desired.R() * baseTtool.R().inverse());

    return rw::math::VelocityScrew6D<double>(dp, dw);
}

rw::math::Q testIK::algorithm1(const rw::math::Transform3D<double> baseTtool_desired, const rw::math::Q q_in) {

	rw::math::Transform3D<double> baseTtool = _device->baseTframe(_toolFrame, _state);
    rw::math::VelocityScrew6D<double> deltaU = calculateDeltaU(baseTtool, baseTtool_desired);
    rw::math::Q q = q_in;

    const double epsilon = 0.0001;
    while( deltaU.norm2() > epsilon ) {
    	rw::math::Jacobian J = _device->baseJframe(_toolFrame, _state);
        rw::math::Q deltaQ(J.e().inverse() * deltaU.e());
        q += deltaQ;
        _device->setQ(q, _state);
        baseTtool = _device->baseTframe(_toolFrame, _state);
        deltaU = calculateDeltaU(baseTtool, baseTtool_desired);
    }

    if( doLogging ){
    	logJointPosition.push_back( q_in );
    	logJointPosition.push_back( q );
    }

    return q;
}
