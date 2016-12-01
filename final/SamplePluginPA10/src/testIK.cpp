/*
 * testIK.cpp
 *
 *  Created on: Nov 22, 2016
 *      Author: theis
 */

#include "testIK.hpp"

testIK::testIK() {

	maxVelocity = _device->getVelocityLimits();
}

testIK::~testIK() {
}

void testIK::setCurrentState( rw::kinematics::State state ){
	_state = state;
}

void testIK::setDevice( rw::models::WorkCell::Ptr wc ){
	_device = wc->findDevice("PA10");
}

void testIK::setToolFrame( rw::models::WorkCell::Ptr wc ){
	_toolFrame = wc->findFrame("CameraSim");
}

rw::math::Jacobian testIK::getJacobian(){
	double z = 100;
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

	/**
	 * 	Make for loop, and check if the new velocities are okay.
	 */

	return q;
}

rw::math::Q testIK::step(){
	// Get configuration, q
	rw::math::Q q_cur = _device->getQ(_state);

	// Get transformation T_base_camera
	const rw::math::Transform3D<double> baseTcamera = _device->baseTframe(_toolFrame, _state);

	// Choose a small positional change, deltaP (ca. 10^-4)
	const double delta = 0.0001;
	const rw::math::Vector3D<double> deltaP(delta, delta, delta);

	// Choose baseTtool_desired by adding the positional change deltaP to the position part of baseTtool
	const rw::math::Transform3D<double> deltaPdesired = baseTcamera.P() + deltaP;
	const rw::math::Transform3D<double> baseTcamera_desired(deltaPdesired, baseTcamera.R());

	// Apply algorithm 1
	return algorithm1( baseTcamera_desired, q_cur );
}

rw::math::VelocityScrew6D<double> testIK::calculateDeltaU(rw::math::Transform3D<double> baseTtool, rw::math::Transform3D<double> baseTtool_desired) {
    // Calculate dp
    rw::math::Vector3D<double> dp = baseTtool_desired.P() - baseTtool.P();

    // Calculate dw
    rw::math::EAA<double> dw(baseTtool_desired.R() * baseTtool.R().inverse());

    return rw::math::VelocityScrew6D<double>(dp, dw);
}

void testIK::setDesiredChange(){

}

rw::math::Q testIK::algorithm1(const rw::math::Transform3D<double> baseTtool_desired, const rw::math::Q q_in) {

	rw::math::Transform3D<double> baseTtool = _device->baseTframe(_toolFrame, _state);
    rw::math::VelocityScrew6D<double> deltaU = calculateDeltaU(baseTtool, baseTtool_desired);
    rw::math::Q q = q_in;

    const double epsilon = 0.0001;
    while(deltaU.norm2() > epsilon) {
    	rw::math::Jacobian J = _device->baseJframe(_toolFrame, _state);
        rw::math::Q deltaQ(J.e().inverse() * deltaU.e());
        q += deltaQ;
        _device->setQ(q, _state);
        baseTtool = _device->baseTframe(_toolFrame, _state);
        deltaU = calculateDeltaU(baseTtool, baseTtool_desired);
    }
    return q;
}
