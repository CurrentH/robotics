/*
 * IK.cpp
 *
 *  Created on: Nov 22, 2016
 *      Author: theis
 */

#include "testIK.hpp"

testIK::testIK() {
}

testIK::~testIK() {
}

rw::math::Q testIK::step(){
	// Get configuration, q
	auto q_cur = _device->getQ(_state);

	// Get transformation T_base_camera
	const auto baseTcamera = _device->baseTframe(cameraFrame, _state);

	// Choose a small positional change, deltaP (ca. 10^-4)
	const double delta{0.0001};
	const rw::math::Vector3D<double> deltaP(delta, delta, delta);

	// Choose baseTtool_desired by adding the positional change deltaP to the position part of baseTtool
	const auto deltaPdesired = baseTcamera.P() + deltaP;
	const rw::math::Transform3D<double> baseTcamera_desired(deltaPdesired, baseTcamera.R());

	// Apply algorithm 1
	return algorithm1( _device, _state, cameraFrame, baseTcamera_desired, q_cur );
}


rw::math::VelocityScrew6D<double> testIK::calculateDeltaU(rw::math::Transform3D<double> baseTtool, rw::math::Transform3D<double> baseTtool_desired) {
    // Calculate dp
    rw::math::Vector3D<double> dp = baseTtool_desired.P() - baseTtool.P();

    // Calculate dw
    rw::math::EAA<double> dw(baseTtool_desired.R() * baseTtool.R().inverse());

    return rw::math::VelocityScrew6D<double>(dp, dw);
}

rw::math::Q testIK::algorithm1(const rw::models::Device::Ptr device, rw::kinematics::State state, const rw::kinematics::Frame* tool,
                       const rw::math::Transform3D<double> baseTtool_desired, const rw::math::Q q_in) {

    auto baseTtool = device->baseTframe(tool, state);
    auto deltaU = calculateDeltaU(baseTtool, baseTtool_desired);
    rw::math::Q q = q_in;
    const double epsilon = 0.0001;
    while(deltaU.norm2() > epsilon) {
        auto J = device->baseJframe(tool, state);
        rw::math::Q deltaQ(J.e().inverse() * deltaU.e());
        q += deltaQ;
        device->setQ(q, state);
        baseTtool = device->baseTframe(tool, state);
        deltaU = calculateDeltaU(baseTtool, baseTtool_desired);
    }
    return q;
}
