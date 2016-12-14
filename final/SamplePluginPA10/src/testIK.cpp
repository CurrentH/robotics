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
	_toolFrame = wc->findFrame("Camera");
}

void testIK::setMarkerFrame( rw::models::WorkCell::Ptr wc ){
	_markerFrame = (MovableFrame*) _wc->findFrame("Marker");
}

void testIK::resetPose(){
	rw::math::Q pose(7, 0, -0.65, 0, 1.80, 0, 0.42, 0);
	_device->setQ( pose, _state );
	initialRun = true; //todo: I think this should be reset every run
}

rw::math::Transform3D<> testIK::getMarkerTransformation(){
	rw::kinematics::FKRange forwardKinematicRangeMarker( _device->getBase(), _markerFrame, _state );
	return forwardKinematicRangeMarker.get( _state );
}

rw::math::Transform3D<> testIK::getCameraTransformation(){
	rw::kinematics::FKRange forwardKinematicRangeCamera( _device->getBase(), _toolFrame, _state );
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

rw::math::Jacobian testIK::transpose( rw::math::Jacobian J){
	rw::math::Jacobian JT(J.size2(), J.size1());
	for( unsigned int i = 0; i < J.size1()-1; i++ ){
		for( unsigned int j = 0; j < J.size2()-1; j++ ){
			JT(i,j) = J(j,i);
		}
	}
	return JT;
}

rw::math::Q testIK::getTrueDeltaQ(){
	MovableFrame* _WorldFrame = (MovableFrame*) _wc->findFrame("WORLD");
	MovableFrame* _CameraFrame = (MovableFrame*) _wc->findFrame("CameraSim");
	MovableFrame* _ToolFrame = (MovableFrame*) _wc->findFrame("Tool");
	MovableFrame* _MarkerFrame = (MovableFrame*) _wc->findFrame("Marker");

	rw::math::Transform3D<> marTcam = inverse(_MarkerFrame->fTf(_CameraFrame, _state));

	std::vector<rw::math::Vector3D<> > P;
	P.push_back( marTcam * P0 );
	P.push_back( marTcam * P1 );
	P.push_back( marTcam * P2 );

	double x0 =  P[0][0];
	double y0 =  P[0][1];
	double x1 =  P[1][0];
	double y1 =  P[1][1];
	double x2 =  P[2][0];
	double y2 =  P[2][1];

	float u0 = (x0*f)/z;
	float v0 = (y0*f)/z;
	float u1 = (x1*f)/z;
	float v1 = (y1*f)/z;
	float u2 = (x2*f)/z;
	float v2 = (y2*f)/z;

	if( initialRun ){
		setInitialTrueGoal();
			goal[0] = u0;
			goal[1] = v0;
			goal[2] = u1;
			goal[3] = v1;
			goal[4] = u2;
			goal[5] = v2;
		/*
		for( unsigned int i = 0; i < numP; i++ ){
			goal[i*2] 	= 0 - (P[P.size()-1][0] * f) / z;
			goal[i*2+1]	= 0 - (P[P.size()-1][1] * f) / z;
		}*/
		initialRun = false;
	}

	std::cout << "uv:" << std::endl;
	std::cout << u0 << "\t" << u1 << "\t" << u2 << std::endl;
	std::cout << v0 << "\t" << v1 << "\t" << v2 << std::endl;
	std::cout << std::endl;

	rw::math::Jacobian dudv(6,1);
	dudv(0,0) = goal[0] - u0;
	dudv(1,0) = goal[1] - v0;
	dudv(2,0) = goal[2] - u1;
	dudv(3,0) = goal[3] - v1;
	dudv(4,0) = goal[4] - u2;
	dudv(5,0) = goal[5] - v2;

	std::cout << "dudv:" << std::endl;
	std::cout << dudv(0,0) << "\t" << dudv(2,0) << "\t" << dudv(4,0) << std::endl;
	std::cout << dudv(1,0) << "\t" << dudv(3,0) << "\t" << dudv(5,0) << std::endl;
	std::cout << std::endl;

	rw::math::Jacobian J_image(6,6);
	J_image(0,0) 	= -f/z;
	J_image(0,1) 	= 0;
	J_image(0,2) 	= u0/z;
	J_image(0,3) 	= (u0*v0)/f;
	J_image(0,4) 	= -(pow(f,2)+pow(u0,2))/f;
	J_image(0,5) 	= v0;
	J_image(1,0) 	= 0;
	J_image(1,1) 	= -f/z;
	J_image(1,2) 	= v0/z;
	J_image(1,3) 	= (pow(f,2)+pow(v0,2))/f;
	J_image(1,4) 	= -(u0*v0)/f;
	J_image(1,5) 	= -u0;

	J_image(2,0) 	= -f/z;
	J_image(2,1) 	= 0;
	J_image(2,2) 	= u1/z;
	J_image(2,3) 	= (u1*v1)/f;
	J_image(2,4) 	= -(pow(f,2)+pow(u1,2))/f;
	J_image(2,5) 	= v1;
	J_image(3,0) 	= 0;
	J_image(3,1) 	= -f/z;
	J_image(3,2) 	= v1/z;
	J_image(3,3) 	= (pow(f,2)+pow(v1,2))/f;
	J_image(3,4) 	= -(u1*v1)/f;
	J_image(3,5) 	= -u1;

	J_image(4,0) 	= -f/z;
	J_image(4,1) 	= 0;
	J_image(4,2) 	= u2/z;
	J_image(4,3) 	= (u2*v2)/f;
	J_image(4,4) 	= -(pow(f,2)+pow(u2,2))/f;
	J_image(4,5) 	= v2;
	J_image(5,0) 	= 0;
	J_image(5,1) 	= -f/z;
	J_image(5,2) 	= v2/z;
	J_image(5,3) 	= (pow(f,2)+pow(v2,2))/f;
	J_image(5,4) 	= -(u2*v2)/f;
	J_image(5,5) 	= -u2;

	//	Device Jacobian
	rw::math::Jacobian J = _device->baseJframe(_CameraFrame, _state); // Returns jacobian from tool to base frame.
	//	Gets R from base to tool and calculate S
	rw::math::Transform3D<> toolTworld = _device->baseTframe(_CameraFrame, _state);
	rw::math::Rotation3D<> baseRtool = toolTworld.R().inverse();
	rw::math::Jacobian S = Jacobian(baseRtool);
	//	Z_image and Z_image transposed.
	auto Z_image = (J_image*S*J).e();
	auto Z_image_T = Z_image.transpose();
	//	Calculate dq
	auto dq = (Z_image_T * (Z_image*Z_image_T).inverse()) * dudv.e();

	std::cout << "_dq:" << std::endl;
	std::cout << dq << std::endl << std::endl;

	//	Check joint velocities
	rw::math::Q q_new = bracketJointVelocity( rw::math::Q(dq) );

	return q_new;
	//return _device->getQ( _state );
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

	}
	std::cout << "_dq clammed:" << std::endl;
	std::cout <<  _dq << std::endl << std::endl;

	for( unsigned int i = 0; i < q.m().size(); i++ ){ q[i] += _dq[i]; }

    if( doLogging ){
    	logJointPosition.push_back( _dq );
    	logJointVelocity.push_back( q );
    }

	return q;
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

/* 10:58
rw::math::Q testIK::getTrueDeltaQ(){
	rw::math::Transform3D<> camTmarker = inverse( _markerFrame->fTf( _toolFrame, _state ) );

	std::vector<rw::math::Vector3D<> > P;
	P.push_back( camTmarker.R() * P0 + camTmarker.P() );
	P.push_back( camTmarker.R() * P1 + camTmarker.P() );
	P.push_back( camTmarker.R() * P2 + camTmarker.P() );
	P.push_back( camTmarker.P() );

	rw::math::Jacobian uv(numP*2,1);
	for( unsigned int i = 0; i < P.size()-1; i++ ){
		uv(2*i,0) 		= (P[i][0] * f) / z;
		uv(2*i+1,0) 	= (P[i][1] * f) / z;
	}

	if( initialRun ){
		setInitialTrueGoal();
		for( unsigned int i = 0; i < numP*2; i++ ){
			goal[i] = uv(i,0);
		}
		for( unsigned int i = 0; i < numP; i++ ){
			goal[i*2] 	= 0 - (P[P.size()-1][0] * f) / z;
			goal[i*2+1]	= 0 - (P[P.size()-1][1] * f) / z;
		}
		initialRun = false;
	}

	rw::math::Jacobian d_uv(numP*2,1);
	for( unsigned int i = 0; i < numP; i++ ){
		d_uv(2*i,0)		= goal[i*2] 	-uv(i*2,0);
		d_uv(2*i+1,0)	= goal[i*2+1]	-uv(i*2+1,0);
	}

	rw::math::Jacobian J_image(2*numP,6);
	for( unsigned int i = 0; i < numP; i++ ){
		double u = uv(i*2,0);
		double v = uv(i*2+1,0);

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
	rw::math::Jacobian J = _device->baseJframe( _toolFrame, _state);

	rw::math::Transform3D<> tmp = inverse( _device->baseTframe( _toolFrame, _state) );
	rw::math::Jacobian S( tmp.R() );

	rw::math::Jacobian Z_image = J_image * S * J;
	rw::math::Jacobian Z_image_T = transpose( Z_image );

	rw::math::Jacobian J_dq( Z_image_T.e() * d_uv.e() );
	rw::math::Q dq( J_dq.e() );

	return dq;
}
*/

/*	8:52
rw::math::Q testIK::getTrueDeltaQ(){
	MovableFrame* _WorldFrame = (MovableFrame*) _wc->findFrame("WORLD");
	MovableFrame* _CameraFrame = (MovableFrame*) _wc->findFrame("CameraSim");
	MovableFrame* _ToolFrame = (MovableFrame*) _wc->findFrame("Tool");
	MovableFrame* _MarkerFrame = (MovableFrame*) _wc->findFrame("Marker");

/*
	rw::kinematics::FKRange forwardKinematicRangeMarker( _WorldFrame, _MarkerFrame, _state );
	rw::math::Transform3D<> _transforMarker = forwardKinematicRangeMarker.get( _state );
*//*
	rw::math::Vector3D<> _offset0(0.0,	0.0,	0.0);
	rw::math::Vector3D<> _offset1(-0.1,	0.0,	0.0);
	rw::math::Vector3D<> _offset2(0.0,	-0.1,	0.0);
/*
	rw::math::Transform3D<> _transformOffset0(_offset0);
	rw::math::Transform3D<> _transformOffset1(_offset1);
	rw::math::Transform3D<> _transformOffset2(_offset2);
*//*

//	rw::math::Transform3D<> marTwor = Kinematics::frameTframe(_MarkerFrame, _WorldFrame,_state);

	rw::math::Transform3D<> camTmar = Kinematics::frameTframe(_ToolFrame, _MarkerFrame,_state);
/*
	rw::math::Transform3D<> marPos0Fmar = marTwor * _transforMarker * _transformOffset0;
	rw::math::Transform3D<> marPos1Fmar = marTwor * _transforMarker * _transformOffset1;
	rw::math::Transform3D<> marPos2Fmar = marTwor * _transforMarker * _transformOffset2;
*/
/*
	rw::math::Transform3D<> marPos0Fcam = camTmar * marPos0Fmar;
	rw::math::Transform3D<> marPos1Fcam = camTmar * marPos1Fmar;
	rw::math::Transform3D<> marPos2Fcam = camTmar * marPos2Fmar;
*//*
	std::vector<rw::math::Vector3D<> > P;
	P.push_back( camTmar.R() * P0 + camTmar.P() );
	P.push_back( camTmar.R() * P1 + camTmar.P() );
	P.push_back( camTmar.R() * P2 + camTmar.P() );
	//P.push_back( camTmar.P() );

	double x0 =  P[0][0];
	double y0 =  P[0][1];
	double x1 =  P[1][0];
	double y1 =  P[1][1];
	double x2 =  P[2][0];
	double y2 =  P[2][1];
/*
	double x00 =  marPos0Fcam.P()[0];
	double y00 =  marPos0Fcam.P()[1];
	double x10 =  marPos1Fcam.P()[0];
	double y10 =  marPos1Fcam.P()[1];
	double x20 =  marPos2Fcam.P()[0];
	double y20 =  marPos2Fcam.P()[1];
*/
/*
	rw::math::Vector3D<> _offset0Fcam = camTmar.R() * P0 + camTmar.P();
	rw::math::Vector3D<> _offset1Fcam = camTmar.R() * P1 + camTmar.P();
	rw::math::Vector3D<> _offset2Fcam = camTmar.R() * P2 + camTmar.P();
*//*

	float u0 = (x0*f)/z;
	float v0 = (y0*f)/z;
	float u1 = (x1*f)/z;
	float v1 = (y1*f)/z;
	float u2 = (x2*f)/z;
	float v2 = (y2*f)/z;

	std::cout << "uv:" << std::endl;
	std::cout << u0 << "\t" << u1 << "\t" << u2 << std::endl;
	std::cout << v0 << "\t" << v1 << "\t" << v2 << std::endl;
	std::cout << std::endl;

	rw::math::Jacobian dudv(6,1);
	dudv(0,0) = (_offset0(0)*f)/z - u0;
	dudv(1,0) = (_offset0(1)*f)/z - v0;
	dudv(2,0) = (_offset1(0)*f)/z - u1;
	dudv(3,0) = (_offset1(1)*f)/z - v1;
	dudv(4,0) = (_offset2(0)*f)/z - u2;
	dudv(5,0) = (_offset2(1)*f)/z - v2;

	std::cout << "dudv:" << std::endl;
	std::cout << f*_offset0(0)/z - u0 << "\t" << f*_offset1(0)/z - u1 << "\t" << f*_offset2(0)/z - u2 << std::endl;
	std::cout << f*_offset0(1)/z - v0 << "\t" << f*_offset1(1)/z - v1 << "\t" << f*_offset2(1)/z - v2 << std::endl;
	std::cout << std::endl;

	rw::math::Jacobian J_image(6,6);
	J_image(0,0) 	= -f/z;
	J_image(0,1) 	= 0;
	J_image(0,2) 	= u0/z;
	J_image(0,3) 	= (u0*v0)/f;
	J_image(0,4) 	= -(pow(f,2)+pow(u0,2))/f;
	J_image(0,5) 	= v0;
	J_image(1,0) 	= 0;
	J_image(1,1) 	= -f/z;
	J_image(1,2) 	= v0/z;
	J_image(1,3) 	= (pow(f,2)+pow(v0,2))/f;
	J_image(1,4) 	= -(u0*v0)/f;
	J_image(1,5) 	= -u0;
	J_image(2,0) 	= -f/z;
	J_image(2,1) 	= 0;
	J_image(2,2) 	= u1/z;
	J_image(2,3) 	= (u1*v1)/f;
	J_image(2,4) 	= -(pow(f,2)+pow(u1,2))/f;
	J_image(2,5) 	= v1;
	J_image(3,0) 	= 0;
	J_image(3,1) 	= -f/z;
	J_image(3,2) 	= v1/z;
	J_image(3,3) 	= (pow(f,2)+pow(v1,2))/f;
	J_image(3,4) 	= -(u1*v1)/f;
	J_image(3,5) 	= -u1;
	J_image(4,0) 	= -f/z;
	J_image(4,1) 	= 0;
	J_image(4,2) 	= u2/z;
	J_image(4,3) 	= (u2*v2)/f;
	J_image(4,4) 	= -(pow(f,2)+pow(u2,2))/f;
	J_image(4,5) 	= v2;
	J_image(5,0) 	= 0;
	J_image(5,1) 	= -f/z;
	J_image(5,2) 	= v2/z;
	J_image(5,3) 	= (pow(f,2)+pow(v2,2))/f;
	J_image(5,4) 	= -(u2*v2)/f;
	J_image(5,5) 	= -u2;

	//	Device Jacobian
	rw::math::Jacobian J = _device->baseJframe(_ToolFrame, _state); // Returns jacobian from tool to base frame.
	//	Gets R from base to tool and calculate S
	rw::math::Transform3D<> TToolWorld = _device->baseTframe(_ToolFrame, _state);
	rw::math::Rotation3D<> RBaseTool = TToolWorld.R().inverse();
	rw::math::Jacobian S = Jacobian(RBaseTool);
	//	Z_image and Z_image transposed.

	auto Z_image = (J_image*S*J).e();
	auto Z_image_T = Z_image.transpose();
	//	Calculate dq
	auto dq = (Z_image_T * (Z_image*Z_image_T).inverse()) * dudv.e();
	//	Check joint velocities
	rw::math::Q q_new = bracketJointVelocity( rw::math::Q(dq) );

	std::cout << "Test:" << q_new << std::endl;
	std::cout << std::endl;

	return q_new;
}
*/

/*	Earlier than 8:52
rw::math::Q testIK::getTrueDeltaQ(){
	MovableFrame* _WorldFrame = (MovableFrame*) _wc->findFrame("WORLD");
	MovableFrame* _CameraFrame = (MovableFrame*) _wc->findFrame("CameraSim");
	MovableFrame* _ToolFrame = (MovableFrame*) _wc->findFrame("Tool");
	MovableFrame* _MarkerFrame = (MovableFrame*) _wc->findFrame("Marker");

	rw::math::Vector3D<> Offset0(0.0,	0.0,	0);
	rw::math::Vector3D<> Offset1(-0.1,	0.0,	0);
	rw::math::Vector3D<> Offset2(0.0,	-0.1,	0);

	rw::math::Transform3D<double> Offset0T(Offset0);
	rw::math::Transform3D<double> Offset1T(Offset1);
	rw::math::Transform3D<double> Offset2T(Offset2);

	rw::kinematics::FKRange forwardKinematicRangeMarker( _WorldFrame, _MarkerFrame, _state );
	rw::math::Transform3D<> MarkerTransform3D = forwardKinematicRangeMarker.get( _state );

	rw::math::Transform3D<> MarkerTWorld = Kinematics::frameTframe(_MarkerFrame, _WorldFrame,_state);
	rw::math::Transform3D<> CameraTMarker = Kinematics::frameTframe(_ToolFrame, _MarkerFrame,_state);

	rw::math::Transform3D<> MarkerPosition0InMarkerFrame = MarkerTWorld * MarkerTransform3D * Offset0T;
	rw::math::Transform3D<> MarkerPosition1InMarkerFrame = MarkerTWorld * MarkerTransform3D * Offset1T;
	rw::math::Transform3D<> MarkerPosition2InMarkerFrame = MarkerTWorld * MarkerTransform3D * Offset2T;

	rw::math::Transform3D<> MarkerPosition0InCameraFrame = CameraTMarker * MarkerPosition0InMarkerFrame;
	rw::math::Transform3D<> MarkerPosition1InCameraFrame = CameraTMarker * MarkerPosition1InMarkerFrame;
	rw::math::Transform3D<> MarkerPosition2InCameraFrame = CameraTMarker * MarkerPosition2InMarkerFrame;

	rw::math::Jacobian P(2*numP,1);.e()
	P(0,0) = MarkerPosition0InCameraFrame.P()[0];
	P(1,0) = MarkerPosition0InCameraFrame.P()[0];/*
	P(2,0) = MarkerPosition0InCameraFrame.P()[1];
	P(3,0) = MarkerPosition1InCameraFrame.P()[0];
	P(4,0) = MarkerPosition1InCameraFrame.P()[1];
	P(5,0) = MarkerPosition2InCameraFrame.P()[0];
	P(6,0) = MarkerPosition2InCameraFrame.P()[1];*//*

	rw::math::Vector3D<> Offset0InCamera = CameraTMarker.R() * Offset0 + CameraTMarker.P();
	rw::math::Vector3D<> Offset1InCamera = CameraTMarker.R() * Offset1 + CameraTMarker.P();
	rw::math::Vector3D<> Offset2InCamera = CameraTMarker.R() * Offset2 + CameraTMarker.P();

	rw::math::Jacobian uv(2*numP,1);
	for( unsigned int i = 0; i < numP; i++ ){
		uv(2*i,0) 	= (P(2*i,0)*f)/z;
		uv(2*i+1,0) = (P(2*i+1,0)*f)/z;
	}

	rw::math::Jacobian dudv(2*numP,1);
	dudv(0,0) = f*Offset0(0)/z - uv(0,0);
	dudv(1,0) = f*Offset0(1)/z - uv(1,0);/*
	dudv(2,0) = f*Offset1(0)/z - uv(2,0);
	dudv(3,0) = f*Offset1(1)/z - uv(3,0);
	dudv(4,0) = f*Offset2(0)/z - uv(4,0);
	dudv(5,0) = f*Offset2(1)/z - uv(5,0);*//*

	rw::math::Jacobian J_image(2*numP,6);
	for( unsigned int i = 0; i < numP; i++ ){
		double u = uv(i*2,0);
		double v = uv(i*2+1,0);
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

	//	Device Jacobian
	rw::math::Jacobian J = _device->baseJframe(_ToolFrame, _state); // Returns jacobian from tool to base frame.
	//	Gets R from base to tool and calculate S
	rw::math::Transform3D<> TToolWorld = _device->baseTframe(_ToolFrame, _state);
	rw::math::Rotation3D<> RBaseTool = TToolWorld.R().inverse();
	rw::math::Jacobian S = Jacobian(RBaseTool);
	//	Z_image and Z_image transposed.

	auto Z_image = (J_image*S*J).e();
	auto Z_image_T = Z_image.transpose();
	//	Calculate dq
	auto dq = (Z_image_T * (Z_image*Z_image_T).inverse()) * dudv.e();
	//	Check joint velocities
	rw::math::Q q_new = bracketJointVelocity( rw::math::Q(dq) );
	std::cout << "Test:" << q_new << std::endl;
	return q_new;
}
*/

/*
	Eigen::MatrixXd U;
	Eigen::VectorXd SIGMA;
	Eigen::MatrixXd V;
	rw::math::LinearAlgebra::svd(J_image.e(),U,SIGMA,V);

	std::cout << "---SV---" << std::endl;
	std::cout << SIGMA << std::endl;
	std::cout << std::endl;
*/
