#include <rw/rw.hpp>
#include <rw/math/Q.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/common.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
 
using namespace rw;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::common;
using namespace rwlibs::proximitystrategies;

float epsilon = 0.1;

bool alg1( State state, Device::Ptr device, CollisionDetector::Ptr CD, rw::math::Q q_init, rw::math::Q q_goal )
{
	float deltaQ = rw::math::Distance(q_init, q_goal);
	int n = ( rw::math::Math::abs(deltaQ) / epsilon ) - 1;

	for (int i = 1; i<n; i++ )
	{
		rw::math::Q q_i = i * epsilon * deltaQ/(rw::Math::abs(deltaQ)) + q_init;

		//	Check collision
		device->setQ(q_i, state);
		CollisionDetector::QueryResult data;
		if( CD->inCollision(state,&data) ){
			return false;
		}
	}
	return true;
}

bool alg2( State state, Device::Ptr device, CollisionDetector::Ptr CD, rw::math::Q q_init, rw::math::Q q_goal )
{
	float deltaQ = rw::math::Distance(q_init, q_goal);
	int n = ( rw::math::Math::abs(deltaQ) / epsilon ) - 1;
	float step = deltaQ / (n + 1);

	for (int i = 1; i<n; i++ )
	{
		rw::math::Q q_i = i * step + q_init;

		//	Check collision
		device->setQ(q_i, state);
		CollisionDetector::QueryResult data;
		if( CD->inCollision(state,&data) ){
			return false;
		}
	}
	return true;
}

bool alg3( State state, Device::Ptr device, CollisionDetector::Ptr CD, rw::math::Q q_init, rw::math::Q q_goal )
{
	float deltaQ = rw::math::Distance(q_init, q_goal);
	int n = ( rw::math::Math::abs(deltaQ) / epsilon );
	int levels = rw::math::Math::ceilLog2( n );

	for (int i = 1; i<levels; i++ )
	{
		float steps = pow(2, i-1);
		float step = deltaQ / steps;

		for (int j = 1; j < steps; j++ )
		{	
			rw::math::Q q_i = q_init + ( j-0.5 ) * step;
			
			//	Check collision
			device->setQ(q_i, state);
			CollisionDetector::QueryResult data;
			if( CD->inCollision(state,&data) ){
				return false;
			}
		}
	}
	return true;
}

bool alg4( State state, Device::Ptr device, CollisionDetector::Ptr CD, rw::math::Q q_init, rw::math::Q q_goal )
{
	float deltaQ = rw::math::Distance(q_init, q_goal);
	int n = ( rw::math::Math::abs(deltaQ) / epsilon );
	int levels = rw::math::Math::ceilLog2( n );
	float expDeltaQ = deltaQ/rw::math::Math::abs(deltaQ) * pow(2, levels) * epsilon;


	for (int i = 1; i<levels; i++ )
	{
		float steps = pow(2, i-1);
		float step = expDeltaQ / steps;

		for (int j = 1; j < steps; j++ )
		{	
			rw::math::Q q_i = q_init + ( j-0.5 ) * step;
			if( rw::math::Distance( q_i, q_init ) < rw::math::Math::abs(deltaQ) )
			{
			
				//	Check collision
				device->setQ(q_i, state);
				CollisionDetector::QueryResult data;
				if( CD->inCollision(state,&data) ){
					return false;
				}
			}
		}
	}
	return true;
}

int main(){
	/*
	*	Load the Workcell and Device
	*/
	std::string wcFile = "/home/theis/workspace/robotics/collision/KukaKr16/Scene.wc.xml";
	std::string deviceName = "KukaKr16";
	std::cout << "Trying to use workcell: " << wcFile << " and device " << deviceName << std::endl;
 
	WorkCell::Ptr workcell = rw::loaders::WorkCellLoader::Factory::load(wcFile);
	if (workcell == NULL) {
		std::cerr << "Workcell: " << workcell << " not found!" << std::endl;
		return -1;
	}
	Device::Ptr device = workcell->findDevice(deviceName);
	if (device == NULL) {
		std::cerr << "Device: " << deviceName << " not found!" << std::endl;
		return -1;
	}
	State state = workcell->getDefaultState();
	CollisionDetector::Ptr detector = new CollisionDetector(workcell, ProximityStrategyFactory::makeDefaultCollisionStrategy());

	/*
	*	Find two random positions
	*/
	rw::math::Q q_init(6,0,0,0,0,0,0);
	rw::math::Q q_goal(6,1,1,1,1,1,1);

	/*
		Do collision detection with the algorithms
	*/
	alg1(state, device, detector, q_init, q_goal);

	return 0;
}
