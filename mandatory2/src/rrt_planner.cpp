#include <iostream>
#include <fstream>
#include <rw/rw.hpp>
#include <rw/pathplanning/PathAnalyzer.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

struct Data{
	Data(){}

	double cartesianLength;
	double time;
	uint step;
	double epsilon;
};

#define MAXTIME 10.

bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
	CollisionDetector::QueryResult data;
	bool colFrom;

	State testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		}
		return false;
	}
	return true;
}

void setup(){

}

void generateLua(Q &from, Q &to, Device::Ptr &device, WorkCell::Ptr &wc, double extend, State &state, ofstream &luaFile){
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
	PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

	if (!checkCollisions(device, state, detector, from)){ std::cout << "Collision detected" << std::endl; return;}
	if (!checkCollisions(device, state, detector, to)){ std::cout << "Collision detected" << std::endl; return;}

	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
	QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

	QPath path;
	planner->query(from,to,path,MAXTIME);
	for (QPath::iterator it = path.begin(); it < path.end(); it++) {
		luaFile << "setQ({";
		for (unsigned int j = 0; j < it->size() - 1; j++){
			luaFile << it->m()[j] << ",";
		}
		luaFile << it->m()[it->size() - 1] << "})" << std::endl;
	}
}

Data runIteration(Q &from, Q &to, Device::Ptr &device, WorkCell::Ptr &wc, double extend, State &state, Frame *gripper){
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
	PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

	if (!checkCollisions(device, state, detector, from)) return Data();
	if (!checkCollisions(device, state, detector, to)) return Data();

	PathAnalyzer analyzer(device, state);
	PathAnalyzer::CartesianAnalysis pathLength;

	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
	QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

	QPath path;
	Timer t;
	t.resetAndResume();
	planner->query(from,to,path,MAXTIME);
	t.pause();

	pathLength = analyzer.analyzeCartesian(path, gripper);

	struct Data data;
	data.cartesianLength = pathLength.length;
	data.time = t.getTime();
	data.step = path.size();
	data.epsilon = extend;

	return data;
}

int main(int argc, char** argv) {
	Math::seed(5);

	ofstream luaFile;
	luaFile.open("../src/path.lua");

	const string wcFile = "../../Kr16WallWorkCell/Scene.wc.xml";
	const string deviceName = "KukaKr16";
	const string gripperName = "Tool";
	const string bottleName = "Bottle";
	const string tableName = "Table";

	cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;

	WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
	if (wc == NULL) {cerr << "Workcell: " << wcFile << " not found! " << endl; return 0;}
	Device::Ptr device = wc->findDevice(deviceName);
	if (device == NULL) {cerr << "Device: " << deviceName << " not found!" << endl; return 0;}
	Frame *gripper = wc->findFrame(gripperName);
	if (gripper == NULL) {cerr << "Device: " << gripperName << " not found!" << endl; return 0;}
	Frame *bottle = wc->findFrame(bottleName);
	if (bottle == NULL) {cerr << "Device: " << bottleName << " not found!" << endl; return 0;}
	Frame *table = wc->findFrame(tableName);
	if (table == NULL) {cerr << "Device: " << tableName << " not found!" << endl; return 0;}

	State state = wc->getDefaultState();

// We plan from the default position, to the bottle position.
// then afterwards we move from the bottle to the table, and back to the start position.

	Q start = device->getQ(state);
	Q from(6, -3.142, -0.827, -3.002, -3.143, 0.099, -1.573);
	Q to(6, 1.571, 0.006, 0.030, 0.153, 0.762, 4.490 );

	double extend = 0.2;

	device->setQ(start, state);
	generateLua(start, from, device, wc, extend, state, luaFile);

	// Goto bottle and grip
	device->setQ(from, state);
	Kinematics::gripFrame(bottle, gripper, state);
	luaFile << "attach(bottle, gripper)" << std::endl;
	generateLua(from, to, device, wc, extend, state, luaFile);

	device->setQ(to, state);
	Kinematics::gripFrame(bottle, table, state);
	luaFile << "attach(bottle, table)" << std::endl;
	generateLua(start, to, device, wc, extend, state, luaFile);

	luaFile.close();
	ofstream statFile;
	statFile.open("stat_file.csv");

// The above only handle the visual run for the robot, whereas below the statistical analysis
// of the planner is done.
	device->setQ(from,state);
	Kinematics::gripFrame(bottle, gripper, state);
	while(extend<2*Pi){
		std::cout << "Start iteration with extend: " << extend << std::endl;
		for (int i = 0; i < 100; i++){
			struct Data temp = runIteration(from, to, device, wc, extend, state, gripper);
			statFile << temp.cartesianLength << "," << temp.time << "," << temp.step << "," << temp.epsilon << std::endl;
			std::cout << "\t iteration: " << i << " done." << std::endl;
		}
		extend += 0.01;	
	}

	statFile.close();
	return 0;
}
