/*
 * testMarker.cpp
 *
 *  Created on: Nov 22, 2016
 *      Author: theis
 */

#include "/home/theis/workspace/robotics/final/SamplePluginPA10/src/testMarker.hpp"

testMarker::testMarker(std::string filename) {
	loadMotions(filename);
}

testMarker::~testMarker() {
}

rw::math::Transform3D<> testMarker::step(){
	rw::math::Vector3D<> temp_vector= rw::math::Vector3D<>( motions[index][0], motions[index][1], motions[index][2] );
	rw::math::RPY<> temp_rotation = rw::math::RPY<>( motions[index][3], motions[index][4], motions[index][5] );
	rw::math::Transform3D<> T1 = rw::math::Transform3D<>( temp_vector, temp_rotation.toRotation3D() );

	index++;

	return T1;
}

bool testMarker::sequenceDone(){
	if( index == motions.size() ) return true;
	return false;
}

void testMarker::resetIndex(){
	index = 0;
}

void testMarker::loadMotions(std::string filename){
	std::ifstream file(filename);
	std::string word;

	motions.clear();
	resetIndex();

	int wordCount = 0;

	if (file.is_open()){
		while (file >> word){
			if (wordCount % 6 == 0){
				motions.push_back(std::vector<double>(6, 0));
			}
			motions[wordCount / 6][wordCount % 6] = stod(word);
			wordCount++;
		}
	}
}



