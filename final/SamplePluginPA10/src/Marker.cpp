/*
 * Marker.cpp
 *
 *  Created on: Nov 22, 2016
 *      Author: theis
 */

#include "Marker.hpp"

Marker::Marker( std::string filename ) {
	loadTextFile( filename );
}

Marker::~Marker() {
}

rw::kinematics::State & Marker::step(){
	/**
	 * Update the state, and increment index.
	 */





	return state;
}

void Marker::loadTextFile( std::string filename ){
	/**
	 * 	Load the text file.
	 */
}
