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

Transform3D<> T1 & Marker::step(){
	/**
	 * Update the state, and increment index.
	 */





	return transformation;
}

void Marker::loadTextFile( std::string filename ){
	/**
	 * 	Load the text file.
	 */
}
