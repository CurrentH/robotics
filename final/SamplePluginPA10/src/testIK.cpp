/*
 * testIK.cpp
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

	rw::math::Q q = rw::math::Q( 6, 0,0,0,0,0,0 );

	return q;
}

