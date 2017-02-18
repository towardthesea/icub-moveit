/*
 * object3D.cpp
 *
 *  Created on: Jul 8, 2016
 *      Author: pnguyen
 */

#include "object3D.h"

object3D::object3D() {
	// TODO Auto-generated constructor stub
	position.resize(3);
	dimension.resize(3);
}

object3D::~object3D() {
	// TODO Auto-generated destructor stub
	position.clear();
	dimension.clear();
}

