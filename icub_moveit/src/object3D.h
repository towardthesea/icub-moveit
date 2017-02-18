/*
 * object3D.h
 *
 *  Created on: Jul 8, 2016
 *      Author: pnguyen
 */

#ifndef SRC_OBJECT3D_H_
#define SRC_OBJECT3D_H_

#include <vector>

using namespace std;

class object3D {
public:
	object3D();

	vector<double> position;
	vector<double> dimension;
	virtual ~object3D();
};

#endif /* SRC_OBJECT3D_H_ */
