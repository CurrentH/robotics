#include <vector>
#include <math.h>
#include <iostream>

int main() {
	int numberLinks = 2;
	std::vector<double> linkLenghts = { 1, 1 };
	std::vector<double> startAngles = { 0, 0 };
	std::vector<double> startPos = { 1, 1, 1 };
	std::vector<double> endPos = { 0, 1, 1 };

	step( linkLenghts, startAngles, startPos, endPos );

	return (0);
}

void step( std::vector<double> l, std::vector<double> sa, std::vector<double> sp, std::vector<double> ep ){
	/**
	 * 	We get all the info we need here. We will then, in here do all the calculations,
	 * 	and when done, end the function here.
	 */

	double currentDist = sqrt( pow(ep[0] - sp[0], 2) + pow(ep[1] - sp[1], 2) + pow(ep[2] - sp[2], 2));
	double treshold = 0.005;

	while( currentDist > treshold ){




	}
}

void matrixMulti(std::vector<std::vector<double>> &newM,
		std::vector<std::vector<double>> m1,
		std::vector<std::vector<double>> m2) {
	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 4; col++) {
			for (int inner = 0; inner < 3; inner++) {
				newM[row][col] += m1[row][inner] * m2[inner][col];
			}
		}
	}
}

void rotate(std::vector<std::vector<double>> &matrix, double x, double y,
		double z) {
	matrix[0][0] = cos(y) * cos(z);						//	R11
	matrix[0][1] = sin(x) * sin(y) * cos(z) - cos(x) * sin(z);	//	R12
	matrix[0][2] = cos(x) * sin(y) * cos(z) + sin(x) * sin(z);	//	R13

	matrix[1][0] = cos(y) * sin(z);						//	R21
	matrix[1][1] = sin(x) * sin(y) * sin(z) + cos(x) * cos(z);	//	R22
	matrix[1][2] = cos(x) * sin(y) * sin(z) - sin(x) * cos(z);	//	R23

	matrix[2][0] = -sin(y);								//	R31
	matrix[2][1] = sin(x) * cos(y);						//	R32
	matrix[2][2] = cos(x) * cos(y);						//	R33
}

void homo(std::vector<std::vector<double>> &t,
		std::vector<std::vector<double>> r, std::vector<double> p) {
	t[0][0] = r[0][0];
	t[0][1] = r[0][1];
	t[0][2] = r[0][2];
	t[0][3] = p[0];

	t[1][0] = r[1][0];
	t[1][1] = r[1][1];
	t[1][2] = r[1][2];
	t[1][3] = p[1];

	t[2][0] = r[2][0];
	t[2][1] = r[2][1];
	t[2][2] = r[2][2];
	t[2][3] = p[2];

	t[3][0] = 0;
	t[3][1] = 0;
	t[3][2] = 0;
	t[3][3] = 1;
}

void kinematic(std::vector<double> &newAngle, std::vector<double> theta) {
	std::vector<double> tc;		//	TC offset.
	std::vector<double> coxa, rc;
	std::vector<double> femur, rf;
	std::vector<double> tibia, rt;

	double scale = 3.75 + 9.111 + 10.324;

	tc = {0,0,0};

	//	Translation and rotation of TC
	std::vector<std::vector<double>> rab(3, std::vector<double>(3));
	std::vector<std::vector<double>> tab(4, std::vector<double>(4));
	rotate(rab, rc[0], 0, rc[1] + theta[0]);
	homo(tab, rab, tc);

	//	Translation and rotation of CF
	std::vector<std::vector<double>> rbc(3, std::vector<double>(3));
	std::vector<std::vector<double>> tbc(4, std::vector<double>(4));
	rotate(rbc, 0, rf[1] + theta[1], rf[0]);
	homo(tbc, rbc, coxa);

	//	Translation and rotation of FT
	std::vector<std::vector<double>> rcd(3, std::vector<double>(3));
	std::vector<std::vector<double>> tcd(4, std::vector<double>(4));
	rotate(rcd, 0, rt[0] + theta[2], 0);
	homo(tcd, rcd, femur);

	//	Translation and rotation of End effector
	std::vector<std::vector<double>> rd(3, std::vector<double>(3));
	std::vector<std::vector<double>> td(4, std::vector<double>(4));
	rotate(rd, 0, 0, 0);
	homo(td, rd, tibia);

	//	Now we need to return the values.
	std::vector<std::vector<double>> temp1(4, std::vector<double>(4));
	std::vector<std::vector<double>> temp2(4, std::vector<double>(4));
	std::vector<std::vector<double>> temp3(4, std::vector<double>(4));

	matrixMulti(temp1, tab, tbc);
	matrixMulti(temp2, temp1, tcd);
	matrixMulti(temp3, temp2, td);

	newAngle.push_back(temp3[0][3]);
	newAngle.push_back(temp3[1][3]);
	newAngle.push_back(-temp3[2][3]);
}

void legPositionControl(std::vector<std::vector<double>> &angleVector,
		int legNum) {
	std::vector<double> targetPos = { 0, 0, 0 };

	//	The current angle
	std::vector<double> currentAngle;

	//	Do forward kinematics to find the current position.
	std::vector<double> currentPos;
	kinematic(currentPos, currentAngle);

	//	Find the error between the target- and current position.
	std::vector<double> errorPos = { targetPos[0] - currentPos[0], targetPos[1]
			- currentPos[1], targetPos[2] - currentPos[2] };

	std::vector<double> deltaStep;
	std::vector<double> stepPosition;
	double dist_test;
	double nummerical_value = 0.001;
	double stepsize = 0.0005;
//used in the bottom of the function
	double threshold = 0.01; //todo: put in .h

	while (true) {
		//	Step we want to take
		deltaStep = {errorPos[0]*stepsize, errorPos[1]*stepsize, errorPos[2]*stepsize};

		//	Position we want to hit
		stepPosition = {currentPos[0] + deltaStep[0], currentPos[1] + deltaStep[1], currentPos[2] + deltaStep[2]};

		dist_test = sqrt( pow(deltaStep[0],2) + pow(deltaStep[1],2) + pow(deltaStep[2],2) );

		if( dist_test < nummerical_value ) {
			stepsize *= 2;
		} else {
			break;
		}
	}

	//	Now that we have the position we want to hit, we try to rotate the joints, to the positions.
	//	Do the six calculations.
	std::vector<double> temp = currentAngle;

	//	joint 1-
	//std::cout << "******CAL1******" << std::endl;
	std::vector<double> cal1;
	currentAngle[0] = currentAngle[0] - stepsize;
	kinematic(cal1, currentAngle);
	currentAngle = temp;

	//	joint 1+
	std::vector<double> cal2;
	//std::cout << "******CAL2******" << std::endl;
	currentAngle[0] = currentAngle[0] + stepsize;
	kinematic(cal2, currentAngle);
	currentAngle = temp;

	//	joint 2-
	//std::cout << "******CAL3******" << std::endl;
	std::vector<double> cal3;
	currentAngle[1] = currentAngle[1] - stepsize;
	kinematic(cal3, currentAngle);
	currentAngle = temp;

	//	joint 2+
	//std::cout << "******CAL4******" << std::endl;
	std::vector<double> cal4;
	currentAngle[1] = currentAngle[1] + stepsize;
	kinematic(cal4, currentAngle);
	currentAngle = temp;

	//	joint 3-
	//std::cout << "******CAL5******" << std::endl;
	std::vector<double> cal5;
	currentAngle[2] = currentAngle[2] - stepsize;
	kinematic(cal5, currentAngle);
	currentAngle = temp;

	//	joint 3+
	//std::cout << "******CAL6******" << std::endl;
	std::vector<double> cal6;
	currentAngle[2] = currentAngle[2] + stepsize;
	kinematic(cal6, currentAngle);
	currentAngle = temp;

	//	Now we need to calculate the distance to the point, with the different joint moves.
	std::vector<double> lt;
	lt.push_back(
			sqrt(
					pow(stepPosition[0] - cal1[0], 2)
							+ pow(stepPosition[1] - cal1[1], 2)
							+ pow(stepPosition[2] - cal1[2], 2)));
	lt.push_back(
			sqrt(
					pow(stepPosition[0] - cal2[0], 2)
							+ pow(stepPosition[1] - cal2[1], 2)
							+ pow(stepPosition[2] - cal2[2], 2)));
	lt.push_back(
			sqrt(
					pow(stepPosition[0] - cal3[0], 2)
							+ pow(stepPosition[1] - cal3[1], 2)
							+ pow(stepPosition[2] - cal3[2], 2)));
	lt.push_back(
			sqrt(
					pow(stepPosition[0] - cal4[0], 2)
							+ pow(stepPosition[1] - cal4[1], 2)
							+ pow(stepPosition[2] - cal4[2], 2)));
	lt.push_back(
			sqrt(
					pow(stepPosition[0] - cal5[0], 2)
							+ pow(stepPosition[1] - cal5[1], 2)
							+ pow(stepPosition[2] - cal5[2], 2)));
	lt.push_back(
			sqrt(
					pow(stepPosition[0] - cal6[0], 2)
							+ pow(stepPosition[1] - cal6[1], 2)
							+ pow(stepPosition[2] - cal6[2], 2)));

	if ((lt[0] < lt[1]) && (lt[0] < lt[2]) && (lt[0] < lt[3]) && (lt[0] < lt[4])
			&& (lt[0] < lt[5])) {
		std::cout << "CASE 0" << std::endl;
		angleVector[legNum][0] = currentAngle[0] - stepsize;
		angleVector[legNum][1] = currentAngle[1];
		angleVector[legNum][2] = currentAngle[2];
	} else if ((lt[1] < lt[0]) && (lt[1] < lt[2]) && (lt[1] < lt[3])
			&& (lt[1] < lt[4]) && (lt[1] < lt[5])) {
		std::cout << "CASE 1" << std::endl;
		angleVector[legNum][0] = currentAngle[0] + stepsize;
		angleVector[legNum][1] = currentAngle[1];
		angleVector[legNum][2] = currentAngle[2];
	} else if ((lt[2] < lt[0]) && (lt[2] < lt[1]) && (lt[2] < lt[3])
			&& (lt[2] < lt[4]) && (lt[2] < lt[5])) {
		std::cout << "CASE 2" << std::endl;
		angleVector[legNum][0] = currentAngle[0];
		angleVector[legNum][1] = currentAngle[1] - stepsize;
		angleVector[legNum][2] = currentAngle[2];
	} else if ((lt[3] < lt[0]) && (lt[3] < lt[1]) && (lt[3] < lt[2])
			&& (lt[3] < lt[4]) && (lt[3] < lt[5])) {
		std::cout << "CASE 3" << std::endl;
		angleVector[legNum][0] = currentAngle[0];
		angleVector[legNum][1] = currentAngle[1] + stepsize;
		angleVector[legNum][2] = currentAngle[2];
	} else if ((lt[4] < lt[0]) && (lt[4] < lt[1]) && (lt[4] < lt[2])
			&& (lt[4] < lt[3]) && (lt[4] < lt[5])) {
		std::cout << "CASE 4" << std::endl;
		angleVector[legNum][0] = currentAngle[0];
		angleVector[legNum][1] = currentAngle[1];
		angleVector[legNum][2] = currentAngle[2] - stepsize;
	} else if ((lt[5] < lt[0]) && (lt[5] < lt[1]) && (lt[5] < lt[2])
			&& (lt[5] < lt[3]) && (lt[5] < lt[4])) {
		std::cout << "CASE 5" << std::endl;
		angleVector[legNum][0] = currentAngle[0];
		angleVector[legNum][1] = currentAngle[1];
		angleVector[legNum][2] = currentAngle[2] + stepsize;
	} else {
		std::cout << "DEFAULT CASE" << std::endl;
		std::cout << lt[0] << " " << lt[1] << " " << lt[2] << " " << lt[3]
				<< " " << lt[4] << " " << lt[5] << std::endl;
	}

	std::vector<double> nextPos;
	temp = {angleVector[legNum][0], angleVector[legNum][1], angleVector[legNum][2]};
	kinematic(nextPos, temp);

	double dist_fin = sqrt(
			pow(targetPos[0] - nextPos[0], 2)
					+ pow(targetPos[1] - nextPos[1], 2)
					+ pow(targetPos[2] - nextPos[2], 2));

	if (dist_fin < threshold) {
		std::cout << "done" << std::endl;
	}
}

