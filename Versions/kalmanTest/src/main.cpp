// Imports the flight test data from the IMU and motion capture data to test
// the Kalman filter functions.

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>

#include "csvParser.h"
#include "uNavINS.h"
#include "Eigen/Dense"

using namespace Eigen;

template <typename Derived>
int find(const MatrixBase<Derived> &A, float val) {
	int idx = 0;
	float minValue = abs(A(idx));
	for (int i = 0; i < A.size(); i++) {
		float currentValue = abs(A(i) - val);
		if (currentValue < minValue) {
			minValue = currentValue; idx = i;
		}
	}
	return idx;
}

int main() {
	uNavINS ins;

	std::cout << "Loading csv data..." << std::endl;

	// Using new flight data which includes all fields above in a single csv file
	std::string flightDataPath = "./csv/flightData.csv";
	MatrixXf flightData = load_csv<MatrixXf>(flightDataPath);
	std::cout << "Flight data - rows: " << flightData.rows() << " cols: " << flightData.cols() << std::endl;

	std::cout << "... csv data loaded." << std::endl;


	ins.Configure();
	ins.Initialize(flightData(5, seq(4, 6)), flightData(5, seq(1, 3)), flightData(5, seq(7, 9)).cast<double>());

	Matrix<float, 15, Dynamic> outputState;
	outputState.resize(15, flightData.rows());
	Matrix<float, 3, Dynamic> innovations;
	innovations.resize(3, flightData.rows());
	Matrix<float, 3, Dynamic> residual;
	residual.resize(3, flightData.rows());

	// Main loop
	for (int LV1 = 0; LV1 < flightData.rows(); LV1++) {
		uint64_t currentTime_us = flightData(LV1, 0);
		uint64_t tow = flightData(LV1, 10);
		ins.Update(currentTime_us, tow, flightData(LV1, seq(4,6)), flightData(LV1, seq(1, 3)), flightData(LV1, seq(7, 9)).cast<double>());
		Vector<float, 15> currentState = ins.Get_State();
		outputState.col(LV1) = currentState;
		Vector3f innovation = ins.Get_InnovationPos();
		innovations.col(LV1) = innovation;
		residual.col(LV1) = ins.Get_Residual();
	}
	write_csv("./csv/outputState.csv", outputState);
	write_csv("./csv/innovations.csv", innovations);
	write_csv("./csv/residual.csv", residual);

	return 0;
}
