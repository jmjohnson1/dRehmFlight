// Imports the flight test data from the IMU and motion capture data to test
// the Kalman filter functions.
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include "csvParser.h"
#include "uNavINS.h"

//#define USE_VELOCITY

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

	std::string imuData_path = "./csv/imu_data.csv";
	std::string imuTime_path = "./csv/imu_time.csv";
	std::string mocapPos_path = "./csv/mocap_pos.csv";
	std::string mocapTime_path = "./csv/mocap_time.csv";
#ifdef USE_VELOCITY
	std::string mocapVel_path = "./csv/mocap_vel.csv";
#endif
	MatrixXf imuData = load_csv<MatrixXf>(imuData_path);
	MatrixXf imuTime = load_csv<MatrixXf>(imuTime_path);
	MatrixXf mocapPos = load_csv<MatrixXf>(mocapPos_path);
#ifdef USE_VELOCITY
	MatrixXf mocapVel = load_csv<MatrixXf>(mocapVel_path);
#endif
	MatrixXf mocapTime = load_csv<MatrixXf>(mocapTime_path);
	std::cout << "imuData - rows: " << imuData.rows() << " cols: " << imuData.cols() << std::endl;
	std::cout << "imuTime - rows: " << imuTime.rows() << " cols: " << imuTime.cols() << std::endl;
	std::cout << "mocapPos - rows: " << mocapPos.rows() << " cols: " << mocapPos.cols() << std::endl;
	std::cout << "mocapTime - rows: " << mocapTime.rows() << " cols: " << mocapTime.cols() << std::endl;

	std::cout << "... csv data loaded." << std::endl;

	// Convert imu gyro to rad/s
	imuData.block(0, 3, imuData.rows(), 3) = imuData.block(0, 3, imuData.rows(), 3)*M_PI/180.0f;

	// Convert accelerometer data to m/s/s
	imuData.block(0, 0, imuData.rows(), 3) = imuData.block(0, 0, imuData.rows(), 3)*9.807f;

	// Change mocap data units to m from mm
	mocapPos = mocapPos/1000.0f;

	// Figure out index for start of mocap
	int startIndex = find(mocapTime, imuTime(0));
	std::cout << "Start index = " << startIndex << std::endl;

	// Set the initial position to be (0, 0, 0);
	Matrix<float, Dynamic, Dynamic> coordinateShift(mocapPos.rows(), mocapPos.cols());
	coordinateShift.col(0).setConstant(mocapPos(startIndex, 0));
	coordinateShift.col(1).setConstant(mocapPos(startIndex, 1));
	coordinateShift.col(2).setConstant(mocapPos(startIndex, 2));
	mocapPos = mocapPos - coordinateShift;

	// Export data after preprocessing for debug
	write_csv("./csv/imuData_debug.csv", imuData);
	write_csv("./csv/mocapPos_debug.csv", mocapPos);

	ins.Configure();
	ins.Initialize(imuData(0, seq(3, 5)), imuData(0, seq(0, 2)), Vector3d::Zero());

	uint64_t previousMeasUpdateTime_us = 0;
	Vector3d posMeas = {0, 0, 0};
#ifdef USE_VELOCITY
	Vector3f velMeas = {0, 0, 0};
#endif
	unsigned long tow = 0;
	Matrix<float, 15, Dynamic> outputState;

	// Main loop
	for (int imu_index = 0; imu_index < imuTime.size(); imu_index++) {
	//for (int imu_index = 1600; imu_index < 7600; imu_index++) {
		uint64_t currentTime_us = imuTime(imu_index)*1e06;

		// Find out if it's time for a measurement update
		if ((currentTime_us - previousMeasUpdateTime_us) > 1e06) {
			int mocap_index = find(mocapTime, imuTime(imu_index));
			posMeas = mocapPos.row(mocap_index).cast <double> ();
#ifdef USE_VELOCITY
			velMeas = mocapVel.row(mocap_index);
#endif
			previousMeasUpdateTime_us = currentTime_us;
			tow += 1;
		}
#ifdef USE_VELOCITY		
		ins.Update(currentTime_us, tow, imuData(imu_index, seq(3, 5)), imuData(imu_index, seq(0, 2)), posMeas, velMeas);
#else
		ins.Update(currentTime_us, tow, imuData(imu_index, seq(3, 5)), imuData(imu_index, seq(0, 2)), posMeas);
#endif
		

		Vector<float, 15> currentState = ins.Get_State();
		outputState.conservativeResize(outputState.rows(), outputState.cols() + 1);
		outputState.col(outputState.cols() - 1) = currentState;
	}
	write_csv("./csv/outputState.csv", outputState);

	return 0;
}
