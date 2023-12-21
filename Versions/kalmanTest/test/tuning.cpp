// Imports the flight test data from the IMU and motion capture data to test
// the Kalman filter functions.

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <fstream>

#include "csvParser.h"
#include "uNavINS.h"
#include "statFun.h"

#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

void printProgress(double percentage) {
    int val = (int) (percentage * 100);
    int lpad = (int) (percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
    fflush(stdout);
}

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
	std::ofstream outputFile;
	uNavINS ins;
	float minQ = 0.1f;
	float maxQ = 2.0f;
	float stepQ = 0.1f;

	outputFile.open("rmse.csv");

	if (outputFile.is_open()) {
		std::cout << "output file open" << std::endl;
		outputFile << "scaleQ,rmse_x,rmse_y,rmse_z" << std::endl;
 	}

	std::cout << "Loading csv data..." << std::endl;

	std::string imuData_path = "./csv/imu_data.csv";
	std::string imuTime_path = "./csv/imu_time.csv";
	std::string mocapPos_path = "./csv/mocap_pos.csv";
	std::string mocapTime_path = "./csv/mocap_time.csv";
	MatrixXf imuData = load_csv<MatrixXf>(imuData_path);
	MatrixXf imuTime = load_csv<MatrixXf>(imuTime_path);
	MatrixXf mocapPos = load_csv<MatrixXf>(mocapPos_path);
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
	//mocapPos = mocapPos/1000.0f;

	// Figure out index for start of mocap
	int startIndex = find(mocapTime, imuTime(0));
	std::cout << "Start index = " << startIndex << std::endl;

	// Set the initial position to be (0, 0, 0);
	Matrix<float, Dynamic, Dynamic> coordinateShift(mocapPos.rows(), mocapPos.cols());
	coordinateShift.col(0).setConstant(mocapPos(startIndex, 0));
	coordinateShift.col(1).setConstant(mocapPos(startIndex, 1));
	coordinateShift.col(2).setConstant(mocapPos(startIndex, 2));
	mocapPos = mocapPos - coordinateShift;

	// Initial values
	Vector3f accSigma0 = {0.0167f, 0.0167f, 0.0245f}; // Std dev of accelerometer wide band noise (m/s^2)
	Vector3f accMarkov0 = {0.103f, 0.167f, 0.129f}; // Std dev of accelerometer Markov bias
	Vector3f gyroSigma0 = {0.0008727f, 0.0008727f, 0.0008727f}; // Std dev of rotation rate output noise (rad/s)
	Vector3f gyroMarkov0 = {0.0299f, 0.0316f, 0.0168f}; // Std dev of correlated rotation rate bias

	VectorXf x_interp;
	x_interp.resizeLike(imuTime);
	LinearInterp(mocapTime, mocapPos.col(0), imuTime, x_interp);
	VectorXf y_interp;
	y_interp.resizeLike(imuTime);
	LinearInterp(mocapTime, mocapPos.col(1), imuTime, y_interp);
	VectorXf z_interp;
	z_interp.resizeLike(imuTime);
	LinearInterp(mocapTime, mocapPos.col(2), imuTime, z_interp);

	write_csv("./csv/debug/interpPos_x.csv", x_interp);
	write_csv("./csv/debug/interpPos_y.csv", y_interp);
	write_csv("./csv/debug/interpPos_z.csv", z_interp);

	double progress = 0.0;
	printProgress(progress);
	for (float scaleQ = minQ; scaleQ < maxQ; scaleQ += stepQ) {

		ins.Set_AccelSigma(accSigma0*sqrt(scaleQ));
		ins.Set_AccelMarkov(accMarkov0);
		ins.Set_RotRateSigma(gyroSigma0);
		ins.Set_RotRateMarkov(gyroMarkov0);

		ins.Configure();
		ins.Initialize(imuData(0, seq(3, 5)), imuData(0, seq(0, 2)), Vector3d::Zero());

		uint64_t previousMeasUpdateTime_us = 0;
		Vector3d posMeas = {0, 0, 0};
		unsigned long tow = 0;
		Matrix<float, 15, Dynamic> outputState;

		// Main loop
		for (int imu_index = 0; imu_index < imuTime.size(); imu_index++) {
			uint64_t currentTime_us = imuTime(imu_index)*1e06;

			// Find out if it's time for a measurement update
			if ((currentTime_us - previousMeasUpdateTime_us) > 1e06) {
				int mocap_index = find(mocapTime, imuTime(imu_index));
				posMeas = mocapPos.row(mocap_index).cast <double> ();
				previousMeasUpdateTime_us = currentTime_us;
				tow += 1;
			}
			ins.Update(currentTime_us, tow, imuData(imu_index, seq(3, 5)), imuData(imu_index, seq(0, 2)), posMeas);
			Vector<float, 15> currentState = ins.Get_State();
			outputState.conservativeResize(outputState.rows(), outputState.cols() + 1);
			outputState.col(outputState.cols() - 1) = currentState;

		}

		float rmse_x = RMSE(x_interp, outputState.row(0));
		float rmse_y = RMSE(y_interp, outputState.row(1));
		float rmse_z = RMSE(z_interp, outputState.row(2));

		if (outputFile.is_open()) {
			outputFile << scaleQ << "," << rmse_x << "," << rmse_y << "," << rmse_z << std::endl;
		}

		progress = (scaleQ - minQ)/(maxQ - minQ);
		printProgress(progress);

	}

	outputFile.close();

	return 0;
}
