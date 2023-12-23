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
	float maxQ = 10.0f;
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
	std::string mocapPos_i_path = "./csv/mocap_pos_i.csv";
	std::string mocapTime_path = "./csv/mocap_time.csv";
	MatrixXf imuData = load_csv<MatrixXf>(imuData_path);
	MatrixXf imuTime = load_csv<MatrixXf>(imuTime_path);
	MatrixXf mocapPos = load_csv<MatrixXf>(mocapPos_i_path);
	std::cout << "imuData - rows: " << imuData.rows() << " cols: " << imuData.cols() << std::endl;
	std::cout << "imuTime - rows: " << imuTime.rows() << " cols: " << imuTime.cols() << std::endl;
	std::cout << "mocapPos - rows: " << mocapPos.rows() << " cols: " << mocapPos.cols() << std::endl;

	std::cout << "... csv data loaded." << std::endl;

	// Convert imu gyro to rad/s
	imuData.block(0, 3, imuData.rows(), 3) = imuData.block(0, 3, imuData.rows(), 3)*M_PI/180.0f;

	// Convert accelerometer data to m/s/s
	imuData.block(0, 0, imuData.rows(), 3) = imuData.block(0, 0, imuData.rows(), 3)*9.807f;

	// Initial values
	Vector3f accSigma0 = {0.0167f, 0.0167f, 0.0245f}; // Std dev of accelerometer wide band noise (m/s^2)
	Vector3f accMarkov0 = {0.103f, 0.167f, 0.129f}; // Std dev of accelerometer Markov bias
	Vector3f gyroSigma0 = {0.0008727f, 0.0008727f, 0.0008727f}; // Std dev of rotation rate output noise (rad/s)
	Vector3f gyroMarkov0 = {0.0299f, 0.0316f, 0.0168f}; // Std dev of correlated rotation rate bias
	
	int M = (maxQ - minQ)/stepQ + 1;
	int N = imuTime.size();

	double progress = 0.0;
	printProgress(progress);
	for (float scaleQ = minQ; scaleQ < (maxQ + stepQ); scaleQ += stepQ) {
		int k = (scaleQ - minQ)/stepQ + 1;

		Vector3f hotdog = { 1, 1, sqrt(scaleQ)};

		ins.Set_AccelSigma(accSigma0.cwiseProduct(hotdog));
		ins.Set_AccelMarkov(accMarkov0.cwiseProduct(hotdog));
		ins.Set_RotRateSigma(gyroSigma0);
		ins.Set_RotRateMarkov(gyroMarkov0.cwiseProduct(hotdog));

		ins.Set_PosSigmaNE(0.01f);
		ins.Set_PosSigmaD(0.01f);

		ins.Configure();
		ins.Initialize(imuData(0, seq(3, 5)), imuData(0, seq(0, 2)), Vector3d::Zero());

		uint64_t previousMeasUpdateTime_us = 0;
		Vector3d posMeas = {0, 0, 0};
		unsigned long tow = 0;
		Matrix<float, 15, Dynamic> outputState;

		// Main loop
		for (int imu_index = 0; imu_index < N; imu_index++) {
			uint64_t currentTime_us = imuTime(imu_index)*1e06;

			// Find out if it's time for a measurement update
			if ((currentTime_us - previousMeasUpdateTime_us) > 1e06) {
				posMeas = mocapPos.row(imu_index).cast <double> ();
				previousMeasUpdateTime_us = currentTime_us;
				tow += 1;
			}
			ins.Update(currentTime_us, tow, imuData(imu_index, seq(3, 5)), imuData(imu_index, seq(0, 2)), posMeas);
			Vector<float, 15> currentState = ins.Get_State();
			outputState.conservativeResize(outputState.rows(), outputState.cols() + 1);
			outputState.col(outputState.cols() - 1) = currentState;

			progress = static_cast<double>(imu_index + N*(k - 1))/static_cast<double>(N * M);
			printProgress(progress);

		}

		float rmse_x = RMSE(mocapPos.col(0), outputState.row(0));
		float rmse_y = RMSE(mocapPos.col(1), outputState.row(1));
		float rmse_z = RMSE(mocapPos.col(2), outputState.row(2));

		if (outputFile.is_open()) {
			outputFile << scaleQ << "," << rmse_x << "," << rmse_y << "," << rmse_z << std::endl;
		}
	}

	outputFile.close();

	return 0;
}
