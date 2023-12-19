#include <iostream>
#include "csvParser.h"
#include "statFun.h"

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

	write_csv("./csv/imuData_debug.csv", imuData);
	write_csv("./csv/mocapPos_debug.csv", mocapPos);

	VectorXf xInterp;
	xInterp.resizeLike(imuTime);
	LinearInterp(mocapTime, mocapPos.col(0), imuTime, xInterp);

	write_csv("./csv/xInterp_test.csv", xInterp);


	return 0;
}
