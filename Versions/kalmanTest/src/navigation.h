#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <iostream>

#include "eigen3/Eigen/Eigen"
typedef Eigen::Matrix<float, 15, 1> Vector15f;
typedef Eigen::Matrix<float, 15, 15> Matrix15f;
typedef Eigen::Matrix<float, 12, 12> Matrix12f;
typedef Eigen::Matrix<float, 3, 15> Matrix3_15f;

class Navigation {
public:
	Navigation() {
	}

	void initializeEKF();
	void navigationEquations();


private:
	Vector15f x = Vector15f::Zero(); // State vector (pos, vel, euler, bias_a, bias_g)
	Matrix15f P = Matrix15f::Zero(); // State error covariance matrix
	Matrix12f Q = Matrix12f::Zero(); // Process noise matrix
	Matrix3_15f H = Matrix3_15f::Zero(); // Measurement matrix
	Eigen::Matrix3f R = Eigen::Matrix3f::Zero(); // Measurement covariance matrix
	
	const float g = 9.81; // m/s^2
};

#endif
