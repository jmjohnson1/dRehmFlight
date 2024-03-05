/*
Copyright (c) 2016 - 2020 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Adapted for RAPTRS: Brian Taylor and Chris Regan

Adapted from prior versions
Copyright 2011 Regents of the University of Minnesota. All rights reserved.
Original Author: Adhika Lie, Gokhan Inalhan, Demoz Gebre, Jung Soon Jang

Reference Frames and Coordinates from nav-functions()
I - ECI (Earch Center Inertial): origin at Earth center
E - ECEF (Earch Center Earth Fixed): origin at Earth center
D - Geodetic: origin at Earth center, Uses earth ellisoid definition (example WGS84)
G - Geocentric: origin at Earth center, Uses spheroid definition
L - Local Level: origin at specified reference, [x- North, y- East, z- Down]
B - Body: origin at Body CG, [x- Fwd, y- Starboard, z- Down]

All units meters and radians
"Acceleration" is actually "specific gravity", ie. gravity is removed.
*/

#pragma once

#include <stdint.h>
#include <math.h>
#include "Eigen/Dense"
using namespace Eigen;

#include "nav-functions.h"

class uNavINS {
  public:
    uNavINS() {};
    void Configure();
    void Initialize(Vector3f wMeas_B_rps, Vector3f aMeas_B_mps2, Vector3d pMeas_NED_m);
    bool Initialized() { return initialized_; } // returns whether the INS has been initialized
    void Update(uint64_t t_us, unsigned long timeWeek, Vector3f wMeas_B_rps, Vector3f aMeas_B_mps2, Vector3d pMeas_NED_m);
    void Update(uint64_t t_us, unsigned long timeWeek, Vector3f wMeas_B_rps, Vector3f aMeas_B_mps2, Vector3d pMeas_NED_m, Vector3f vMeas_NED_mps);

    // Set Configuration
    inline void Set_AccelSigma(Vector3f val) { aNoiseSigma_mps2 = val; }
    inline void Set_AccelMarkov(Vector3f val) { aMarkovSigma_mps2 = val; }
    inline void Set_AccelTau(Vector3f val) { aMarkovTau_s = val; }
    inline void Set_RotRateSigma(Vector3f val) { wNoiseSigma_rps = val; }
    inline void Set_RotRateMarkov(Vector3f val) { wMarkovSigma_rps = val; }
    inline void Set_RotRateTau(Vector3f val) { wMarkovTau_s = val; }
    inline void Set_PosSigmaNE(float val) { pNoiseSigma_NE_m = val; }
    inline void Set_PosSigmaD(float val) { pNoiseSigma_D_m = val; }

    // Set Initial Covariance
    inline void Set_InitPosSigma(float val) { pErrSigma_Init_m = val; }
    inline void Set_InitVelSigma(float val) { vErrSigma_Init_mps = val; }
    inline void Set_InitOrientSigma(float val) { attErrSigma_Init_rad = val; }
    inline void Set_InitHeadingSigma(float val) { hdgErrSigma_Init_rad = val; }
    inline void Set_InitAccelBiasSigma(float val) { aBiasSigma_Init_mps2 = val; }
    inline void Set_InitRotRateBiasSigma(float val) { wBiasSigma_Init_rps = val; }

    // Get Navigation Estimates
    inline Vector3f Get_AccelEst() { return aEst_B_mps2_; }
    inline Vector3f Get_AccelBias() { return aBias_mps2_; }
    inline Vector3f Get_RotRateEst() { return wEst_B_rps_; }
    inline Vector3f Get_RotRateBias() { return wBias_rps_; }
    inline Vector3f Get_OrientEst() { return euler_BL_rad_; }
    inline Vector3d Get_PosEst() { return pEst_NED_m_; }
    inline Vector3f Get_VelEst() { return vEst_NED_mps_; }
    inline float Get_Track() { return atan2f(vEst_NED_mps_(1), vEst_NED_mps_(0)); }

    // Get Covariance Estimates
    inline Vector3f Get_CovPos() { return P_.block(0,0,3,3).diagonal(); }
    inline Vector3f Get_CovVel() { return P_.block(3,3,3,3).diagonal(); }
    inline Vector3f Get_CovOrient() { return P_.block(6,6,3,3).diagonal(); }
    inline Vector3f Get_CovAccelBias() { return P_.block(9,9,3,3).diagonal(); }
    inline Vector3f Get_CovRotRateBias() { return P_.block(12,12,3,3).diagonal(); }

    // Get Innovation
    inline Vector3f Get_InnovationPos() { return S_.block(0,0,3,3).diagonal(); }
    inline Vector3f Get_InnovationVel() { return S_.block(3,3,3,3).diagonal(); }

    // Get residual from measurement
    inline Vector3f Get_Residual() {return y_;}

    // Get State
    inline Vector<float, 15> Get_State() { return state_; }

  private:
    // Model Constants
    const float G = 9.807f; // Acceleration due to gravity
    const double EARTH_RADIUS = 6378137.0; // earth semi-major axis radius (m)

    // Initialize flag
    bool initialized_ = false;

    // Timing
    uint64_t tPrev_us_;
    float dt_s_;
    unsigned long timeWeekPrev_;
//Set 1
    // Sensor variances (as standard deviation) and models (tau)
    Vector3f aNoiseSigma_mps2 = {0.0016*100, 0.0016*100, 0.0021*100}; // Std dev of accelerometer wide band noise (m/s^2)
    Vector3f aMarkovSigma_mps2 = {3.993E-04*100, 5.243E-04*100, 4.660E-04*100}; // Std dev of accelerometer Markov bias
    Vector3f aMarkovTau_s = {134, 365, 44}; // Correlation time or time constant

    Vector3f wNoiseSigma_rps {8.43E-05*100, 8.42E-05*100, 7.42E-05*100}; // Std dev of rotation rate output noise (rad/s)
    Vector3f wMarkovSigma_rps = {2.56E-05*100, 3.76E-05*100, 1.40E-05*100}; // Std dev of correlated rotation rate bias
    Vector3f wMarkovTau_s = {19, 51, 201}; // Correlation time or time constant

    float pNoiseSigma_NE_m = 0.0095*10; // GPS measurement noise std dev (m)
    float pNoiseSigma_D_m = 0.0095f*10; // GPS measurement noise std dev (m)

//Set 2
    //Vector3f aNoiseSigma_mps2 = {0.123325,	0.127677,	0.0612333}; // Std dev of accelerometer wide band noise (m/s^2)
    //Vector3f aMarkovSigma_mps2 = {0.0327728,	0.00713568,	0.0463567}; // Std dev of accelerometer Markov bias
    //Vector3f aMarkovTau_s = {22.8988,	42.3737,	33.3453}; // Correlation time or time constant

    //Vector3f wNoiseSigma_rps {0.0154004,	0.0292863,	0.0405429}; // Std dev of rotation rate output noise (rad/s)
    //Vector3f wMarkovSigma_rps = {0.000178915,	0.00452792,	0.00450591}; // Std dev of correlated rotation rate bias
    //Vector3f wMarkovTau_s = {574.554,	761.428,	203.512}; // Correlation time or time constant

    //float pNoiseSigma_NE_m = 0.0422624; // GPS measurement noise std dev (m)
    //float pNoiseSigma_D_m = 0.00262705; // GPS measurement noise std dev (m)


//Set 3
    //Vector3f aNoiseSigma_mps2 = {0.0612514,	0.0691527,	0.0685713}; // Std dev of accelerometer wide band noise (m/s^2)
    //Vector3f aMarkovSigma_mps2 = {0.00362795,	0.00389026,	0.0499795}; // Std dev of accelerometer Markov bias
    //Vector3f aMarkovTau_s = {104.996,	324.841,	11.9907}; // Correlation time or time constant

    //Vector3f wNoiseSigma_rps {0.00951027,	0.00244128,	0.0497298}; // Std dev of rotation rate output noise (rad/s)
    //Vector3f wMarkovSigma_rps = {0.00241733,	8.78678e-05,	0.000339188}; // Std dev of correlated rotation rate bias
    //Vector3f wMarkovTau_s = {144.877,	866.999,	17.5336}; // Correlation time or time constant

    //float pNoiseSigma_NE_m = 0.0381/2.0; // GPS measurement noise std dev (m)
    //float pNoiseSigma_D_m = 0.0093*2.0; // GPS measurement noise std dev (m)

//Set 4 (Offset by 50 ms)
    //Vector3f aNoiseSigma_mps2 = {0.018,	0.018,	0.017}; // Std dev of accelerometer wide band noise (m/s^2)
    //Vector3f aMarkovSigma_mps2 = {0.002,	0.0018,	0.02}; // Std dev of accelerometer Markov bias
    //Vector3f aMarkovTau_s = {15,	36,	10}; // Correlation time or time constant

    //Vector3f wNoiseSigma_rps {0.01,	0.01,	0.02}; // Std dev of rotation rate output noise (rad/s)
    //Vector3f wMarkovSigma_rps = {0.0001,	0.000725,	0.0005}; // Std dev of correlated rotation rate bias
    //Vector3f wMarkovTau_s = {430,	42,	10}; // Correlation time or time constant

    //float pNoiseSigma_NE_m = 0.01; // GPS measurement noise std dev (m)
    //float pNoiseSigma_D_m = 0.01; // GPS measurement noise std dev (m)

//Set 5 
    //Vector3f aNoiseSigma_mps2 = {0.09,	0.1,	0.2}; // Std dev of accelerometer wide band noise (m/s^2)
    //Vector3f aMarkovSigma_mps2 = {0.075,	0.075,	0.075}; // Std dev of accelerometer Markov bias
    //Vector3f aMarkovTau_s = {250,	300,	200}; // Correlation time or time constant

    //Vector3f wNoiseSigma_rps {0.0022,	0.0047,	0.008}; // Std dev of rotation rate output noise (rad/s)
    //Vector3f wMarkovSigma_rps = {0.000164,	0.00019,	0.00014}; // Std dev of correlated rotation rate bias
    //Vector3f wMarkovTau_s = {75,	15,	30}; // Correlation time or time constant

    //float pNoiseSigma_NE_m = 0.01; // GPS measurement noise std dev (m)
    //float pNoiseSigma_D_m = 0.01; // GPS measurement noise std dev (m)

	
    float vNoiseSigma_NE_mps = 1.0f; // GPS measurement noise std dev (m/s)  PLACEHOLDER!
    float vNoiseSigma_D_mps = 1.0f; // GPS measurement noise std dev (m/s)

    // Initial set of covariance
    float pErrSigma_Init_m = 1.0f; // Std dev of initial position error (m)
    float vErrSigma_Init_mps = 1.0f; // Std dev of initial velocity error (m/s)
    float attErrSigma_Init_rad = 0.34906f; // Std dev of initial attitude (phi and theta) error (rad)
    float hdgErrSigma_Init_rad = 3.14159f; // Std dev of initial Heading (psi) error (rad)
    float aBiasSigma_Init_mps2 = 0.981f; // Std dev of initial acceleration bias (m/s^2)
    float wBiasSigma_Init_rps = 0.01745f; // Std dev of initial rotation rate bias (rad/s)

// 300 iter
// 0.110539000000000	0.181710000000000	0.0923450000000000	0.00105274000000000	0.000933455000000000	0.0796560000000000	125.281000000000	329.275000000000	14.7900000000000	0.00409113000000000	0.000451679000000000	0.0499230000000000	0.000199847000000000	6.40598000000000e-06	4.59359000000000e-05	995.825000000000	999.352000000000	196.458000000000	0.0330281000000000	0.00909819000000000

    // Identity matrices
    const Matrix<float,2,2> I2 = Matrix<float,2,2>::Identity();
    const Matrix<float,3,3> I3 = Matrix<float,3,3>::Identity();
    const Matrix<float,5,5> I5 = Matrix<float,5,5>::Identity();
    const Matrix<float,15,15> I15 = Matrix<float,15,15>::Identity();

    // Kalman Matrices
    Matrix<float,3,15> H_; // Observation matrix
    Matrix<float,3,3> R_;// Covariance of the Observation Noise (associated with MeasUpdate())
    Matrix<float,12,12> Rw_; // Covariance of the Sensor Noise (associated with TimeUpdate())
    Matrix<float,3,3> S_; // Innovation covariance
    Matrix<float,15,15> P_; // Covariance estimate

    Matrix<float,6,15> H2_; // Observation matrix for meas update with velocity
    Matrix<float,6,6> R2_;// Covariance of the Observation Noise (associated with MeasUpdate())
    Matrix<float,6,6> S2_; // Innovation covariance
	
    // Global variables
    Vector3f aBias_mps2_; // acceleration bias
    Vector3f wBias_rps_; // rotation rate bias
    Vector3f euler_BL_rad_; // Euler angles - B wrt L (3-2-1) [phi, theta, psi]
    Quaternionf quat_BL_; // Quaternion of B wrt L
    Vector3f aEst_B_mps2_; // Estimated acceleration in Body
    Vector3f wEst_B_rps_; // Estimated rotation rate in Body
    Vector3f vEst_NED_mps_; // Estimated velocity in NED
    Vector3d pEst_NED_m_; // Estimated position in NED

    Vector3f y_; // Difference between measurement and estimate

    // State Vector
    Vector<float, 15> state_;

    // Methods
    void TimeUpdate();
    void MeasUpdate(Vector3d pMeas_D_rrm);
		void MeasUpdate(Vector3d pMeas_NED_m, Vector3f vMeas_NED_mps);
    void UpdateStateVector();
};
