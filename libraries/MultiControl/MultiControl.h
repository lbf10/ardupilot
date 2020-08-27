/// @file	MultiControl.h
/// @brief	Library for multicopter control

#ifndef MULTICONTROL_H
#define MULTICONTROL_H

#pragma once

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Param/AP_Param.h>
#include <PolyNavigation/PolyNavigation.h>
#include </usr/include/Eigen/Eigen>
#include </usr/include/Eigen/unsupported/Eigen/MatrixFunctions>
#include <Eigen/Sparse>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>

#define SQRT2_DIV2 0.707106781

//////////////////////////////////////////////
/* Multicopter configuration default values */
#define MASS    6.015 
#define MOMENTS_OF_INERTIA_XX 0.3143978800
#define MOMENTS_OF_INERTIA_YY 0.3122127800
#define MOMENTS_OF_INERTIA_ZZ 0.5557912400
#define PRODUCTS_OF_INERTIA_XY 0.0000861200
#define PRODUCTS_OF_INERTIA_XZ -0.0014397600
#define PRODUCTS_OF_INERTIA_YZ 0.0002368800
#define MAIN_TRANSLATIONAL_FRICTION_X 0.25
#define MAIN_TRANSLATIONAL_FRICTION_Y 0.25
#define MAIN_TRANSLATIONAL_FRICTION_Z 0.25
#define NUMBER_OF_ROTORS 8
#define ROTOR_INERTIA 0.00047935
#define ROTOR_LIFT_COEFF 6.97e-5
#define ROTOR_DRAG_COEFF 1.033e-6
#define ROTOR_MAX_SPEED 729.0
#define ROTOR_MAX_SPEED_SQUARED (double) ROTOR_MAX_SPEED*ROTOR_MAX_SPEED
#define ROTOR_MIN_SPEED 0.0
#define ROTOR_MIN_SPEED_SQUARED (double) ROTOR_MIN_SPEED*ROTOR_MIN_SPEED
#define ROTOR_OPERATING_POINT 340.0
#define ROTOR_OPERATING_POINT_SQUARED (double) 0.5*(ROTOR_MAX_SPEED_SQUARED+ROTOR_MIN_SPEED_SQUARED)
#define ROTOR_RM 0.0975
#define ROTOR_L 0.0033
#define ROTOR_KT 0.02498
#define ROTOR_KV 340.0
#define ROTOR_IO 0.6/10.0
#define ROTOR_MAX_VOLTAGE 22.0

#define ROTOR_POSITION 0.33920, -0.33742, 0.09298, \
                       0.33887, 0.33758, 0.09298, \
                       -0.33613, 0.33726, 0.09298, \
                        -0.33581, -0.33775, 0.09298, \
                        0.34364, 0.34235, 0.01597, \
                       0.34396, -0.34219, 0.01597, \
                       -0.34057, -0.34251, 0.01597, \
                        -0.34090, 0.34202, 0.01597

#define ROTOR_ORIENTATION -6.179033021212912e-02, 6.146607671042632e-02, 9.961946980917455e-01, \
                           -6.174583072871852e-02, -6.151077857998878e-02, 9.961946980917455e-01, \
                           6.152491288718258e-02, -6.173174700363311e-02, 9.961946980917455e-01, \
                          6.145065852146369e-02, 6.180566366583593e-02, 9.961946980917455e-01, \
                          -6.174419940539115e-02, -6.151241609369010e-02, 9.961946980917455e-01, \
                           -6.178718847226844e-02, 6.146923486255825e-02, 9.961946980917455e-01, \
                           6.145313940784528e-02, 6.180319693038461e-02, 9.961946980917455e-01, \
                          6.152726235862697e-02, -6.172940531504134e-02, 9.961946980917455e-01

#define ROTOR_DIRECTION 1, -1, 1, -1, 1, -1, 1, -1

//////////////////////////////////////
/* General variables default values */
#define VELOCITY_FILTER_GAIN_X 0.893982849632920  
#define VELOCITY_FILTER_GAIN_Y 0.866909075664830
#define VELOCITY_FILTER_GAIN_Z 0.713328100616703

#define CONTROL_TIME_STEP 1/400.0

////////////////////////////////////////////////
/* Position PIDD configuration default values */
#define PIDD_KP_X 20.0
#define PIDD_KP_Y 20.0
#define PIDD_KP_Z 15.0
#define PIDD_KI_X 4.0
#define PIDD_KI_Y 4.0
#define PIDD_KI_Z 4.0
#define PIDD_KD_X 20.0
#define PIDD_KD_Y 20.0
#define PIDD_KD_Z 12.0
#define PIDD_KDD_X 0.0
#define PIDD_KDD_Y 0.0
#define PIDD_KDD_Z 0.0

/////////////////////////////////////////
/* FT-LQR configuration default values */
#define FTLQR_CONFIG_P_DIAG 5.0e+10, 5.0e+10, 5.0e+05, 5.0e+04, 5.0e+04, 5.0e+09
#define FTLQR_CONFIG_Q_DIAG 5.0e+10, 5.0e+10, 5.0e+05, 5.0e+04, 5.0e+04, 5.0e+09
#define FTLQR_CONFIG_R_DIAG 2.0000e-05, 2.0000e-05, 2.0000e-05, 2.0000e-05, 2.0000e-05, 2.0000e-05, 2.0000e-05, 2.0000e-05
#define FTLQR_CONFIG_EF_ROW 1, 1, 1, 1, 1, 1
#define FTLQR_CONFIG_EG_ROW 1, 1, 1, 1, 1, 1, 1, 1
#define FTLQR_CONFIG_H_COL 1, 1, 1, 1, 1, 1

#define FTLQR_CONFIG_MU 1.0e20
#define FTLQR_CONFIG_ALPHA 1.5

//////////////////////////////////////////////////////////////////
/* Passive NMAC Control allocation configuration default values */
#define CA_WM 1.0
#define CA_WA 0.0

class MultiControl
{
    //private
public:
    ///////////////////////////////////////////
    /* Variables to define on initialization */
    std::ofstream dumpfile;
    Eigen::Quaterniond auxQuaternion;

    /* Multicopter configuration */
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _rotorPosition;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _rotorOrientation;
    Eigen::Matrix<double, Eigen::Dynamic, 1> _rotorDirection;

    // Generic
    AP_AHRS &_ahrs;
    Eigen::Matrix<double, 3, Eigen::Dynamic> _Mf; // Matrix of forces calculated from rotor positions
    Eigen::Matrix<double, 3, Eigen::Dynamic> _Mt; // Matrix of torques calculated from rotor positions and torques
    Eigen::Matrix<double, Eigen::Dynamic, 3> _pinvMt; // pseudo-inverse of Mt
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _nullMt; // Null-space of Mt
    Eigen::Matrix3d _inertia; // body inertia matrix
    double _maxSpeedsSquared; // Maximum rotor speeds squared
    double _minSpeedsSquared; // Minimum rotor speeds squared
    double _opSquared; // Midpoint operational speeds squared
    Eigen::Vector3d _weightVector; // Weight vector in the opposite direction of gravitational aceleration
    Eigen::Vector3d _velocityFilterGain; //Gain for the angular velocity filter

    // FT-LQR related
    struct ftlqrRelatedConst {
        Eigen::Matrix<double, 6, NUMBER_OF_ROTORS> ssB;
        Eigen::SparseMatrix<double> Q;
        Eigen::SparseMatrix<double> R;
        Eigen::Matrix<double, 6, 1> Ef;
    } _ftLQRConst;     

    // Position PIDD related
    struct piddRelatedConst {
        Eigen::Array3d Kp;
        Eigen::Array3d Ki;
        Eigen::Array3d Kd;
        Eigen::Array3d Kdd;
    } _piddConst;  

    PolyNavigation::state _dummyState;
    /////////////////////////////////////////
    /* Variables updated at each iteration */

    // Desired state
    Eigen::Vector3d _desiredPosition;
    Eigen::Vector3d _desiredVelocity;
    Eigen::Vector3d _desiredAcceleration;
    double _desiredYaw;
    Eigen::Quaterniond _desiredAttitude;
    Eigen::Vector3d _desiredForce;
    Eigen::Vector3d _desiredTorque;
    Eigen::Matrix<double,NUMBER_OF_ROTORS,1> _desiredRotorSpeeds;
    Eigen::Matrix<double,NUMBER_OF_ROTORS,1> _desiredRotorVoltages;

    // Current state
    Eigen::Vector3d _currentPosition;
    Eigen::Vector3d _currentVelocity;
    Eigen::Vector3d _currentAcceleration;
    Eigen::Quaterniond _currentAttitude;
    Eigen::Vector3d _currentAngularVelocity;
    Eigen::Matrix<double, NUMBER_OF_ROTORS, 1> _currentRotorSpeeds;

    // Other
    double _lastCall;
    double _controlTimeStep;
    double _measuredTimeStep;
    Eigen::Quaterniond _quaternionError;

    // Velocity filter related
    struct velocityFilterRelated {
        Eigen::Vector3d Wbe;
        Eigen::Vector3d previousAngularVelocity;
        Eigen::Vector3d desiredAngularVelocity;
    } _velFilter;

    // FT-LQR related
    struct ftlqrRelated {
        Eigen::Matrix<double,6,6> P;
        Eigen::SparseMatrix<double> left;
        Eigen::SparseMatrix<double> right;
    } _ftLQR;

    // Position PIDD related
    struct piddRelated {
        Eigen::Array3d iError;
        Eigen::Array3d error;
        Eigen::Array3d derror;
        Eigen::Array3d dderror;
    } _pidd;

    /////////////////////////
    /* Auxiliary variables */
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _attRefAux; //Auxiliary variable for attitude reference calculation. Calculated on init to reduce computation time
    Eigen::MatrixXd _attRefAux2;
    Vector3f _vectorAux;
    Quaternion _quat;
    Eigen::Quaterniond _qyd;
    Eigen::Matrix3d _Qyd; 
    Eigen::Matrix3d _invQyd;   
    Eigen::Vector3d _Tcd;
    Eigen::MatrixXd _v;
    Eigen::MatrixXd _omegaSquared;
    Eigen::Vector3d _Tc;
    Eigen::Vector3d _vCB;    
    Eigen::Vector3d _Ta;
    Eigen::Quaterniond _qCB;

    /* members */
    void matrixBtoA(const Eigen::Quaterniond& quaternion, Eigen::Ref<Eigen::Matrix3d> transformationBA);
    void swapReferenceFrames(const Eigen::Quaterniond &quatIn, Quaternion &quatOut);
    void c2d(const Eigen::Ref<const Eigen::MatrixXd>& Ac,const Eigen::Ref<const Eigen::MatrixXd>& Bc, double ts, Eigen::Ref<Eigen::MatrixXd> Ad, Eigen::Ref<Eigen::MatrixXd> Bd);
    void gainRLQR(Eigen::Ref<Eigen::MatrixXd> F, Eigen::Ref<Eigen::MatrixXd> G, Eigen::Ref<Eigen::MatrixXd> K);
public:
    // Constructor
    MultiControl();
    
    /* Do not allow copies */
    MultiControl(const MultiControl &other) = delete;
    MultiControl &operator=(const MultiControl&) = delete;

    // Destructor
    ~MultiControl();

    // Commands

    // Members
    bool init();
    bool updateStates(PolyNavigation::state desiredState);
    bool positionControl();
    bool attitudeReference();
    bool attitudeFTLQRControl();
    bool controlAllocation();

    Vector3f toEuler(Eigen::Quaterniond quat);
    Vector3f toEuler(Quaternion quat);
    
    // Variable access
    PolyNavigation::state dummyState(){return _dummyState;};
    void desiredAttitude(Quaternion &quat);
    Vector3f desiredAttitude();
    void desiredAttitudeNED(Quaternion &quat);
    Vector3f desiredAttitudeNED();
    Vector3f currentPosition();
    Vector3f currentPositionNED();
    Vector3f currentVelocity();
    Vector3f currentVelocityNED();
    Vector3f currentAcceleration();
    Vector3f currentAccelerationNED();
    void currentAttitude(Quaternion &quat);
    Vector3f currentAttitude();
    void currentAttitudeNED(Quaternion &quat);
    Vector3f currentAttitudeNED();
    Vector3f currentAngularVelocity();
    Vector3f currentAngularVelocityNED();
    float* currentRotorSpeeds();
    float* desiredRotorSpeeds();
    float* desiredRotorVoltages();
    double controlTimeStep(){return _controlTimeStep;};
    double measuredTimeStep(){return _measuredTimeStep;};
};
 
#endif